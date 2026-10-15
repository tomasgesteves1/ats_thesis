"""Web dashboard of the tethered UAV-USV system.

Serves a single page (static/) and a small JSON API on port 8080:
  GET  /api/state            telemetry, mission progress, processes and simulation statistics
  GET  /api/missions         missions available to the mission executor (YAML files)
  GET  /api/logs?name=<p>    last lines of the output of a launched process (sim, mission, bag)
  POST /api/sim/start        {"gui": bool}       launch the simulation
  POST /api/sim/stop
  POST /api/mission/start    {"id": str, "record": bool}   launch the controllers and the mission executor
  POST /api/mission/stop                        stop the mission (and the recording)
  POST /api/mission/abort                       the executor aborts: the boat holds and the UAV lands
The processes are launched in their own session and stopped as a group.
"""
import glob
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import BatteryStatus, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import WorldStatistics
from std_msgs.msg import Bool, Empty, Float64, String

LOG_DIR = '/tmp/ats_dashboard'
BAG_TOPICS = [
    '/tf', '/tf_static', '/clock', '/tether_geometry_marker', '/tether_force_markers', '/tether_force_drone_mag',
    '/tether_force_boat_mag', '/moordyn_tether_node/tether_distance', '/moordyn_tether_node/tether_length',
    '/moordyn_tether_node/tether_pull', '/uav/yaw_from_reference', '/uav/low_altitude', '/drone/ground_truth/odometry',
    '/boat/ground_truth/odometry', '/uav/reference_path', '/usv/reference_path', '/usv/mpc_horizon',
    '/mpc_predicted_path', '/mpc_states_and_inputs', '/px4_1/fmu/out/vehicle_attitude',
    '/px4_1/fmu/out/vehicle_status_v1', '/mission/events', '/intruder/odometry', '/simulation/stats']
NAV_STATES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'MISSION', 4: 'LOITER', 5: 'RTL', 14: 'OFFBOARD',
              17: 'TAKEOFF', 18: 'LAND'}


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def world_velocity(o):
    """Odometry twist (body frame) in the world frame."""
    q, v = o.pose.pose.orientation, o.twist.twist.linear
    x, y, z, w = q.x, q.y, q.z, q.w
    return (v.x * (1 - 2 * (y * y + z * z)) + v.y * 2 * (x * y - z * w) + v.z * 2 * (x * z + y * w),
            v.x * 2 * (x * y + z * w) + v.y * (1 - 2 * (x * x + z * z)) + v.z * 2 * (y * z - x * w),
            v.x * 2 * (x * z - y * w) + v.y * 2 * (y * z + x * w) + v.z * (1 - 2 * (x * x + y * y)))


class Process:
    """A launched command with its output in a log file; stopped by signalling its whole process group."""

    def __init__(self, name):
        self.name, self.proc, self.started, self.cmd = name, None, None, []
        self.log = os.path.join(LOG_DIR, f'{name}.log')

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, cmd):
        self.stop()
        os.makedirs(LOG_DIR, exist_ok=True)
        out = open(self.log, 'w')
        self.proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        self.started, self.cmd = time.time(), cmd

    def stop(self, timeout=10.0):
        if not self.running():
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout)
                return
            except subprocess.TimeoutExpired:
                continue

    def tail(self, n=60):
        try:
            with open(self.log, errors='replace') as f:
                return ''.join(deque(f, maxlen=n))
        except OSError:
            return ''

    def info(self):
        return {'running': self.running(), 'since': self.started if self.running() else None}


class DashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard_node')
        self.workspace = self.declare_parameter('workspace', os.path.expanduser('~/ats_ws')).value
        self.lock = threading.Lock()
        self.procs = {n: Process(n) for n in ('sim', 'mission', 'bag')}
        self.mission = None          # id of the launched mission
        self.bag_path = None
        self.t = {'uav': {}, 'usv': {}, 'tether': {}, 'intruder': None, 'sim': {}, 'paths': {}}
        self.events = deque(maxlen=200)
        self.steps = {'uav': '', 'usv': ''}
        self.mission_state = 'idle'

        sub = self.create_subscription
        sub(Odometry, '/drone/ground_truth/odometry', lambda m: self._odom('uav', m), 10)
        sub(Odometry, '/boat/ground_truth/odometry', lambda m: self._odom('usv', m), 10)
        sub(Odometry, '/intruder/odometry', self._intruder, 10)
        sub(Float64, '/moordyn_tether_node/tether_length', lambda m: self._set('tether', 'length', m.data), 10)
        sub(Float64, '/moordyn_tether_node/tether_distance', lambda m: self._set('tether', 'distance', m.data), 10)
        sub(Float64, '/tether_force_drone_mag', lambda m: self._set('tether', 'force_uav', m.data), 10)
        sub(Bool, '/moordyn_tether_node/tether_pull', lambda m: self._set('tether', 'pull', m.data), 10)
        sub(VehicleStatus, '/px4_1/fmu/out/vehicle_status_v1', self._status, qos_profile_sensor_data)
        sub(BatteryStatus, '/px4_1/fmu/out/battery_status', self._battery, qos_profile_sensor_data)
        sub(WorldStatistics, '/simulation/stats', self._stats, 10)
        sub(String, '/mission/events', self._event, 50)
        sub(Path, '/uav/reference_path', lambda m: self._path('uav', m), 10)
        sub(Path, '/usv/reference_path', lambda m: self._path('usv', m), 10)
        self.abort_pub = self.create_publisher(Empty, '/mission/abort', 10)

    # ------------------------------------------------------------------------------------------------- telemetry
    def _set(self, group, key, value):
        with self.lock:
            self.t[group][key] = value
            self.t[group]['stamp'] = time.time()

    def _odom(self, who, m):
        p = m.pose.pose.position
        v = world_velocity(m)
        with self.lock:
            self.t[who].update({'x': p.x, 'y': p.y, 'z': p.z + (0.265 if who == 'uav' else 0.0),
                                'speed': math.hypot(v[0], v[1]), 'vz': v[2], 'surge': m.twist.twist.linear.x,
                                'yaw': yaw_deg(m.pose.pose.orientation), 'stamp': time.time()})

    def _intruder(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.t['intruder'] = {'x': p.x, 'y': p.y, 'yaw': yaw_deg(m.pose.pose.orientation),
                                  'speed': m.twist.twist.linear.x, 'stamp': time.time()}

    def _status(self, m):
        with self.lock:
            self.t['uav'].update({'armed': m.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                  'mode': NAV_STATES.get(m.nav_state, f'MODE_{m.nav_state}')})

    def _battery(self, m):
        with self.lock:
            self.t['uav']['battery'] = max(0.0, m.remaining * 100.0)

    def _stats(self, m):
        with self.lock:
            self.t['sim'] = {'time': m.sim_time.sec + m.sim_time.nanosec * 1e-9, 'rtf': m.real_time_factor,
                             'paused': m.paused, 'stamp': time.time()}

    def _path(self, who, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses[::5]]
        with self.lock:
            self.t['paths'][who] = pts

    def _event(self, m):
        text = m.data
        with self.lock:
            sim_t = self.t['sim'].get('time')
            self.events.appendleft({'t': sim_t, 'text': text})
            for v in ('uav', 'usv', 'uav_abort', 'usv_abort'):
                if text.startswith(v + '.') and ': started' in text:
                    self.steps[v.split('_')[0]] = text.split(':')[0].split('.', 1)[1] + \
                        (' (abort)' if v.endswith('abort') else '')
            if text == 'mission complete':
                self.mission_state = 'complete'
            elif text.startswith('mission aborted'):
                self.mission_state = 'aborted'
            elif text.startswith('abort sequence complete'):
                self.mission_state = 'aborted (UAV landed)'

    # ------------------------------------------------------------------------------------------------- missions
    def missions_dir(self):
        return os.path.join(get_package_share_directory('mission_executor'), 'missions')

    def list_missions(self):
        out = []
        for f in sorted(glob.glob(os.path.join(self.missions_dir(), '*.yaml'))):
            try:
                d = yaml.safe_load(open(f)) or {}
            except yaml.YAMLError:
                continue
            steps = lambda key: [s.get('type', '?') for s in d.get(key) or []]
            out.append({'id': os.path.splitext(os.path.basename(f))[0], 'name': d.get('name', ''),
                        'description': d.get('description', ''), 'uav': steps('uav'), 'usv': steps('usv'),
                        'launch': d.get('launch') or {}, 'timeout': d.get('timeout')})
        return out

    def start_sim(self, gui):
        self.stop_mission()
        self.procs['sim'].start(['ros2', 'launch', 'bringup', 'moordyn_marsupial.launch.py',
                                 f'headless:={"false" if gui else "true"}', 'use_tether:=true'])

    def stop_sim(self):
        self.stop_mission()
        self.procs['sim'].stop()
        # PX4 and the Gazebo server may outlive the launch: stop the leftovers
        for pattern in ('px4_sitl_default/bin/px4', 'gz sim -s', 'gz sim -r', 'MicroXRCEAgent'):
            subprocess.run(['pkill', '-9', '-f', pattern], check=False)
        with self.lock:
            self.t = {'uav': {}, 'usv': {}, 'tether': {}, 'intruder': None, 'sim': {}, 'paths': {}}

    def start_mission(self, mission_id, record):
        missions = {m['id']: m for m in self.list_missions()}
        if mission_id not in missions:
            raise ValueError(f'unknown mission {mission_id}')
        if not self.procs['sim'].running():
            raise ValueError('the simulation is not running')
        self.stop_mission()
        args = [f'{k}:={str(v).lower() if isinstance(v, bool) else v}' for k, v in missions[mission_id]['launch'].items()]
        with self.lock:
            self.events.clear()
            self.steps = {'uav': '', 'usv': ''}
            self.mission_state = 'running'
        self.mission = mission_id
        if record:
            self.bag_path = os.path.join(self.workspace, 'bags', 'missions',
                                         f'{mission_id}_{datetime.now():%Y%m%d_%H%M%S}')
            os.makedirs(os.path.dirname(self.bag_path), exist_ok=True)
            self.procs['bag'].start(['ros2', 'bag', 'record', '--use-sim-time', '-o', self.bag_path] + BAG_TOPICS)
        else:
            self.bag_path = None
        self.procs['mission'].start(['ros2', 'launch', 'mission_executor', 'mission.launch.py',
                                     f'mission:={mission_id}'] + args)

    def stop_mission(self):
        self.procs['mission'].stop()
        self.procs['bag'].stop(timeout=20.0)   # the recorder needs a moment to write the bag metadata
        with self.lock:
            if self.mission_state == 'running':
                self.mission_state = 'stopped'

    def state(self):
        with self.lock:
            return {'telemetry': self.t, 'events': list(self.events)[:40], 'steps': self.steps,
                    'mission': self.mission, 'mission_state': self.mission_state, 'bag': self.bag_path,
                    'processes': {n: p.info() for n, p in self.procs.items()}, 'now': time.time()}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        node, url = self.server.node, urlparse(self.path)
        if url.path == '/api/state':
            return self._json(node.state())
        if url.path == '/api/missions':
            return self._json(node.list_missions())
        if url.path == '/api/logs':
            name = parse_qs(url.query).get('name', ['sim'])[0]
            proc = node.procs.get(name)
            return self._json({'name': name, 'text': proc.tail() if proc else ''})
        # static files
        rel = 'index.html' if url.path in ('/', '') else url.path.lstrip('/')
        path = os.path.realpath(os.path.join(self.server.static_dir, rel))
        if not path.startswith(os.path.realpath(self.server.static_dir)) or not os.path.isfile(path):
            self.send_error(404)
            return
        ctype = {'.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css'}.get(
            os.path.splitext(path)[1], 'application/octet-stream')
        with open(path, 'rb') as f:
            body = f.read()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        node = self.server.node
        length = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(length) or b'{}')
        except json.JSONDecodeError:
            data = {}
        try:
            if self.path == '/api/sim/start':
                node.start_sim(bool(data.get('gui', False)))
            elif self.path == '/api/sim/stop':
                node.stop_sim()
            elif self.path == '/api/mission/start':
                node.start_mission(data.get('id', ''), bool(data.get('record', False)))
            elif self.path == '/api/mission/stop':
                node.stop_mission()
            elif self.path == '/api/mission/abort':
                node.abort_pub.publish(Empty())
            else:
                return self._json({'ok': False, 'error': 'unknown endpoint'}, 404)
        except ValueError as e:
            return self._json({'ok': False, 'error': str(e)}, 400)
        return self._json({'ok': True})


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        static_dir = os.path.join(get_package_share_directory('web_dashboard'), 'static')
    except Exception:
        static_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'static'))
    port = node.declare_parameter('port', 8080).value
    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    server.node, server.static_dir = node, static_dir
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'Dashboard on http://localhost:{port}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for p in node.procs.values():
            p.stop()
        server.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
