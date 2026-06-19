import os
import sys
import time
import json
import threading
import subprocess
import signal
import http.server
import socketserver
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS message types
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from px4_msgs.msg import BatteryStatus, VehicleStatus

# Thread-safe telemetry storage
class TelemetryData:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            'uav': {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                'yaw': 0.0,
                'armed': False,
                'nav_state': 'STANDBY',
                'battery_percent': 100.0,
                'battery_voltage': 16.8
            },
            'usv': {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                'yaw': 0.0
            },
            'tether': {
                'length': 0.0,
                'distance': 0.0,
                'tension_boat': 0.0,
                'tension_drone': 0.0
            }
        }

    def update_uav_odom(self, msg: Odometry):
        with self.lock:
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            self.data['uav']['x'] = p.x
            self.data['uav']['y'] = p.y
            self.data['uav']['z'] = p.z
            self.data['uav']['vx'] = v.x
            self.data['uav']['vy'] = v.y
            self.data['uav']['vz'] = v.z
            # Basic Euler yaw extraction
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            import math
            self.data['uav']['yaw'] = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

    def update_usv_odom(self, msg: Odometry):
        with self.lock:
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            self.data['usv']['x'] = p.x
            self.data['usv']['y'] = p.y
            self.data['usv']['z'] = p.z
            self.data['usv']['vx'] = v.x
            self.data['usv']['vy'] = v.y
            self.data['usv']['vz'] = v.z
            # Basic Euler yaw extraction
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            import math
            self.data['usv']['yaw'] = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

    def update_tether_length(self, msg: Float64):
        with self.lock:
            self.data['tether']['length'] = msg.data

    def update_tether_distance(self, msg: Float64):
        with self.lock:
            self.data['tether']['distance'] = msg.data

    def update_tether_force_boat(self, msg: Float64):
        with self.lock:
            self.data['tether']['tension_boat'] = msg.data

    def update_tether_force_drone(self, msg: Float64):
        with self.lock:
            self.data['tether']['tension_drone'] = msg.data

    def update_battery(self, msg: BatteryStatus):
        with self.lock:
            self.data['uav']['battery_percent'] = float(msg.remaining * 100.0) if msg.remaining >= 0 else 0.0
            self.data['uav']['battery_voltage'] = float(msg.voltage_v)

    def update_uav_status(self, msg: VehicleStatus):
        with self.lock:
            # arming_state: 1 is INIT/DISARMED, 2 is ARMED
            self.data['uav']['armed'] = (msg.arming_state == 2)
            
            # Map navigation states to names
            nav_states = {
                0: 'MANUAL',
                1: 'ALTCTL',
                2: 'POSCTL',
                3: 'AUTO_MISSION',
                4: 'AUTO_LOITER',
                5: 'AUTO_RTL',
                10: 'ACRO',
                14: 'OFFBOARD',
                17: 'TAKEOFF',
                18: 'LAND',
            }
            self.data['uav']['nav_state'] = nav_states.get(msg.nav_state, f'MODE_{msg.nav_state}')

    def get_json(self):
        with self.lock:
            return json.dumps(self.data)

class DashboardHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    # Suppress request logging to avoid console noise
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        node = self.server.node
        static_dir = self.server.static_dir

        if self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            sim_running = node.processes['simulation'] is not None and node.processes['simulation'].poll() is None
            mission_running = node.processes['mission'] is not None and node.processes['mission'].poll() is None
            
            status = {
                'simulation_running': sim_running,
                'mission_running': mission_running,
                'active_mission': node.active_mission_name if mission_running else 'None'
            }
            self.wfile.write(json.dumps(status).encode('utf-8'))
            return

        elif self.path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            node.get_logger().info("Dashboard: Telemetry SSE client connected.")
            try:
                while not self.server.stop_requested:
                    telemetry_str = node.telemetry.get_json()
                    self.wfile.write(f"data: {telemetry_str}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.1) # 10 Hz updates
            except (ConnectionResetError, BrokenPipeError):
                node.get_logger().info("Dashboard: Telemetry SSE client disconnected.")
            return

        # Serve static files
        clean_path = self.path.split('?')[0]
        if clean_path == '/':
            filepath = os.path.join(static_dir, 'index.html')
        else:
            # Strip leading slash
            filepath = os.path.join(static_dir, clean_path.lstrip('/'))

        if os.path.exists(filepath) and os.path.isfile(filepath):
            # Check content type
            if filepath.endswith('.html'):
                content_type = 'text/html'
            elif filepath.endswith('.css'):
                content_type = 'text/css'
            elif filepath.endswith('.js'):
                content_type = 'application/javascript'
            elif filepath.endswith('.png'):
                content_type = 'image/png'
            elif filepath.endswith('.svg'):
                content_type = 'image/svg+xml'
            else:
                content_type = 'text/plain'

            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            with open(filepath, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, 'File Not Found')

    def do_POST(self):
        node = self.server.node
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        
        try:
            params = json.loads(body.decode('utf-8'))
        except Exception:
            params = {}

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        response = {'success': False, 'message': 'Unknown endpoint'}

        if self.path == '/api/launch':
            proc_type = params.get('type')
            if proc_type == 'simulation':
                sim_type = params.get('sim_type', 'cooperative')
                if sim_type == 'individual':
                    cmd = "ros2 launch bringup boat.launch.py"
                else:
                    use_tether = params.get('use_tether', True)
                    tether_str = 'true' if use_tether else 'false'
                    cmd = f"ros2 launch bringup moordyn_marsupial.launch.py use_tether:={tether_str}"
                
                if node.processes['simulation'] is not None and node.processes['simulation'].poll() is None:
                    response = {'success': False, 'message': 'Simulation is already running.'}
                else:
                    node.get_logger().info(f"Launching Simulation: {cmd}")
                    # Launch in a new process group so we can terminate it and all child processes cleanly
                    proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    node.processes['simulation'] = proc
                    response = {'success': True, 'message': 'Simulation launched successfully.'}
                    
            elif proc_type == 'mission':
                mission_name = params.get('name')
                if mission_name == 'circle':
                    cmd = "ros2 launch bringup control_circle.launch.py"
                elif mission_name == 'marsupial':
                    cmd = "ros2 launch bringup control_marsupial.launch.py"
                elif mission_name == 'boat_mpc':
                    cmd = "ros2 launch usv_mpc usv_mpc.launch.py"
                else:
                    cmd = None

                if not cmd:
                    response = {'success': False, 'message': f"Invalid mission name: {mission_name}"}
                elif node.processes['mission'] is not None and node.processes['mission'].poll() is None:
                    response = {'success': False, 'message': 'A mission is already running. Stop it first.'}
                else:
                    node.get_logger().info(f"Launching Mission '{mission_name}': {cmd}")
                    proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    node.processes['mission'] = proc
                    node.active_mission_name = mission_name
                    response = {'success': True, 'message': f"Mission '{mission_name}' launched successfully."}

        elif self.path == '/api/stop':
            proc_type = params.get('type')
            if proc_type in ['simulation', 'mission']:
                proc = node.processes[proc_type]
                if proc is not None and proc.poll() is None:
                    node.get_logger().info(f"Stopping {proc_type} (PID {proc.pid})...")
                    try:
                        # Kill the entire process group
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        # Wait for process exit
                        for _ in range(30):
                            if proc.poll() is not None:
                                break
                            time.sleep(0.1)
                        if proc.poll() is None:
                            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                        
                        node.processes[proc_type] = None
                        if proc_type == 'mission':
                            node.active_mission_name = 'None'
                            
                        # Extra cleanup for simulation
                        if proc_type == 'simulation':
                            node.get_logger().info("Running extra cleanup for Gazebo, PX4, and Micro-XRCE-DDS...")
                            subprocess.run("pkill -9 -f 'gz sim' || true", shell=True)
                            subprocess.run("pkill -9 -f 'ruby' || true", shell=True)
                            subprocess.run("pkill -9 -f 'px4' || true", shell=True)
                            subprocess.run("pkill -9 -f 'micro-xrce-dds' || true", shell=True)
                            
                        response = {'success': True, 'message': f"{proc_type.capitalize()} stopped successfully."}
                    except Exception as e:
                        response = {'success': False, 'message': f"Failed to kill {proc_type}: {str(e)}"}
                else:
                    response = {'success': False, 'message': f"{proc_type.capitalize()} is not running."}

        elif self.path == '/api/emergency_stop':
            # Stop everything
            node.get_logger().warn("EMERGENCY STOP REQUESTED!")
            stopped = []
            for name, proc in node.processes.items():
                if proc is not None and proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        node.processes[name] = None
                        stopped.append(name)
                    except Exception:
                        pass
            # Force kill all simulation components
            subprocess.run("pkill -9 -f 'gz sim' || true", shell=True)
            subprocess.run("pkill -9 -f 'ruby' || true", shell=True)
            subprocess.run("pkill -9 -f 'px4' || true", shell=True)
            subprocess.run("pkill -9 -f 'micro-xrce-dds' || true", shell=True)
            node.active_mission_name = 'None'
            response = {'success': True, 'message': f"Emergency stop executed. Stopped: {', '.join(stopped) if stopped else 'None'}"}

        self.wfile.write(json.dumps(response).encode('utf-8'))

class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True

class DashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard_node')
        
        # Telemetry Data
        self.telemetry = TelemetryData()
        
        # Track subprocesses
        self.processes = {
            'simulation': None,
            'mission': None
        }
        self.active_mission_name = 'None'

        # Subscriptions
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.uav_odom_sub = self.create_subscription(
            Odometry,
            '/drone/ground_truth/odometry',
            self.telemetry.update_uav_odom,
            10
        )
        
        self.usv_odom_sub = self.create_subscription(
            Odometry,
            '/boat/ground_truth/odometry',
            self.telemetry.update_usv_odom,
            10
        )
        
        self.tether_length_sub = self.create_subscription(
            Float64,
            '/moordyn_tether_node/tether_length',
            self.telemetry.update_tether_length,
            10
        )

        self.tether_distance_sub = self.create_subscription(
            Float64,
            '/moordyn_tether_node/tether_distance',
            self.telemetry.update_tether_distance,
            10
        )
        
        self.tether_force_boat_sub = self.create_subscription(
            Float64,
            '/moordyn_tether_node/tether_force_boat_mag',
            self.telemetry.update_tether_force_boat,
            10
        )
        
        self.tether_force_drone_sub = self.create_subscription(
            Float64,
            '/moordyn_tether_node/tether_force_drone_mag',
            self.telemetry.update_tether_force_drone,
            10
        )

        # PX4 telemetry
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/px4_1/fmu/out/battery_status',
            self.telemetry.update_battery,
            qos_best_effort
        )

        self.uav_status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status_v1',
            self.telemetry.update_uav_status,
            qos_best_effort
        )

        self.get_logger().info("Web Dashboard Node Initialized.")

    def cleanup_processes(self):
        # Kill any remaining subprocesses upon shutdown
        for name, proc in self.processes.items():
            if proc is not None and proc.poll() is None:
                self.get_logger().warn(f"Cleaning up zombie process: {name}")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        # Global force kill of any leftover simulation components
        subprocess.run("pkill -9 -f 'gz sim' || true", shell=True)
        subprocess.run("pkill -9 -f 'ruby' || true", shell=True)
        subprocess.run("pkill -9 -f 'px4' || true", shell=True)
        subprocess.run("pkill -9 -f 'micro-xrce-dds' || true", shell=True)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    # Get the share directory to locate the static files
    try:
        share_dir = get_package_share_directory('web_dashboard')
        static_dir = os.path.join(share_dir, 'static')
    except Exception:
        # Fallback to local source path in workspace during direct testing
        static_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'static'))

    node.get_logger().info(f"Serving static files from: {static_dir}")

    # Set up HTTP Server
    port = 8080
    server = ThreadingHTTPServer(('0.0.0.0', port), DashboardHTTPRequestHandler)
    server.node = node
    server.static_dir = static_dir
    server.stop_requested = False

    node.get_logger().info(f"Dashboard GCS Web Server running at http://localhost:{port}/")

    # Spin ROS 2 in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Serve requests in the main thread (blocking)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        server.stop_requested = True
        server.shutdown()
        server.server_close()
        node.cleanup_processes()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
