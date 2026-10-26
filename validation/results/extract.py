#!/usr/bin/env python3
"""
Extracts the signals used by the figures of Chapter 5 from a tracking or mission bag into an npz, so that the bag
(hundreds of MB) is read once. Times are simulation times (header stamps of the odometry; the other topics are mapped
from the bag time through the odometry). Usage: extract.py <bag> <out.npz>
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DRONE_Z_OFFSET = 0.265   # odometry origin to the point tracked by the controller
PRED_EVERY = 0.5         # s between stored predicted horizons of the UAV


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    return (np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)))


def main(bag, out):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msg = {t: get_message(ty) for t, ty in types.items()}
    scalars = ['/tether_force_drone_mag', '/tether_force_boat_mag', '/moordyn_tether_node/tether_length',
               '/moordyn_tether_node/tether_distance', '/uav_solve_time_ms', '/usv_solve_time_ms',
               '/virtual_tether_distance', '/virtual_tether_limit', '/virtual_tether_separation_speed',
               '/boat/thrusters/left/thrust', '/boat/thrusters/right/thrust', '/boat/thrusters/left/pos',
               '/boat/thrusters/right/pos']
    data = {k: [] for k in ['drone', 'boat', 'intruder', 'uav_ref', 'usv_ref', 'tel', 'dist', 'ext', 'pull', 'att', 'att_sp']}
    sc = {k: [] for k in scalars}
    ev, pred, map_bag, map_sim = [], [], [], []
    last_pred = -1e9
    while r.has_next():
        tp, raw, tb = r.read_next()
        tb *= 1e-9
        if tp not in msg:
            continue
        if tp in ('/drone/ground_truth/odometry', '/boat/ground_truth/odometry', '/intruder/odometry'):
            m = deserialize_message(raw, msg[tp])
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, v = m.pose.pose.position, m.twist.twist.linear
            # t, x, y, z, yaw, vx, vy, vz (body axes, as published), roll, pitch
            row = (ts, p.x, p.y, p.z, yaw_of(m.pose.pose.orientation), v.x, v.y, v.z, *roll_pitch_of(m.pose.pose.orientation))
            key = {'/drone/ground_truth/odometry': 'drone', '/boat/ground_truth/odometry': 'boat'}.get(tp, 'intruder')
            data[key].append(row)
            if key == 'drone':
                map_bag.append(tb); map_sim.append(ts)
        elif tp in ('/uav/reference_path', '/usv/reference_path'):
            m = deserialize_message(raw, msg[tp])
            if m.poses:
                p = m.poses[0].pose
                ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                data['uav_ref' if tp.startswith('/uav') else 'usv_ref'].append(
                    (ts, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))
        elif tp == '/mpc_states_and_inputs':
            data['tel'].append((tb, *deserialize_message(raw, msg[tp]).data))
        elif tp == '/mpc_predicted_path':
            if tb - last_pred >= PRED_EVERY:
                m = deserialize_message(raw, msg[tp])
                pred.append((tb, np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses])))
                last_pred = tb
        elif tp == '/disturbance_estimate':
            m = deserialize_message(raw, msg[tp]); data['dist'].append((tb, m.x, m.y, m.z))
        elif tp == '/moordyn_tether_node/external_force_drone':
            m = deserialize_message(raw, msg[tp]); data['ext'].append((tb, m.x, m.y, m.z))
        elif tp == '/moordyn_tether_node/tether_pull':
            data['pull'].append((tb, float(deserialize_message(raw, msg[tp]).data)))
        elif tp == '/mission/events':
            ev.append((tb, deserialize_message(raw, msg[tp]).data))
        elif tp in ('/px4_1/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude_setpoint'):
            m = deserialize_message(raw, msg[tp])
            if tp.endswith('attitude'):
                data['att'].append((tb, *m.q))
            else:   # t, q_d [w, x, y, z], normalised throttle (thrust_body[2] is its negative)
                data['att_sp'].append((tb, *m.q_d, -m.thrust_body[2]))
        elif tp in sc:
            sc[tp].append((tb, deserialize_message(raw, msg[tp]).data))

    map_bag, map_sim = np.array(map_bag), np.array(map_sim)
    to_sim = lambda t: np.interp(t, map_bag, map_sim)
    out_d = {}
    for k, rows in data.items():
        a = np.array(rows, dtype=float) if rows else np.zeros((0, 2))
        if len(a) and k in ('tel', 'dist', 'ext', 'pull', 'att', 'att_sp'):
            a[:, 0] = to_sim(a[:, 0])
        out_d[k] = a
    if len(out_d['drone']):
        out_d['drone'][:, 3] += DRONE_Z_OFFSET
    for k, rows in sc.items():
        a = np.array(rows, dtype=float) if rows else np.zeros((0, 2))
        if len(a):
            a[:, 0] = to_sim(a[:, 0])
        out_d['s' + k.replace('/', '_')] = a
    out_d['ev_t'] = to_sim(np.array([e[0] for e in ev])) if ev else np.zeros(0)
    out_d['ev'] = np.array([e[1] for e in ev])
    out_d['pred_t'] = to_sim(np.array([p[0] for p in pred])) if pred else np.zeros(0)
    out_d['pred'] = np.array([p[1] for p in pred], dtype=object)
    np.savez(out, **out_d)
    print(f'{out}: {len(out_d["drone"])} odometry samples, {len(ev)} events')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
