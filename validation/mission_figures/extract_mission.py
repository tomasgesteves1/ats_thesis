#!/usr/bin/env python3
"""Extracts the UAV, USV and intruder trajectories, the first point of the UAV reference and the mission events of a
mission bag to an npz. Usage: extract_mission.py <bag> <out.npz>"""

import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B, OUT = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
def yaw(q): return np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
tr = {'d': [], 'b': [], 'i': [], 'r': []}; ev = []
keys = {'/drone/ground_truth/odometry': 'd', '/boat/ground_truth/odometry': 'b', '/intruder/odometry': 'i'}
while r.has_next():
    tp, data, t = r.read_next(); t *= 1e-9
    if tp in keys:
        m = deserialize_message(data, get_message(types[tp])); p = m.pose.pose.position
        tr[keys[tp]].append((t, p.x, p.y, p.z, yaw(m.pose.pose.orientation)))
    elif tp == '/uav/reference_path':
        m = deserialize_message(data, get_message(types[tp]))
        if m.poses:
            p = m.poses[0].pose.position; tr['r'].append((t, p.x, p.y, p.z, 0.0))
    elif tp == '/mission/events':
        ev.append((t, deserialize_message(data, get_message(types[tp])).data))
np.savez(OUT, d=np.array(tr['d']), b=np.array(tr['b']), i=np.array(tr['i']) if tr['i'] else np.zeros((0, 5)),
         r=np.array(tr['r']), ev_t=np.array([e[0] for e in ev]), ev=np.array([e[1] for e in ev]))
t0 = ev[0][0] if ev else 0
for t, e in ev: print(f'{t-t0:7.1f} {e}')
