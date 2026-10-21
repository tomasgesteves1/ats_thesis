#!/usr/bin/env python3
"""Compares the commands of the replayed online configuration with those recorded in the bag (replay fidelity)."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tracking_common import read_bag
from solver_benchmark import DEFAULT_BAG, RESULTS_DIR

bag = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
raw = read_bag(bag, ["/drone/ground_truth/odometry", "/mpc_states_and_inputs", "/boat/thrusters/left/thrust",
                     "/boat/thrusters/right/thrust", "/boat/thrusters/left/pos", "/boat/thrusters/right/pos"])
tb, od = raw["/drone/ground_truth/odometry"]
to_sim = lambda t: np.interp(t, tb, [m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in od])

r = np.load(os.path.join(RESULTS_DIR, "usv_online.npz"))
for i, (topic, unit, sc) in enumerate([("/boat/thrusters/left/thrust", "N", 1), ("/boat/thrusters/right/thrust", "N", 1),
                                        ("/boat/thrusters/left/pos", "deg", 57.3), ("/boat/thrusters/right/pos", "deg", 57.3)]):
    t, ms = raw[topic]
    rec = np.interp(r["t"], to_sim(t), [m.data for m in ms])
    e = sc * (r["cmd"][:, i] - rec)
    print(f"USV {topic:32s} RMS diff {np.sqrt(np.mean(e**2)):7.2f} {unit} (range {sc*rec.min():.1f}..{sc*rec.max():.1f})")

r = np.load(os.path.join(RESULTS_DIR, "uav_online.npz"))
t, ms = raw["/mpc_states_and_inputs"]
data = np.array([m.data[:25] for m in ms])
for i, (col, name, unit, sc) in enumerate([(6, "roll cmd", "deg", 57.3), (7, "pitch cmd", "deg", 57.3), (21, "yaw cmd", "deg", 57.3), (8, "thrust", "N", 1)]):
    rec = np.interp(r["t"], to_sim(t), data[:, col])
    e = sc * (r["cmd"][:, i] - rec)
    if name == "yaw cmd":
        e = sc * ((r["cmd"][:, i] - rec + np.pi) % (2 * np.pi) - np.pi)
    print(f"UAV {name:10s} RMS diff {np.sqrt(np.mean(e**2)):6.2f} {unit} (range {sc*rec.min():.1f}..{sc*rec.max():.1f})")
for v in ("usv", "uav"):
    r = np.load(os.path.join(RESULTS_DIR, f"{v}_online.npz"))
    a, b = r["recorded_ms"], r["time_ms"]
    print(f"{v} time recorded mean {a.mean():.2f} max {a.max():.2f} | replay mean {b.mean():.2f} max {b.max():.2f} ms")
