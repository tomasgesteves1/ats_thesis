#!/usr/bin/env python3
"""
Compares the UAV tracking of several controllers (NMPC, NMPC without the tether model, PX4 position controller) on the
same scenario, from bags recorded with validation/record_tracking.sh. The error is measured in the same way for all of
them, from the ground truth (raised by drone_z_offset to the point tracked by the NMPC) against the published
reference, since the PX4 baseline has no NMPC telemetry.

Usage: python3 compare_controllers.py <bag> [<bag> ...] [--skip SECONDS] [--start-altitude M]
"""
import argparse
import os
import sys
import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))
from tracking_common import read_bag, odometry_arrays, rms

DRONE_Z_OFFSET = 0.265   # odometry origin to the point tracked by the controller (frame_manager drone_z_offset)
TOPICS = ["/drone/ground_truth/odometry", "/uav/reference_path"]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("bags", nargs="+")
    p.add_argument("--skip", type=float, default=15.0, help="seconds discarded after the circle starts")
    p.add_argument("--start-altitude", type=float, default=28.0, help="altitude at which the circle starts (m)")
    return p.parse_args()


def evaluate(bag, skip, start_altitude):
    raw = read_bag(bag, TOPICS)
    t_d, p_d, _, _ = odometry_arrays(*raw["/drone/ground_truth/odometry"])
    p_d = p_d + np.array([0.0, 0.0, DRONE_Z_OFFSET])
    _, paths = raw["/uav/reference_path"]
    t_r = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in paths])
    p_r = np.array([[m.poses[0].pose.position.x, m.poses[0].pose.position.y, m.poses[0].pose.position.z] for m in paths])

    t0 = t_d[np.argmax(p_d[:, 2] - DRONE_Z_OFFSET > start_altitude)] + skip
    m = (t_d >= t0) & (t_d <= t_r[-1])
    t, pos = t_d[m] - t0, p_d[m]
    ref = np.column_stack([np.interp(t_d[m], t_r, p_r[:, k]) for k in range(3)])
    return t, pos - ref


def main():
    a = parse_args()
    runs = {os.path.basename(b.rstrip("/")): evaluate(b, a.skip, a.start_altitude) for b in a.bags}
    T = min(t[-1] for t, _ in runs.values())   # same duration for every run
    print("UAV tracking over %.0f s (after %.0f s from the start of the circle)" % (T, a.skip))
    print("%-30s %8s %8s %8s %8s %8s" % ("run", "RMSE", "max", "x", "y", "z"))
    for name, (t, e) in runs.items():
        e = e[t <= T]; n = np.linalg.norm(e, axis=1)
        print("%-30s %7.1fcm %7.1fcm %7.1fcm %7.1fcm %7.1fcm" % (name, 100 * rms(n), 100 * n.max(),
              100 * rms(e[:, 0]), 100 * rms(e[:, 1]), 100 * rms(e[:, 2])))


if __name__ == "__main__":
    main()
