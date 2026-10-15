#!/usr/bin/env python3
"""
Shared helpers to evaluate the tracking of the USV and UAV NMPC controllers from a rosbag2 recording made with
validation/record_tracking.sh: bag reading, wall-to-simulation time mapping, resampling and the controller limits
(Tables 4.11 and 4.12 of the thesis) against which the constraints are checked.
"""
import os
import glob
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

VALIDATION_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(VALIDATION_DIR, "..", ".."))

# --- Controller limits (Table 4.11 -- USV, Table 4.12 -- UAV) ---------------------------------------------------
USV_LIMITS = dict(T_max=510.0, T_min=-380.0, alpha_max=np.deg2rad(30.0), alpha_dot_max=np.deg2rad(10.0),
                  T_dot_max=500.0, d_max=15.0, z_t=1.25, Ts=0.1)
UAV_LIMITS = dict(z_min=2.0, v_xy_max=10.0, v_z_max=10.0, phi_max=0.5, theta_max=0.5, phi_dot_max=0.8,
                  theta_dot_max=0.8, psi_dot_max=1.0, T_min=2.02, T_max=32.96, d_max=15.0, d_dot_max=2.0,
                  mass=2.06, gravity=9.81, Ts=0.02)

# Layout of /mpc_states_and_inputs (uav_mpc_node)
UAV_TELEMETRY = {"x": 0, "y": 1, "z": 2, "roll_cmd": 6, "pitch_cmd": 7, "thrust": 8, "roll": 9, "pitch": 10, "yaw": 11,
                 "ref_x": 12, "ref_y": 13, "ref_z": 14, "vx": 15, "vy": 16, "vz": 17, "ref_vx": 18, "ref_vy": 19,
                 "ref_vz": 20, "yaw_cmd": 21}


def latest_bag(pattern):
    """Most recent bag in <workspace>/bags/tracking matching the glob pattern (e.g. 'tracking_reachable_*')."""
    bags = sorted(glob.glob(os.path.join(WORKSPACE_DIR, "bags", "tracking", pattern)))
    if not bags:
        raise FileNotFoundError(f"no bag matching {pattern} in {WORKSPACE_DIR}/bags/tracking")
    return bags[-1]


def read_bag(bag_path, topics):
    """Returns {topic: (t_bag [s], [messages])} for the requested topics."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path)),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {t: get_message(types[t]) for t in topics if t in types}
    out = {t: ([], []) for t in msg_types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in msg_types:
            out[topic][0].append(stamp * 1e-9)
            out[topic][1].append(deserialize_message(data, msg_types[topic]))
    return {t: (np.array(ts), ms) for t, (ts, ms) in out.items()}


def odometry_arrays(times, msgs):
    """t_sim, position (N,3), quaternion (N,4: x y z w), body linear velocity (N,3) from nav_msgs/Odometry."""
    t_sim = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in msgs])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                   m.pose.pose.orientation.w] for m in msgs])
    v = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in msgs])
    return t_sim, p, q, v


def euler_zyx(q):
    """(yaw, pitch, roll) with yaw unwrapped, from quaternions (x y z w)."""
    yaw, pitch, roll = R.from_quat(q).as_euler("ZYX").T
    return np.unwrap(yaw), pitch, roll


R_ENU_TO_NED = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_FLU_TO_FRD = R.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def px4_attitude_to_enu_euler(q_wxyz):
    """Roll, pitch, yaw (ROS ENU/FLU convention) from PX4 quaternions [w, x, y, z] (NED/FRD)."""
    r = R_ENU_TO_NED * R.from_quat(np.asarray(q_wxyz)[:, [1, 2, 3, 0]]) * R_FLU_TO_FRD
    yaw, pitch, roll = r.as_euler("ZYX").T
    return roll, pitch, np.unwrap(yaw)


def bag_to_sim_time(t_bag, odom_t_bag, odom_t_sim):
    """Maps bag (wall) time to simulation time using the odometry header stamps."""
    return np.interp(t_bag, odom_t_bag, odom_t_sim)


def resample(t, y, t_grid):
    y = np.asarray(y)
    if y.ndim == 1:
        return np.interp(t_grid, t, y)
    return np.column_stack([np.interp(t_grid, t, y[:, i]) for i in range(y.shape[1])])


def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def violation(value, lower=None, upper=None):
    """Largest violation of [lower, upper] (0 if inside) and fraction of the samples outside."""
    v = np.zeros_like(value, dtype=float)
    if lower is not None:
        v = np.maximum(v, lower - value)
    if upper is not None:
        v = np.maximum(v, value - upper)
    return float(v.max()), float((v > 1e-9).mean())


def tidy_limits(fig, show_limits=False, margin=0.5):
    """Drops the dashed constraint lines that are far from the data of an axis, so that they do not flatten the
    curves (the limit is then noted in the axis). Lines are recognised by the style used for limits (dashed red);
    show_limits=True keeps all of them."""
    import matplotlib.pyplot as plt  # noqa: F401
    from matplotlib.colors import to_hex
    for ax in fig.axes:
        limits = [l for l in ax.lines if l.get_linestyle() == "--" and to_hex(l.get_color()) == "#d62728"]
        data = [l for l in ax.lines if l not in limits and len(l.get_ydata()) > 2]
        if not limits or not data or show_limits:
            continue
        y = np.concatenate([np.asarray(l.get_ydata(), dtype=float) for l in data])
        y = y[np.isfinite(y)]
        lo, hi = y.min(), y.max()
        span = max(hi - lo, 1e-9)
        removed = []
        for l in limits:
            v = float(np.asarray(l.get_ydata())[0])
            if v < lo - margin * span or v > hi + margin * span:
                removed.append(v)
                l.remove()
        if removed:
            leg = ax.get_legend()
            ncol = getattr(leg, "_ncols", 1) if leg is not None else None
            if leg is not None:
                leg.remove()
                if ax.get_legend_handles_labels()[0]:
                    ax.legend(ncol=ncol)
            vals = ", ".join(f"{v:g}" for v in sorted(set(removed)))
            ax.text(0.01, 0.02, f"limit {vals} not shown", transform=ax.transAxes, fontsize=7, color="#d62728", va="bottom")
        ax.relim(); ax.autoscale_view()
