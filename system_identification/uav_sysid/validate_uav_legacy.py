#!/usr/bin/env python3
"""
UAV Dynamics Validation Script (Tethered UAV-USV Project)
Validates the control-oriented x500 quadrotor model used by the UAV NMPC
(translational point-mass dynamics driven by thrust and attitude, plus the
closed-form tether force) against Gazebo/PX4 ground truth from rosbag recordings.

The model is evaluated with the first-principles nominal parameters of the
simulator (no parameter fitting). The only identified quantities are diagnostic:
the attitude response lag, and the tether anchor offset on the deck.

Validation layers:
  1. Attitude tracking   - does the achieved attitude follow the commanded one?
  2. Thrust mapping      - does normalised thrust map to force via m*g/hover?
  3. Acceleration level  - one-step-ahead model vs. measured acceleration.
  4. Multi-step horizon  - open-loop prediction over the NMPC horizon (N*Ts).
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d
from scipy.optimize import minimize, least_squares
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R

from rosbags.highlevel import AnyReader

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SYSID_DIR = os.path.dirname(CURRENT_DIR)
if SYSID_DIR not in sys.path:
    sys.path.append(SYSID_DIR)

from thesis_style import (
    set_thesis_style,
    save_figure,
    GROUND_TRUTH_COLOR,
    COLOR_CYCLE,
)

# Nominal physical parameters (model.sdf of the x500, Table 4.x of the thesis)
MASS = 2.06            # kg: 2.0 kg main body + 4 rotors of 0.016 kg
G = 9.81               # m/s^2

# Tether (Section "Tether Force Model"): mu = 0.020 kg/m, eps0 = 0.05, gamma0 = 28 deg
MU_TETHER = 0.020
EPS0 = 0.05
GAMMA0 = np.deg2rad(28.0)
C_D = MU_TETHER * G / (1.0 - EPS0)

# UAV NMPC prediction horizon (acados_generator_config.json): N = 50, Ts = 0.02 s
TS = 0.02
N_HORIZON = 50
PHI_DOT_MAX = 0.8      # rad/s, attitude rate bound used by the NMPC
FLIGHT_START_Z = 5.5   # m, the analysed window starts when the UAV first reaches this altitude

TOPICS = [
    '/drone/ground_truth/odometry',
    '/boat/ground_truth/odometry',
    '/px4_1/fmu/in/vehicle_attitude_setpoint',   # NMPC bags (command sent to PX4)
    '/px4_1/fmu/out/vehicle_attitude_setpoint',  # PX4 position-controller bags (command it generates)
    '/px4_1/fmu/out/hover_thrust_estimate',
    '/moordyn_tether_node/tether_distance',
    '/moordyn_tether_node/tether_length',
    '/tether_force_drone_mag',
    '/world/wamv_world/wrench',
]

# Constant rotations between the ENU/FLU (ROS) and NED/FRD (PX4) conventions
R_ENU_TO_NED = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_FLU_TO_FRD = R.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def parse_args():
    parser = argparse.ArgumentParser(description="Validate UAV model against rosbag telemetry.")
    default_bag = os.path.abspath(os.path.join(CURRENT_DIR, "../../../bags/uav_px4_offboard_20261005_213544"))
    parser.add_argument("--bag", type=str, default=default_bag, help="Path to ROS 2 bag folder")
    parser.add_argument("--skip", type=float, default=14.0,
                        help="Seconds skipped at the start (hover before the offboard setpoints take over)")
    parser.add_argument("--no-export", dest="export", action="store_false",
                        help="Do not copy figures to the thesis directory")
    parser.set_defaults(export=True)
    return parser.parse_args()


# ------------------------------------------------------------------------------
# Data loading
# ------------------------------------------------------------------------------
def read_bag(bag_path):
    print(f"[INFO] Reading telemetry from: {bag_path}")
    raw = {k: [] for k in TOPICS}
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, stamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            t_w = stamp / 1e9
            tp = conn.topic
            if tp.endswith('ground_truth/odometry'):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                raw[tp].append([t_w, t_s, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z])
            elif tp.endswith('vehicle_attitude_setpoint'):
                raw[tp].append([t_w, *msg.q_d, msg.thrust_body[2]])
            elif tp.endswith('hover_thrust_estimate'):
                if msg.valid:
                    raw[tp].append([t_w, msg.hover_thrust])
            elif tp.endswith('/wrench'):
                if 'x500' in msg.entity.name:      # tether force applied to the UAV link
                    f = msg.wrench.force
                    raw[tp].append([t_w, f.x, f.y, f.z])
            else:
                raw[tp].append([t_w, msg.data])
    return {k: np.array(v) for k, v in raw.items()}


def build_dataset(raw, skip=0.0):
    """Resample every signal onto a uniform 50 Hz simulation-time grid."""
    d_odom = raw['/drone/ground_truth/odometry']
    b_odom = raw['/boat/ground_truth/odometry']
    sp = raw['/px4_1/fmu/out/vehicle_attitude_setpoint']
    if len(sp) == 0:
        sp = raw['/px4_1/fmu/in/vehicle_attitude_setpoint']
    hov = raw['/px4_1/fmu/out/hover_thrust_estimate']

    # Wall clock -> simulation clock, through the odometry header stamps
    wall2sim = interp1d(d_odom[:, 0], d_odom[:, 1], fill_value='extrapolate')
    t_sp = wall2sim(sp[:, 0])
    t_hov = wall2sim(hov[:, 0])
    t_tether = wall2sim(raw['/moordyn_tether_node/tether_length'][:, 0])

    # Free-flight window only: skip the ground phase and the climb to the hover altitude,
    # where the model (no ground contact, no takeoff transient) does not apply
    t_fly = d_odom[np.argmax(d_odom[:, 4] >= FLIGHT_START_Z), 1]
    t_start = max(d_odom[0, 1], b_odom[0, 1], t_sp[0], t_hov[0], t_tether[0], t_fly)
    t_start += skip
    t_end = min(d_odom[-1, 1], b_odom[-1, 1], t_sp[-1], t_hov[-1], t_tether[-1])
    t_grid = np.arange(t_start, t_end, TS)
    t = t_grid - t_start

    def on_grid(ts, vals):
        return interp1d(ts, vals, axis=0, fill_value='extrapolate')(t_grid)

    # --- UAV ground truth ---
    p = on_grid(d_odom[:, 1], d_odom[:, 2:5])
    q = on_grid(d_odom[:, 1], d_odom[:, 5:9])
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rot = R.from_quat(q)
    v_body = on_grid(d_odom[:, 1], d_odom[:, 9:12])
    v = rot.apply(v_body)                      # odometry twist is in the body frame
    yaw, pitch, roll = rot.as_euler('ZYX').T   # thesis convention: ZYX (yaw-pitch-roll)

    # --- Commanded attitude and thrust (PX4 NED/FRD setpoint -> ENU/FLU) ---
    q_d = sp[:, 1:5]                           # [w, x, y, z]
    r_cmd = R_ENU_TO_NED * R.from_quat(q_d[:, [1, 2, 3, 0]]) * R_FLU_TO_FRD
    yaw_c, pitch_c, roll_c = r_cmd.as_euler('ZYX').T
    thr = -sp[:, 5]
    yaw_c = np.unwrap(yaw_c)
    cmd = on_grid(t_sp, np.column_stack([roll_c, pitch_c, yaw_c, thr]))
    hover = on_grid(t_hov, hov[:, 1])
    thrust = MASS * G * cmd[:, 3] / hover      # N, same mapping as the NMPC node

    # --- USV ground truth and tether telemetry ---
    pb = on_grid(b_odom[:, 1], b_odom[:, 2:5])
    qb = on_grid(b_odom[:, 1], b_odom[:, 5:9])
    qb /= np.linalg.norm(qb, axis=1, keepdims=True)
    rot_b = R.from_quat(qb)
    tdist = on_grid(t_tether, raw['/moordyn_tether_node/tether_distance'][:, 1])
    tlen = on_grid(t_tether, raw['/moordyn_tether_node/tether_length'][:, 1])
    tforce = on_grid(wall2sim(raw['/tether_force_drone_mag'][:, 0]),
                     raw['/tether_force_drone_mag'][:, 1])
    wr = raw['/world/wamv_world/wrench']
    f_true = on_grid(wall2sim(wr[:, 0]), wr[:, 1:4])

    return {
        't': t, 'dt': TS,
        'p': p, 'v': v, 'rot': rot,
        'roll': roll, 'pitch': pitch, 'yaw': np.unwrap(yaw),
        'roll_c': cmd[:, 0], 'pitch_c': cmd[:, 1], 'yaw_c': cmd[:, 2],
        'thr_norm': cmd[:, 3], 'hover': hover, 'T': thrust,
        'pb': pb, 'rot_b': rot_b,
        'tether_dist': tdist, 'tether_len': tlen, 'tether_force': tforce, 'F_true': f_true,
    }


# ------------------------------------------------------------------------------
# Tether anchor and force model
# ------------------------------------------------------------------------------
def identify_anchor_offset(data):
    """Tether anchor offset in the USV body frame, from the logged anchor distance.
    The anchor TF is not recorded, so it is recovered by least squares."""
    def residual(o):
        anchor = data['pb'] + data['rot_b'].apply(o)
        return np.linalg.norm(anchor - data['p'], axis=1) - data['tether_dist']
    sol = least_squares(residual, x0=np.array([0.0, 0.0, 1.0]))
    return sol.x, np.sqrt(np.mean(sol.fun ** 2))


def tether_force(p, p_v):
    """Closed-form tether force at the UAV (thesis Eqs. tether geometry -> force)."""
    r = p_v - p
    d = np.linalg.norm(r, axis=-1, keepdims=True)
    d0 = r / d
    horiz = d0.copy()
    horiz[..., 2] = 0.0
    eh = horiz / np.maximum(np.linalg.norm(horiz, axis=-1, keepdims=True), 1e-9)
    alpha = np.arcsin(np.clip(-d0[..., 2:3], -1.0, 1.0))      # depression angle
    gamma = GAMMA0 * np.cos(alpha)
    ang = alpha + gamma
    dhat = np.cos(ang) * eh
    dhat[..., 2:3] += -np.sin(ang)
    return C_D * d * dhat


# ------------------------------------------------------------------------------
# UAV model (control-oriented, thesis Eq. uav_dynamics)
# ------------------------------------------------------------------------------
def rotation_zyx(phi, theta, psi):
    return R.from_euler('ZYX', np.column_stack([psi, theta, phi])).as_matrix()


def thrust_accel(T, phi, theta, psi):
    """(T/m) R(phi,theta,psi) e3, i.e. the third column of the ZYX rotation matrix."""
    cps, sps = np.cos(psi), np.sin(psi)
    cth, sth = np.cos(theta), np.sin(theta)
    cph, sph = np.cos(phi), np.sin(phi)
    return (T / MASS) * np.array([cps * sth * cph + sps * sph,
                                  sps * sth * cph - cps * sph,
                                  cth * cph])


def predict_window(k0, n, data, att, use_tether, T, drag=0.0):
    """RK4 open-loop integration from the measured state at sample k0, n steps ahead.
    att = (phi, theta, psi) arrays driving the model. Returns states [p, v] (n+1, 6)."""
    phi, theta, psi = att
    pb_anchor = data['p_anchor']
    dt = data['dt']
    g_vec = np.array([0.0, 0.0, -G])

    def f(s, k_a, k_b, w):
        # linear interpolation of the inputs between samples k_a and k_b
        ph = (1 - w) * phi[k_a] + w * phi[k_b]
        th = (1 - w) * theta[k_a] + w * theta[k_b]
        ps = (1 - w) * psi[k_a] + w * psi[k_b]
        Tn = (1 - w) * T[k_a] + w * T[k_b]
        a = thrust_accel(Tn, ph, th, ps) + g_vec
        if use_tether:
            pv = (1 - w) * pb_anchor[k_a] + w * pb_anchor[k_b]
            a = a + tether_force(s[:3], pv) / MASS
        if drag:
            a = a - drag * np.array([s[3], s[4], 0.0])
        return np.concatenate([s[3:], a])

    s = np.zeros((n + 1, 6))
    s[0] = np.concatenate([data['p'][k0], data['v'][k0]])
    for i in range(n):
        k = k0 + i
        k1 = f(s[i], k, k + 1, 0.0)
        k2 = f(s[i] + 0.5 * dt * k1, k, k + 1, 0.5)
        k3 = f(s[i] + 0.5 * dt * k2, k, k + 1, 0.5)
        k4 = f(s[i] + dt * k3, k, k + 1, 1.0)
        s[i + 1] = s[i] + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return s


# ------------------------------------------------------------------------------
# Metrics
# ------------------------------------------------------------------------------
def compute_metrics(y_true, y_pred):
    err = y_true - y_pred
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    ss_res = np.sum(err ** 2)
    return {
        'rmse': np.sqrt(np.mean(err ** 2)),
        'mae': np.mean(np.abs(err)),
        'max': np.max(np.abs(err)),
        'r2': 100.0 * (1.0 - ss_res / ss_tot) if ss_tot > 1e-9 else 0.0,
    }


# ------------------------------------------------------------------------------
# 1. Attitude tracking
# ------------------------------------------------------------------------------
def simulate_lag(tau, delay, cmd, y0, dt):
    """First-order lag with pure delay: y_dot = (cmd(t - delay) - y) / tau (exact ZOH step)."""
    k_d = int(round(delay / dt))
    cmd_d = np.concatenate([np.full(k_d, cmd[0]), cmd])[:len(cmd)]
    a = np.exp(-dt / tau)
    y = np.zeros_like(cmd)
    y[0] = y0
    for k in range(len(cmd) - 1):
        y[k + 1] = a * y[k] + (1 - a) * cmd_d[k]
    return y


def identify_attitude_lag(meas, cmd, dt):
    def cost(x):
        tau, delay = x
        if tau < 1e-3 or delay < 0.0:
            return 1e6
        return np.mean((meas - simulate_lag(tau, delay, cmd, meas[0], dt)) ** 2)
    best = min((minimize(cost, x0=[tau0, 0.02], method='Nelder-Mead') for tau0 in (0.05, 0.15, 0.4)),
               key=lambda r: r.fun)
    tau, delay = best.x
    return tau, delay, simulate_lag(tau, delay, cmd, meas[0], dt)


# ------------------------------------------------------------------------------
# Plotting
# ------------------------------------------------------------------------------
def plot_inputs(data, att_fit, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)
    axs[0].plot(t, np.rad2deg(data['roll_c']), color=COLOR_CYCLE[0], label=r'Roll $\phi_c$', linewidth=1.3)
    axs[0].plot(t, np.rad2deg(data['pitch_c']), color=COLOR_CYCLE[1], label=r'Pitch $\theta_c$', linewidth=1.3)
    axs[0].set_ylabel('Attitude cmd [deg]')
    axs[0].legend(loc='upper right', framealpha=0.9, ncol=2)
    axs[1].plot(t, np.rad2deg(data['yaw_c']), color=COLOR_CYCLE[2], label=r'Yaw $\psi_c$', linewidth=1.3)
    axs[1].set_ylabel('Yaw cmd [deg]')
    axs[2].plot(t, data['T'], color=COLOR_CYCLE[0], linewidth=1.3, label='Commanded thrust $T$')
    axs[2].axhline(MASS * G, color='r', linestyle=':', alpha=0.6, label='Weight $mg$')
    axs[2].set_ylabel('Thrust [N]')
    axs[2].set_xlabel('Time [s]')
    axs[2].legend(loc='upper right', framealpha=0.9, ncol=2)
    for ax in axs:
        ax.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_actuator_inputs'), save_png=True)
    plt.close(fig)


def plot_attitude(data, att_fit, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=True)
    for ax, key, name, sym in zip(axs, ('roll', 'pitch', 'yaw'), ('Roll', 'Pitch', 'Yaw'),
                                  ('\\phi', '\\theta', '\\psi')):
        ax.plot(t, np.rad2deg(data[key + '_c']), color=COLOR_CYCLE[1], linewidth=1.2,
                linestyle='-.', label=f'Command ${sym}_c$')
        ax.plot(t, np.rad2deg(data[key]), color=GROUND_TRUTH_COLOR, linewidth=1.4,
                label='Ground Truth')
        ax.plot(t, np.rad2deg(att_fit[key]['sim']), color=COLOR_CYCLE[0], linewidth=1.4,
                linestyle='--',
                label=f"Lag model ($\\tau={att_fit[key]['tau']:.3f}$ s, "
                      f"$t_d={att_fit[key]['delay']*1e3:.0f}$ ms)")
        ax.set_ylabel(f'{name} [deg]')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', framealpha=0.9)
    axs[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_attitude_validation'), save_png=True)
    plt.close(fig)


def plot_acceleration(data, a_meas, a_model, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)
    for i, (ax, name) in enumerate(zip(axs, ('x', 'y', 'z'))):
        ax.plot(t, a_meas[:, i], color=GROUND_TRUTH_COLOR, linewidth=1.3, label='Ground Truth')
        ax.plot(t, a_model[:, i], color=COLOR_CYCLE[0], linewidth=1.3, linestyle='--', label='Model')
        ax.set_ylabel(f'$\\ddot{{{name}}}$ [m/s$^2$]')
        ax.grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right', framealpha=0.9, ncol=2)
    axs[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_acceleration_validation'), save_png=True)
    plt.close(fig)


def plot_prediction_error(horizon_t, curves, out_dirs):
    set_thesis_style()
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 3.6))
    styles = ['-', '--', '-.', ':']
    for i, (label, c) in enumerate(curves.items()):
        col = COLOR_CYCLE[i]
        axs[0].plot(horizon_t, c['pos_rmse'], color=col, linestyle=styles[i % 4], linewidth=1.5, label=label)
        axs[1].plot(horizon_t, c['vel_rmse'], color=col, linestyle=styles[i % 4], linewidth=1.5, label=label)
    axs[0].set_ylabel('Position error RMSE [m]')
    axs[1].set_ylabel('Velocity error RMSE [m/s]')
    for ax in axs:
        ax.set_xlabel('Prediction horizon [s]')
        ax.grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper left', framealpha=0.9)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_prediction_error'), save_png=True)
    plt.close(fig)


def plot_prediction_examples(data, windows, out_dirs):
    set_thesis_style()
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 3.8), gridspec_kw={'width_ratios': [1.2, 1]})
    axs[0].plot(data['p'][:, 0], data['p'][:, 1], color=GROUND_TRUTH_COLOR, linewidth=1.3, label='Ground Truth')
    for j, (k0, s) in enumerate(windows):
        axs[0].plot(s[:, 0], s[:, 1], color=COLOR_CYCLE[0], linewidth=2.0,
                    label='1 s predictions' if j == 0 else None)
        axs[0].plot(s[0, 0], s[0, 1], 'o', color=COLOR_CYCLE[0], markersize=3)
    axs[0].set_xlabel('$x$ [m]')
    axs[0].set_ylabel('$y$ [m]')
    axs[0].axis('equal')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='best', framealpha=0.9)

    # Zoom on the window with the largest horizontal speed
    k0, s = max(windows, key=lambda w: np.linalg.norm(data['v'][w[0], :2]))
    n = len(s) - 1
    tt = np.arange(n + 1) * data['dt']
    for i, (name, col) in enumerate(zip(('x', 'y'), COLOR_CYCLE[:2])):
        axs[1].plot(tt, data['p'][k0:k0 + n + 1, i] - data['p'][k0, i], color=GROUND_TRUTH_COLOR,
                    linewidth=1.4, label='Ground Truth' if i == 0 else None)
        axs[1].plot(tt, s[:, i] - s[0, i], color=col, linewidth=1.4, linestyle='--', label=f'Model ${name}$')
    axs[1].set_xlabel('Time since window start [s]')
    axs[1].set_ylabel('Displacement [m]')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='best', framealpha=0.9)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_prediction_examples'), save_png=True)
    plt.close(fig)


# ------------------------------------------------------------------------------
def main():
    args = parse_args()
    raw = read_bag(args.bag)
    data = build_dataset(raw, args.skip)
    t, dt = data['t'], data['dt']
    N = len(t)

    # Tether anchor on the USV deck (not logged; recovered from the anchor distance)
    offset, anchor_rmse = identify_anchor_offset(data)
    data['p_anchor'] = data['pb'] + data['rot_b'].apply(offset)
    F_tether = tether_force(data['p'], data['p_anchor'])
    F_true = data['F_true']
    m_tether = [compute_metrics(F_true[:, i], F_tether[:, i]) for i in range(3)]
    m_tether_mag = compute_metrics(np.linalg.norm(F_true, axis=1), np.linalg.norm(F_tether, axis=1))
    m_tether_3d = np.sqrt(np.mean(np.sum((F_true - F_tether) ** 2, axis=1)))

    dist = np.linalg.norm(data['p_anchor'] - data['p'], axis=1)
    eps_meas = (data['tether_len'] - dist) / data['tether_len']

    # --- 1. Attitude tracking ---------------------------------------------------
    att_fit = {}
    for key in ('roll', 'pitch', 'yaw'):
        tau, delay, sim = identify_attitude_lag(data[key], data[key + '_c'], dt)
        att_fit[key] = {'tau': tau, 'delay': delay, 'sim': sim,
                        'm_lag': compute_metrics(np.rad2deg(data[key]), np.rad2deg(sim)),
                        'm_ideal': compute_metrics(np.rad2deg(data[key]), np.rad2deg(data[key + '_c']))}
    rate_cmd = {k: np.abs(np.gradient(data[k + '_c'], t)) for k in ('roll', 'pitch', 'yaw')}
    rate_meas = {k: np.abs(np.gradient(data[k], t)) for k in ('roll', 'pitch', 'yaw')}

    # --- 2. Thrust mapping ------------------------------------------------------
    win = 11  # 0.22 s Savitzky-Golay window
    a_meas = savgol_filter(data['v'], win, 2, deriv=1, delta=dt, axis=0)
    g_vec = np.array([0.0, 0.0, -G])
    z_b = data['rot'].apply(np.array([0.0, 0.0, 1.0]))
    att_meas = (data['roll'], data['pitch'], data['yaw'])
    att_cmd = (data['roll_c'], data['pitch_c'], data['yaw_c'])

    # Thrust acceleration delivered, implied by the measured motion (gravity only removed)
    a_bz = np.einsum('ij,ij->i', a_meas - g_vec, z_b)
    tc = data['T'] / MASS
    s_fit = np.sum(a_bz * tc) / np.sum(tc * tc)
    m_thrust = compute_metrics(a_bz, tc)
    # Same, but also removing the tether force logged by the coupling node
    a_bz_t = np.einsum('ij,ij->i', a_meas - g_vec - F_true / MASS, z_b)
    s_fit_t = np.sum(a_bz_t * tc) / np.sum(tc * tc)
    m_thrust_t = compute_metrics(a_bz_t, tc)

    # --- 3. Acceleration level ----------------------------------------------------
    def accel(att, tether, drag=0.0):
        a = np.array([thrust_accel(data['T'][k], att[0][k], att[1][k], att[2][k])
                      for k in range(N)]) + g_vec
        if tether:
            a = a + F_tether / MASS
        if drag:
            a[:, :2] -= drag * data['v'][:, :2]
        return a

    # Linear drag diagnostic: horizontal residual of the tether-free model vs. velocity
    a_base = accel(att_meas, False)
    res_xy = (a_meas - a_base)[:, :2]
    v_xy = data['v'][:, :2]
    k_drag = np.sum(res_xy * -v_xy) / np.sum(v_xy ** 2)
    drag_r2 = 100.0 * (1.0 - np.sum((res_xy + k_drag * v_xy) ** 2) / np.sum(res_xy ** 2))
    # Effective tether gain acting on the UAV, jointly with drag: res = alpha * F_logged - k m v
    y = (MASS * res_xy).T.reshape(-1)
    X = np.column_stack([F_true[:, :2].T.reshape(-1), -MASS * v_xy.T.reshape(-1)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    alpha_eff, k_joint = coef

    a_models = {
        'Meas. att., no tether': a_base,
        'Meas. att., tether': accel(att_meas, True),
        'Cmd. att., no tether': accel(att_cmd, False),
        'Cmd. att., tether': accel(att_cmd, True),
        'Cmd. att., no tether, drag': accel(att_cmd, False, k_drag),
    }
    a_model = a_models['Meas. att., no tether']
    acc_metrics = {k: [compute_metrics(a_meas[:, i], v[:, i]) for i in range(3)] for k, v in a_models.items()}

    # --- 4. Multi-step open-loop prediction over the NMPC horizon ----------------
    variants = {
        'Meas. att., no tether': (att_meas, False, data['T'], 0.0),
        'Meas. att., tether': (att_meas, True, data['T'], 0.0),
        'Cmd. att., no tether': (att_cmd, False, data['T'], 0.0),
        'Cmd. att., tether': (att_cmd, True, data['T'], 0.0),
        'Cmd. att., no tether, drag': (att_cmd, False, data['T'], k_drag),
    }
    n_h = N_HORIZON
    starts = np.arange(0, N - n_h - 1, 5)   # a new window every 0.1 s
    curves, final_err, windows = {}, {}, []
    for name, (att, use_t, T_in, drag) in variants.items():
        err_p = np.zeros((len(starts), n_h + 1, 3))
        err_v = np.zeros((len(starts), n_h + 1, 3))
        for j, k0 in enumerate(starts):
            s = predict_window(k0, n_h, data, att, use_t, T_in, drag)
            err_p[j] = s[:, :3] - data['p'][k0:k0 + n_h + 1]
            err_v[j] = s[:, 3:] - data['v'][k0:k0 + n_h + 1]
            if name == 'Cmd. att., no tether' and k0 % 250 == 0:
                windows.append((k0, s))
        pos_norm = np.linalg.norm(err_p, axis=2)
        vel_norm = np.linalg.norm(err_v, axis=2)
        curves[name] = {
            'pos_rmse': np.sqrt(np.mean(pos_norm ** 2, axis=0)),
            'vel_rmse': np.sqrt(np.mean(vel_norm ** 2, axis=0)),
        }
        final_err[name] = {
            'pos_rmse': curves[name]['pos_rmse'][-1],
            'pos_p95': np.percentile(pos_norm[:, -1], 95),
            'pos_max': pos_norm[:, -1].max(),
            'xy_rmse': np.sqrt(np.mean(np.sum(err_p[:, -1, :2] ** 2, axis=1))),
            'z_rmse': np.sqrt(np.mean(err_p[:, -1, 2] ** 2)),
            'vel_rmse': curves[name]['vel_rmse'][-1],
        }

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------
    dx = np.diff(data['p'][:, 0]); dy = np.diff(data['p'][:, 1])
    dist_travelled = np.sum(np.hypot(dx, dy))
    speed = np.linalg.norm(data['v'], axis=1)
    dist = np.linalg.norm(data['p_anchor'] - data['p'], axis=1)
    print("\n" + "=" * 78)
    print("          UAV DYNAMICS FIRST-PRINCIPLES VALIDATION REPORT")
    print("=" * 78)
    print(f"Duration: {t[-1]:.2f} s | Horizontal distance: {dist_travelled:.2f} m | "
          f"z in [{data['p'][:,2].min():.2f}, {data['p'][:,2].max():.2f}] m | "
          f"speed max {speed.max():.2f} m/s, max |vz| {np.abs(data['v'][:,2]).max():.2f} m/s")
    print(f"Roll/pitch cmd max: {np.rad2deg(np.abs(data['roll_c']).max()):.1f} / "
          f"{np.rad2deg(np.abs(data['pitch_c']).max()):.1f} deg | yaw cmd +-{np.rad2deg(np.abs(data['yaw_c']).max()):.1f} deg | "
          f"thrust range [{data['T'].min():.2f}, {data['T'].max():.2f}] N (mg = {MASS*G:.2f} N)")
    print(f"Hover thrust estimate: {data['hover'].mean():.4f}")
    print("-" * 78)
    print("[1] ATTITUDE TRACKING (cmd -> measured)")
    for key in ('roll', 'pitch', 'yaw'):
        f = att_fit[key]
        print(f"  {key:<5}: tau = {f['tau']:.4f} s, delay = {f['delay']*1e3:.1f} ms | "
              f"lag model RMSE {f['m_lag']['rmse']:.3f} deg R2 {f['m_lag']['r2']:.2f}% | "
              f"ideal (y=cmd) RMSE {f['m_ideal']['rmse']:.3f} deg R2 {f['m_ideal']['r2']:.2f}% "
              f"max {f['m_ideal']['max']:.2f} deg")
    for k in ('roll', 'pitch', 'yaw'):
        print(f"  |{k}_cmd rate| max {np.rad2deg(rate_cmd[k].max()):.1f} deg/s "
              f"(p99 {np.rad2deg(np.percentile(rate_cmd[k], 99)):.1f}) | "
              f"measured max {np.rad2deg(rate_meas[k].max()):.1f} deg/s"
              + (f" | roll/pitch bound {np.rad2deg(PHI_DOT_MAX):.1f} deg/s" if k != 'yaw' else ""))
    print("-" * 78)
    print("[2] THRUST MAPPING  (thrust acceleration implied by motion vs commanded T/m)")
    print(f"  mean implied {np.mean(a_bz):.4f} m/s^2 | mean T/m {np.mean(tc):.4f} m/s^2 | gain s = {s_fit:.4f} | "
          f"RMSE {m_thrust['rmse']:.4f} m/s^2, R2 {m_thrust['r2']:.1f}%")
    print(f"  if logged tether force is removed first: gain s = {s_fit_t:.4f}, RMSE {m_thrust_t['rmse']:.4f}, "
          f"R2 {m_thrust_t['r2']:.1f}%  (mean logged pull {-F_true[:,2].mean():.3f} N)")
    print("-" * 78)
    print("[3] ACCELERATION LEVEL (RMSE m/s^2 / R2 %)")
    for name, ms in acc_metrics.items():
        print(f"  {name:<30}: " + " | ".join(
            f"{ax}: {m['rmse']:.3f} / {m['r2']:.1f}" for ax, m in zip('xyz', ms)))
    print(f"  Linear drag: k = {k_drag:.4f} 1/s (explains {drag_r2:.1f}% of the horizontal residual of the tether-free model)")
    print(f"  Joint fit res = alpha*F_logged - k*m*v : alpha = {alpha_eff:.3f}, k = {k_joint:.4f} 1/s")
    print("-" * 78)
    print("[T] TETHER (closed-form model vs force logged by the coupling node)")
    print(f"  Anchor offset (USV body): {np.round(offset, 3)} m, distance fit RMSE {anchor_rmse*1e3:.2f} mm")
    print(f"  d in [{dist.min():.2f}, {dist.max():.2f}] m, slack eps in [{eps_meas.min():.3f}, {eps_meas.max():.3f}]"
          f" (mean {eps_meas.mean():.3f})")
    print("  logged F mean " + str(np.round(F_true.mean(0), 3)) + " N | model F mean " + str(np.round(F_tether.mean(0), 3)) + " N")
    print("  " + " | ".join(f"F{ax}: RMSE {m['rmse']:.3f} N, MAE {m['mae']:.3f}, R2 {m['r2']:.1f}%" for ax, m in zip('xyz', m_tether)) +
          f" | 3D RMSE {m_tether_3d:.3f} N | |F| RMSE {m_tether_mag['rmse']:.3f} N "
          f"| tension/weight {np.linalg.norm(F_true, axis=1).mean()/(MASS*G)*100:.1f}%")
    print("-" * 78)
    print(f"[4] OPEN-LOOP PREDICTION OVER THE NMPC HORIZON ({n_h*dt:.1f} s, {len(starts)} windows)")
    for name, e in final_err.items():
        print(f"  {name:<28}: pos RMSE {e['pos_rmse']:.3f} m (xy {e['xy_rmse']:.3f}, z {e['z_rmse']:.3f}), "
              f"p95 {e['pos_p95']:.3f}, max {e['pos_max']:.3f} | vel RMSE {e['vel_rmse']:.3f} m/s")
    print("=" * 78 + "\n")

    # Export
    plot_dir = os.path.join(CURRENT_DIR, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    out_dirs = [plot_dir]
    if args.export:
        thesis_dir = os.path.abspath(os.path.join(
            CURRENT_DIR, "../../latex/tese/Implementation/figures/02_model_validation"))
        os.makedirs(thesis_dir, exist_ok=True)
        out_dirs.append(thesis_dir)

    plot_inputs(data, att_fit, out_dirs)
    plot_attitude(data, att_fit, out_dirs)
    plot_acceleration(data, a_meas, a_model, out_dirs)
    plot_prediction_error(np.arange(n_h + 1) * dt, curves, out_dirs)
    plot_prediction_examples(data, windows, out_dirs)
    print("[INFO] Plots successfully generated and exported.")


if __name__ == "__main__":
    main()
