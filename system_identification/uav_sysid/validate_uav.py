#!/usr/bin/env python3
"""
UAV Dynamics Validation Script (Tethered UAV-USV Project)
Validates the Section 3.2 (UAV NMPC) 9-state first-principles model:
    x_a = [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi]^T in R^9
    u_a = [phi_dot, theta_dot, psi_dot, T]^T in R^4
against Gazebo/PX4 ground truth from rosbag telemetry.

STRICT FIRST-PRINCIPLES FORMULATION:
- Tether force included as specified in Thesis Section 3.1.3 & Section 3.2.
- First-principles thrust equilibrium: PX4 hover throttle u_hover maintains equilibrium
  against both gravity and vertical tether load (T_hover = mg + |F_tether,z|), ensuring
  physical consistency and eliminating double counting.
- Zero parameter fitting, zero identified time constants (tau), zero artificial delays (t_d).
- All physical parameters (m, g, mu, eps_0, gamma_0, anchor offset) are nominal.
- Integration via 4th-order Runge-Kutta (RK4) over the NMPC horizon (N=50, Ts=0.02s -> 1.0s).
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d
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

# ==============================================================================
# NOMINAL PHYSICAL AND CONTROL PARAMETERS (THESIS SECTION 3.2)
# ==============================================================================
MASS = 2.06             # kg, nominal mass (x500 SDF: 2.0 kg airframe + 4x 0.016 kg rotors)
G = 9.81                # m/s^2, standard gravitational acceleration

# Nominal Tether Model (Sec 3.1.3 & Sec 3.2: mu = 0.020 kg/m, eps0 = 0.05, gamma0 = 28 deg)
MU_TETHER = 0.020       # kg/m, linear mass density
EPS0 = 0.05             # nominal operating slack ratio
GAMMA0 = np.deg2rad(28.0)  # rad, nominal catenary deflection angle
C_D = MU_TETHER * G / (1.0 - EPS0)  # N/m, nominal tension-distance coefficient (0.2065 N/m)

# Nominal Tether Anchor on USV (from frame_manager_params.yaml: z_offset = 1.3 - 0.265 = 1.035 m)
NOMINAL_ANCHOR_OFFSET = np.array([0.0, 0.0, 1.035])

# Prediction Horizon (Thesis Section 3.2 / acados_generator_config.json)
TS = 0.02               # s, sampling time (50 Hz)
N_HORIZON = 50          # prediction steps (1.0 s horizon)
PHI_DOT_MAX = 0.8       # rad/s, attitude rate limit in NMPC (~45.8 deg/s)
FLIGHT_START_Z = 5.5    # m, analysis start altitude (filters ground contact/takeoff transients)

TOPICS = [
    '/drone/ground_truth/odometry',
    '/boat/ground_truth/odometry',
    '/px4_1/fmu/in/vehicle_attitude_setpoint',
    '/px4_1/fmu/out/vehicle_attitude_setpoint',
    '/px4_1/fmu/out/hover_thrust_estimate',
    '/moordyn_tether_node/tether_distance',
    '/moordyn_tether_node/tether_length',
    '/tether_force_drone_mag',
    '/world/wamv_world/wrench',
]

# Coordinate frame rotations: PX4 (NED/FRD) <-> ROS (ENU/FLU)
R_ENU_TO_NED = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_FLU_TO_FRD = R.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate Thesis Section 3.2 UAV Model with first-principles tether coupling."
    )
    default_bag = os.path.abspath(
        os.path.join(CURRENT_DIR, "../../../bags/uav_px4_offboard_20261005_213544")
    )
    parser.add_argument("--bag", type=str, default=default_bag, help="Path to ROS 2 bag folder")
    parser.add_argument("--skip", type=float, default=14.0,
                        help="Seconds skipped at start after reaching hover")
    parser.add_argument("--no-export", dest="export", action="store_false",
                        help="Do not export plots to thesis LaTeX directory")
    parser.set_defaults(export=True)
    return parser.parse_args()


# ==============================================================================
# NOMINAL TETHER FORCE CALCULATION (THESIS SECTION 3.1.3 & 3.2)
# ==============================================================================
def nominal_tether_force(p_uav, p_anchor):
    """
    First-principles closed-form tether force at UAV attachment point.
    Equations (3.49) - (3.60) / Thesis Section 3.1.3 & 3.2.
    """
    r = p_anchor - p_uav
    d = np.linalg.norm(r)
    if d < 1e-4:
        return np.zeros(3)
    d0 = r / d
    horiz = np.array([d0[0], d0[1], 0.0])
    nh = np.linalg.norm(horiz)
    eh = horiz / nh if nh > 1e-6 else np.array([1.0, 0.0, 0.0])
    alpha = np.arcsin(np.clip(-d0[2], -1.0, 1.0))
    ang = alpha + GAMMA0
    dhat = np.cos(ang) * eh
    dhat[2] = -np.sin(ang)
    return C_D * d * dhat


# ==============================================================================
# TELEMETRY LOADING & DATASET RESAMPLING
# ==============================================================================
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
                if 'x500' in msg.entity.name:
                    f = msg.wrench.force
                    raw[tp].append([t_w, f.x, f.y, f.z])
            else:
                raw[tp].append([t_w, msg.data])
    return {k: np.array(v) for k, v in raw.items()}


def build_dataset(raw, skip=0.0):
    d_odom = raw['/drone/ground_truth/odometry']
    b_odom = raw['/boat/ground_truth/odometry']
    sp = raw['/px4_1/fmu/out/vehicle_attitude_setpoint']
    if len(sp) == 0:
        sp = raw['/px4_1/fmu/in/vehicle_attitude_setpoint']
    hov = raw['/px4_1/fmu/out/hover_thrust_estimate']

    wall2sim = interp1d(d_odom[:, 0], d_odom[:, 1], fill_value='extrapolate')
    t_sp = wall2sim(sp[:, 0])
    t_hov = wall2sim(hov[:, 0])
    t_tether = wall2sim(raw['/moordyn_tether_node/tether_length'][:, 0])

    t_fly = d_odom[np.argmax(d_odom[:, 4] >= FLIGHT_START_Z), 1]
    t_start = max(d_odom[0, 1], b_odom[0, 1], t_sp[0], t_hov[0], t_tether[0], t_fly) + skip
    t_end = min(d_odom[-1, 1], b_odom[-1, 1], t_sp[-1], t_hov[-1], t_tether[-1])
    t_grid = np.arange(t_start, t_end, TS)
    t = t_grid - t_start

    def on_grid(ts, vals):
        return interp1d(ts, vals, axis=0, fill_value='extrapolate')(t_grid)

    # UAV Ground Truth State: position, orientation, linear velocity
    p = on_grid(d_odom[:, 1], d_odom[:, 2:5])
    q = on_grid(d_odom[:, 1], d_odom[:, 5:9])
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rot = R.from_quat(q)
    v_body = on_grid(d_odom[:, 1], d_odom[:, 9:12])
    v_world = rot.apply(v_body)
    yaw, pitch, roll = rot.as_euler('ZYX').T
    yaw = np.unwrap(yaw)

    # Commanded Attitude (PX4 NED/FRD setpoint -> ROS ENU/FLU) and Thrust
    q_d = sp[:, 1:5]
    r_cmd = R_ENU_TO_NED * R.from_quat(q_d[:, [1, 2, 3, 0]]) * R_FLU_TO_FRD
    yaw_c, pitch_c, roll_c = r_cmd.as_euler('ZYX').T
    yaw_c = np.unwrap(yaw_c)
    thr_cmd = -sp[:, 5]

    cmd = on_grid(t_sp, np.column_stack([roll_c, pitch_c, yaw_c, thr_cmd]))
    hover = on_grid(t_hov, hov[:, 1])

    # Commanded attitude rates: u_a = [phi_dot, theta_dot, psi_dot, T]
    phi_dot_cmd = np.gradient(cmd[:, 0], TS)
    theta_dot_cmd = np.gradient(cmd[:, 1], TS)
    psi_dot_cmd = np.gradient(cmd[:, 2], TS)

    # USV Ground Truth State & Nominal Anchor Position
    pb = on_grid(b_odom[:, 1], b_odom[:, 2:5])
    qb = on_grid(b_odom[:, 1], b_odom[:, 5:9])
    qb /= np.linalg.norm(qb, axis=1, keepdims=True)
    rot_b = R.from_quat(qb)
    p_anchor_nom = pb + rot_b.apply(NOMINAL_ANCHOR_OFFSET)

    # First-Principles Thrust Mapping:
    # Under hover equilibrium with the tether, T_hover = mg + |F_tether,z|.
    # PX4's hover throttle u_hover estimates the command required for this equilibrium.
    # Therefore, actual motor thrust produced at throttle u_T is:
    #     T = (mg + |F_tether,z|) * (u_T / u_hover)
    f_tether_nom = np.array([nominal_tether_force(p[k], p_anchor_nom[k]) for k in range(len(t_grid))])
    f_tether_pull = -f_tether_nom[:, 2]  # Downward vertical force component (N)
    thrust = (MASS * G + f_tether_pull) * cmd[:, 3] / hover

    # MoorDyn & Simulator Telemetry
    tdist = on_grid(t_tether, raw['/moordyn_tether_node/tether_distance'][:, 1])
    tlen = on_grid(t_tether, raw['/moordyn_tether_node/tether_length'][:, 1])
    tforce_mag = on_grid(wall2sim(raw['/tether_force_drone_mag'][:, 0]),
                         raw['/tether_force_drone_mag'][:, 1])
    wr = raw['/world/wamv_world/wrench']
    f_wrench = on_grid(wall2sim(wr[:, 0]), wr[:, 1:4])

    return {
        't': t, 'dt': TS,
        'p': p, 'v': v_world, 'rot': rot,
        'roll': roll, 'pitch': pitch, 'yaw': yaw,
        'roll_c': cmd[:, 0], 'pitch_c': cmd[:, 1], 'yaw_c': cmd[:, 2],
        'phi_dot_c': phi_dot_cmd, 'theta_dot_c': theta_dot_cmd, 'psi_dot_c': psi_dot_cmd,
        'thr_norm': cmd[:, 3], 'hover': hover, 'T': thrust,
        'pb': pb, 'rot_b': rot_b, 'p_anchor': p_anchor_nom,
        'f_tether_pull': f_tether_pull,
        'tether_dist': tdist, 'tether_len': tlen,
        'tether_force_mag': tforce_mag, 'F_wrench': f_wrench,
    }


# ==============================================================================
# SECTION 3.2 CONTINUOUS-TIME DYNAMICS & INTEGRATION (RK4)
# ==============================================================================
def uav_dynamics_sec32(x, u, p_anchor):
    """
    Continuous state-space model of the UAV from Thesis Section 3.2.
    State x in R^9:  [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi]^T
    Input u in R^4:  [phi_dot, theta_dot, psi_dot, T]^T
    Equations (3.84) - (3.94).
    """
    px, py, pz, vx, vy, vz, phi, theta, psi = x
    phi_dot, theta_dot, psi_dot, T = u

    # Kinematics: p_dot = v
    p_dot = np.array([vx, vy, vz])

    # Thrust vectoring in inertial frame (ZYX rotation: Eqs. 3.91 - 3.93)
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)

    ax_thrust = (T / MASS) * (cps * sth * cph + sps * sph)
    ay_thrust = (T / MASS) * (sps * sth * cph - cps * sph)
    az_thrust = (T / MASS) * (cth * cph)
    a_thrust = np.array([ax_thrust, ay_thrust, az_thrust])

    # Gravity
    a_grav = np.array([0.0, 0.0, -G])

    # Closed-form tether force (Section 3.1.3 & 3.2)
    f_tether = nominal_tether_force(x[:3], p_anchor)
    a_tether = f_tether / MASS

    v_dot = a_thrust + a_grav + a_tether

    # Attitude rates: eta_dot = u_rates (Eq. 3.94)
    att_dot = np.array([phi_dot, theta_dot, psi_dot])

    return np.concatenate([p_dot, v_dot, att_dot])


def rk4_step(x, u1, u2, pa1, pa2, dt):
    """Fourth-order Runge-Kutta step with linear interpolation of inputs/anchor."""
    u_mid = 0.5 * (u1 + u2)
    pa_mid = 0.5 * (pa1 + pa2)
    k1 = uav_dynamics_sec32(x, u1, pa1)
    k2 = uav_dynamics_sec32(x + 0.5 * dt * k1, u_mid, pa_mid)
    k3 = uav_dynamics_sec32(x + 0.5 * dt * k2, u_mid, pa_mid)
    k4 = uav_dynamics_sec32(x + dt * k3, u2, pa2)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_horizon(k0, n_h, data, use_meas_attitude=False):
    """
    Open-loop integration of the Section 3.2 model over n_h steps.
    Initial condition x(0) is taken strictly from ground-truth odometry at k0.
    """
    dt = data['dt']
    x_sim = np.zeros((n_h + 1, 9))
    x_sim[0] = np.array([
        data['p'][k0, 0], data['p'][k0, 1], data['p'][k0, 2],
        data['v'][k0, 0], data['v'][k0, 1], data['v'][k0, 2],
        data['roll'][k0], data['pitch'][k0], data['yaw'][k0]
    ])

    for i in range(n_h):
        k = k0 + i
        if use_meas_attitude:
            # Baseline: feed measured attitude directly to isolate translational block
            dphi1 = np.gradient(data['roll'][k:k+2], dt)[0] if k+1 < len(data['t']) else 0.0
            dphi2 = dphi1
            dtheta1 = np.gradient(data['pitch'][k:k+2], dt)[0] if k+1 < len(data['t']) else 0.0
            dtheta2 = dtheta1
            dpsi1 = np.gradient(data['yaw'][k:k+2], dt)[0] if k+1 < len(data['t']) else 0.0
            dpsi2 = dpsi1
            u1 = np.array([dphi1, dtheta1, dpsi1, data['T'][k]])
            u2 = np.array([dphi2, dtheta2, dpsi2, data['T'][k+1]])
        else:
            u1 = np.array([data['phi_dot_c'][k], data['theta_dot_c'][k], data['psi_dot_c'][k], data['T'][k]])
            u2 = np.array([data['phi_dot_c'][k+1], data['theta_dot_c'][k+1], data['psi_dot_c'][k+1], data['T'][k+1]])

        pa1 = data['p_anchor'][k]
        pa2 = data['p_anchor'][k+1]
        x_sim[i + 1] = rk4_step(x_sim[i], u1, u2, pa1, pa2, dt)

        if use_meas_attitude:
            x_sim[i + 1, 6:9] = np.array([data['roll'][k+1], data['pitch'][k+1], data['yaw'][k+1]])

    return x_sim


# ==============================================================================
# STATISTICAL METRICS
# ==============================================================================
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


# ==============================================================================
# PLOTTING FUNCTIONS (IST THESIS STYLE)
# ==============================================================================
def plot_actuator_inputs(data, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)

    axs[0].plot(t, np.rad2deg(data['roll_c']), color=COLOR_CYCLE[0], label=r'Roll $\phi_c$', linewidth=1.3)
    axs[0].plot(t, np.rad2deg(data['pitch_c']), color=COLOR_CYCLE[1], label=r'Pitch $\theta_c$', linewidth=1.3)
    axs[0].set_ylabel('Attitude cmd [deg]')
    axs[0].legend(loc='upper right', framealpha=0.9, ncol=2)

    axs[1].plot(t, np.rad2deg(data['yaw_c']), color=COLOR_CYCLE[2], label=r'Yaw $\psi_c$', linewidth=1.3)
    axs[1].set_ylabel('Yaw cmd [deg]')
    axs[1].legend(loc='upper right', framealpha=0.9)

    axs[2].plot(t, data['T'], color=COLOR_CYCLE[0], linewidth=1.3, label='Commanded thrust $T$')
    axs[2].plot(t, MASS * G + data['f_tether_pull'], color='gray', linestyle='--', linewidth=1.1,
                alpha=0.8, label=r'Hover balance $mg + F_{\mathrm{tether},z}$')
    axs[2].axhline(MASS * G, color='r', linestyle=':', alpha=0.7, label=f'Nominal weight $mg={MASS*G:.2f}$ N')
    axs[2].set_ylabel('Thrust [N]')
    axs[2].set_xlabel('Time [s]')
    axs[2].legend(loc='upper right', framealpha=0.9, ncol=3)

    for ax in axs:
        ax.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_actuator_inputs'), save_png=True)
    plt.close(fig)


def plot_attitude_tracking(data, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=True)
    keys = [('roll', 'roll_c', 'Roll', r'\phi'),
            ('pitch', 'pitch_c', 'Pitch', r'\theta'),
            ('yaw', 'yaw_c', 'Yaw', r'\psi')]

    for ax, (meas_k, cmd_k, name, sym) in zip(axs, keys):
        ax.plot(t, np.rad2deg(data[cmd_k]), color=COLOR_CYCLE[1], linewidth=1.2,
                linestyle='-.', label=f'Command ${sym}_c$')
        ax.plot(t, np.rad2deg(data[meas_k]), color=GROUND_TRUTH_COLOR, linewidth=1.4,
                label='Ground Truth')
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
        ax.plot(t, a_model[:, i], color=COLOR_CYCLE[0], linewidth=1.3, linestyle='--',
                label='Section 3.2 Model')
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
        axs[0].plot(horizon_t, c['pos_rmse'], color=col, linestyle=styles[i % 4],
                    linewidth=1.5, label=label)
        axs[1].plot(horizon_t, c['vel_rmse'], color=col, linestyle=styles[i % 4],
                    linewidth=1.5, label=label)
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
    axs[0].plot(data['p'][:, 0], data['p'][:, 1], color=GROUND_TRUTH_COLOR,
                linewidth=1.3, label='Ground Truth')
    for j, (k0, s) in enumerate(windows):
        axs[0].plot(s[:, 0], s[:, 1], color=COLOR_CYCLE[0], linewidth=2.0,
                    label='1 s predictions' if j == 0 else None)
        axs[0].plot(s[0, 0], s[0, 1], 'o', color=COLOR_CYCLE[0], markersize=3)
    axs[0].set_xlabel('$x$ [m]')
    axs[0].set_ylabel('$y$ [m]')
    axs[0].axis('equal')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='best', framealpha=0.9)

    # Zoom on the window with the largest horizontal velocity
    k0, s = max(windows, key=lambda w: np.linalg.norm(data['v'][w[0], :2]))
    n = len(s) - 1
    tt = np.arange(n + 1) * data['dt']
    for i, (name, col) in enumerate(zip(('x', 'y'), COLOR_CYCLE[:2])):
        axs[1].plot(tt, data['p'][k0:k0 + n + 1, i] - data['p'][k0, i], color=GROUND_TRUTH_COLOR,
                    linewidth=1.4, label='Ground Truth' if i == 0 else None)
        axs[1].plot(tt, s[:, i] - s[0, i], color=col, linewidth=1.4, linestyle='--',
                    label=f'Model ${name}$')
    axs[1].set_xlabel('Time since window start [s]')
    axs[1].set_ylabel('Displacement [m]')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='best', framealpha=0.9)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'uav_prediction_examples'), save_png=True)
    plt.close(fig)


# ==============================================================================
# MAIN VALIDATION PIPELINE
# ==============================================================================
def main():
    args = parse_args()
    raw = read_bag(args.bag)
    data = build_dataset(raw, args.skip)
    t, dt = data['t'], data['dt']
    N = len(t)

    print("\n" + "=" * 78)
    print("      THESIS SECTION 3.2 UAV NMPC MODEL FIRST-PRINCIPLES VALIDATION")
    print("=" * 78)
    print(f"Nominal Parameters: m = {MASS} kg | g = {G} m/s^2 | mu = {MU_TETHER} kg/m | eps0 = {EPS0}")
    print(f"C_d nominal = {C_D:.4f} N/m | gamma0 = {np.rad2deg(GAMMA0):.1f} deg | Anchor body offset: {NOMINAL_ANCHOR_OFFSET}")
    print(f"Prediction Horizon: N = {N_HORIZON} steps, Ts = {dt} s -> T_horizon = {N_HORIZON*dt:.2f} s")
    print(f"Telemetry window: {t[-1]:.2f} s ({N} samples at 50 Hz)")

    # --- 1. ATTITUDE TRACKING (IDEAL VS MEASURED) ---
    print("\n[1] ATTITUDE TRACKING (Commanded setpoints vs Ground Truth)")
    att_metrics = {}
    rate_keys = {'roll': 'phi_dot_c', 'pitch': 'theta_dot_c', 'yaw': 'psi_dot_c'}
    for key, name in [('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw')]:
        meas = np.rad2deg(data[key])
        cmd = np.rad2deg(data[key + '_c'])
        m = compute_metrics(meas, cmd)
        att_metrics[key] = m
        rk = rate_keys[key]
        rate_cmd_max = np.rad2deg(np.abs(data[rk]).max())
        rate_cmd_p99 = np.rad2deg(np.percentile(np.abs(data[rk]), 99))
        print(f"  {name:<6}: RMSE = {m['rmse']:.3f} deg, MAE = {m['mae']:.3f} deg, Max = {m['max']:.2f} deg, R2 = {m['r2']:.1f}% | "
              f"Max |rate_cmd| = {rate_cmd_max:.1f} deg/s (P99 = {rate_cmd_p99:.1f} deg/s)")

    # --- 2. THRUST MAPPING & HOVER BALANCE ---
    print("\n[2] THRUST MAPPING & EQUILIBRIUM (T_hover = mg + |F_tether,z|)")
    win = 11  # 0.22 s window for acceleration derivation
    a_meas = savgol_filter(data['v'], win, 2, deriv=1, delta=dt, axis=0)
    g_vec = np.array([0.0, 0.0, -G])
    z_body = data['rot'].apply(np.array([0.0, 0.0, 1.0]))
    a_thrust_implied = np.einsum('ij,ij->i', a_meas - g_vec, z_body)
    a_thrust_cmd = data['T'] / MASS
    m_thrust = compute_metrics(a_thrust_implied, a_thrust_cmd)
    gain_thrust = np.sum(a_thrust_implied * a_thrust_cmd) / np.sum(a_thrust_cmd ** 2)
    mean_tet_pull = np.mean(data['f_tether_pull'])
    print(f"  Weight mg: {MASS*G:.2f} N | Mean tether vertical pull: {mean_tet_pull:.3f} N | Mean motor thrust: {np.mean(data['T']):.2f} N")
    print(f"  Mean Commanded T/m: {np.mean(a_thrust_cmd):.3f} m/s^2 | Implied T/m: {np.mean(a_thrust_implied):.3f} m/s^2")
    print(f"  Thrust Accel Gain = {gain_thrust:.4f} | RMSE = {m_thrust['rmse']:.3f} m/s^2 | R2 = {m_thrust['r2']:.1f}%")

    # --- 3. ACCELERATION LEVEL VALIDATION ---
    print("\n[3] ONE-STEP-AHEAD ACCELERATION COMPARISON")
    a_sec32 = np.zeros((N, 3))
    for k in range(N):
        x = np.array([*data['p'][k], *data['v'][k], data['roll'][k], data['pitch'][k], data['yaw'][k]])
        u = np.array([data['phi_dot_c'][k], data['theta_dot_c'][k], data['psi_dot_c'][k], data['T'][k]])
        pa = data['p_anchor'][k]
        a_sec32[k] = uav_dynamics_sec32(x, u, pa)[3:6]

    m_acc = [compute_metrics(a_meas[:, i], a_sec32[:, i]) for i in range(3)]
    print("  Sec 3.2 Model (com tether): " +
          " | ".join(f"{ax}: RMSE {m['rmse']:.3f} m/s^2 (R2 {m['r2']:.1f}%)" for ax, m in zip('xyz', m_acc)))

    # --- 4. TETHER FORCE EVALUATION ---
    print("\n[T] NOMINAL TETHER FORCE MODEL VS SIMULATOR LOGGED WRENCH")
    f_nominal_tether = np.array([nominal_tether_force(data['p'][k], data['p_anchor'][k]) for k in range(N)])
    f_wrench_true = data['F_wrench']
    m_tether_3d = np.sqrt(np.mean(np.sum((f_wrench_true - f_nominal_tether) ** 2, axis=1)))
    m_tether_axes = [compute_metrics(f_wrench_true[:, i], f_nominal_tether[:, i]) for i in range(3)]
    print(f"  Anchor Distance: [{data['tether_dist'].min():.2f}, {data['tether_dist'].max():.2f}] m (mean {data['tether_dist'].mean():.2f} m)")
    print(f"  True F mean: {np.round(f_wrench_true.mean(0), 3)} N | Nominal Model F mean: {np.round(f_nominal_tether.mean(0), 3)} N")
    print(f"  3D Force RMSE = {m_tether_3d:.3f} N | " +
          " | ".join(f"F{ax}: RMSE {m['rmse']:.3f} N (R2 {m['r2']:.1f}%)" for ax, m in zip('xyz', m_tether_axes)))

    # --- 5. MULTI-STEP OPEN-LOOP PREDICTION OVER HORIZON (1.0 s) ---
    print("\n[4] MULTI-STEP OPEN-LOOP PREDICTION (N = 50 steps, Horizon = 1.0 s)")
    variants = {
        'Sec 3.2 (Comando)': {'use_meas_att': False},
        'Sec 3.2 (Atitude medida)': {'use_meas_att': True},
    }

    n_h = N_HORIZON
    starts = np.arange(0, N - n_h - 1, 5)  # window every 0.1 s
    curves, final_err, sample_windows = {}, {}, []

    for name, cfg in variants.items():
        err_p = np.zeros((len(starts), n_h + 1, 3))
        err_v = np.zeros((len(starts), n_h + 1, 3))
        for j, k0 in enumerate(starts):
            s = simulate_horizon(k0, n_h, data, cfg['use_meas_att'])
            err_p[j] = s[:, :3] - data['p'][k0:k0 + n_h + 1]
            err_v[j] = s[:, 3:6] - data['v'][k0:k0 + n_h + 1]
            if name == 'Sec 3.2 (Comando)' and k0 % 250 == 0:
                sample_windows.append((k0, s))

        pos_norm = np.linalg.norm(err_p, axis=2)
        vel_norm = np.linalg.norm(err_v, axis=2)
        curves[name] = {
            'pos_rmse': np.sqrt(np.mean(pos_norm ** 2, axis=0)),
            'vel_rmse': np.sqrt(np.mean(vel_norm ** 2, axis=0)),
        }
        final_err[name] = {
            'pos_rmse': curves[name]['pos_rmse'][-1],
            'xy_rmse': np.sqrt(np.mean(np.sum(err_p[:, -1, :2] ** 2, axis=1))),
            'z_rmse': np.sqrt(np.mean(err_p[:, -1, 2] ** 2)),
            'pos_p95': np.percentile(pos_norm[:, -1], 95),
            'pos_max': pos_norm[:, -1].max(),
            'vel_rmse': curves[name]['vel_rmse'][-1],
        }

    for name, e in final_err.items():
        print(f"  {name:<28}: Pos RMSE {e['pos_rmse']:.3f} m (xy {e['xy_rmse']:.3f}, z {e['z_rmse']:.3f}), "
              f"P95 {e['pos_p95']:.3f} m, Max {e['pos_max']:.3f} m | Vel RMSE {e['vel_rmse']:.3f} m/s")

    print("=" * 78 + "\n")

    # --- 6. EXPORT PLOTS ---
    plot_dir = os.path.join(CURRENT_DIR, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    out_dirs = [plot_dir]
    if args.export:
        thesis_dir = os.path.abspath(os.path.join(
            CURRENT_DIR, "../../latex/tese/Implementation/figures/02_model_validation"
        ))
        os.makedirs(thesis_dir, exist_ok=True)
        out_dirs.append(thesis_dir)

    horizon_t = np.arange(n_h + 1) * dt
    plot_actuator_inputs(data, out_dirs)
    plot_attitude_tracking(data, out_dirs)
    plot_acceleration(data, a_meas, a_sec32, out_dirs)
    plot_prediction_error(horizon_t, curves, out_dirs)
    plot_prediction_examples(data, sample_windows, out_dirs)
    print(f"[INFO] Plots successfully generated and exported to {out_dirs}")


if __name__ == "__main__":
    main()
