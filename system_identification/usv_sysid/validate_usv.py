#!/usr/bin/env python3
"""
USV Dynamics Validation Script (Tethered UAV-USV Project)
Validates the first-principles 3-DOF planar Fossen model using nominal simulator
parameters against Gazebo ground truth telemetry from rosbag recordings.
Strictly relies on simulator specifications without empirical data fitting.
Implements receding horizon multi-step prediction (NMPC horizon = 2.0 s).
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

# Add parent directory for thesis_style import
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SYSID_DIR = os.path.dirname(CURRENT_DIR)
if SYSID_DIR not in sys.path:
    sys.path.append(SYSID_DIR)

from thesis_style import (
    set_thesis_style,
    save_figure,
    GROUND_TRUTH_COLOR,
    SIMULATION_COLOR,
    COLOR_CYCLE,
)

# Nominal physical parameters directly from Thesis (Table 4.1 / model.sdf)
# Total Mass: Base (227 kg) + 2 Engines (30 kg) + Sensors/Props (2.1 kg) = 259.10 kg
MASS_NOMINAL = 259.10
# Total Yaw Inertia: Base Izz (495 kg*m^2) + Parallel Axis Theorem for 2 engines:
# 495.0 + 2 * 15.0 * ((-2.373776)^2 + 1.027135^2) = 695.9 kg*m^2
IZZ_NOMINAL = 695.90

# Hydrodynamic damping coefficients from Table 4.1 (VRX SimpleHydrodynamics)
X_U_NOMINAL = 100.00
X_UU_NOMINAL = 150.00
Y_V_NOMINAL = 100.00
Y_VV_NOMINAL = 100.00
N_R_NOMINAL = 800.00
N_RR_NOMINAL = 800.00

# Thruster geometry relative to CG (meters)
THRUSTER_X_ARM = 2.373776
THRUSTER_Y_ARM = 1.027135

# NMPC Horizon: N = 20 steps at Ts = 0.1 s -> T_horizon = 2.0 s (at 50 Hz dt=0.02s -> N_HORIZON = 100)
N_HORIZON = 100


def parse_args():
    parser = argparse.ArgumentParser(description="Validate USV planar model against rosbag telemetry.")
    default_bag = os.path.abspath(os.path.join(CURRENT_DIR, "../../../bags/mission_sysid_20261005_194359"))
    parser.add_argument("--bag", type=str, default=default_bag, help="Path to ROS 2 bag folder")
    parser.add_argument("--export", action="store_true", default=True, help="Export figures to thesis directory")
    return parser.parse_args()


def load_telemetry(bag_path):
    print(f"[INFO] Reading telemetry from: {bag_path}")
    typestore = get_typestore(Stores.LATEST)
    
    boat_topics = [
        '/boat/ground_truth/odometry',
        '/boat/thrusters/left/thrust',
        '/boat/thrusters/right/thrust',
        '/boat/thrusters/left/pos',
        '/boat/thrusters/right/pos'
    ]
    
    odom_wall, odom_sim = [], []
    x_gt, y_gt, psi_gt, u_gt, v_gt, r_gt = [], [], [], [], [], []
    tl_wall, tl_val = [], []
    tr_wall, tr_val = [], []
    al_wall, al_val = [], []
    ar_wall, ar_val = [], []
    
    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic in boat_topics]
        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t_w = timestamp / 1e9
            
            if conn.topic == '/boat/ground_truth/odometry':
                t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                odom_wall.append(t_w)
                odom_sim.append(t_s)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                psi = np.arctan2(siny_cosp, cosy_cosp)
                x_gt.append(p.x)
                y_gt.append(p.y)
                psi_gt.append(psi)
                u_gt.append(msg.twist.twist.linear.x)
                v_gt.append(msg.twist.twist.linear.y)
                r_gt.append(msg.twist.twist.angular.z)
            elif conn.topic == '/boat/thrusters/left/thrust':
                tl_wall.append(t_w)
                tl_val.append(msg.data)
            elif conn.topic == '/boat/thrusters/right/thrust':
                tr_wall.append(t_w)
                tr_val.append(msg.data)
            elif conn.topic == '/boat/thrusters/left/pos':
                al_wall.append(t_w)
                al_val.append(msg.data)
            elif conn.topic == '/boat/thrusters/right/pos':
                ar_wall.append(t_w)
                ar_val.append(msg.data)
                
    odom_wall = np.array(odom_wall)
    odom_sim = np.array(odom_sim)
    
    # Map wall clock timestamps of actuators to simulation time grid
    sim_interp = interp1d(odom_wall, odom_sim, fill_value='extrapolate')
    t_tl_sim = sim_interp(tl_wall)
    t_tr_sim = sim_interp(tr_wall)
    t_al_sim = sim_interp(al_wall)
    t_ar_sim = sim_interp(ar_wall)
    
    t_start = max(odom_sim[0], t_tl_sim[0], t_tr_sim[0], t_al_sim[0], t_ar_sim[0])
    t_end = min(odom_sim[-1], t_tl_sim[-1], t_tr_sim[-1], t_al_sim[-1], t_ar_sim[-1])
    dt = 0.02  # 50 Hz simulation step
    t_grid = np.arange(t_start, t_end, dt)
    t_rel = t_grid - t_start
    
    # Synchronous interpolation
    tl_grid = interp1d(t_tl_sim, tl_val, fill_value='extrapolate')(t_grid)
    tr_grid = interp1d(t_tr_sim, tr_val, fill_value='extrapolate')(t_grid)
    al_grid = interp1d(t_al_sim, al_val, fill_value='extrapolate')(t_grid)
    ar_grid = interp1d(t_ar_sim, ar_val, fill_value='extrapolate')(t_grid)
    
    x_gt_grid = interp1d(odom_sim, x_gt, fill_value='extrapolate')(t_grid)
    y_gt_grid = interp1d(odom_sim, y_gt, fill_value='extrapolate')(t_grid)
    u_gt_grid = interp1d(odom_sim, u_gt, fill_value='extrapolate')(t_grid)
    v_gt_grid = interp1d(odom_sim, v_gt, fill_value='extrapolate')(t_grid)
    r_gt_grid = interp1d(odom_sim, r_gt, fill_value='extrapolate')(t_grid)
    psi_gt_grid = np.unwrap(interp1d(odom_sim, np.unwrap(psi_gt), fill_value='extrapolate')(t_grid))
    
    return {
        't': t_rel,
        'dt': dt,
        'T_L': tl_grid,
        'T_R': tr_grid,
        'alpha_L': al_grid,
        'alpha_R': ar_grid,
        'x_gt': x_gt_grid,
        'y_gt': y_gt_grid,
        'psi_gt': psi_gt_grid,
        'u_gt': u_gt_grid,
        'v_gt': v_gt_grid,
        'r_gt': r_gt_grid
    }


def compute_thruster_forces(data):
    T_L = data['T_L']
    T_R = data['T_R']
    a_L = data['alpha_L']
    a_R = data['alpha_R']
    
    X = T_L * np.cos(a_L) + T_R * np.cos(a_R)
    Y = T_L * np.sin(a_L) + T_R * np.sin(a_R)
    N_moment = -THRUSTER_X_ARM * (T_L * np.sin(a_L) + T_R * np.sin(a_R)) - THRUSTER_Y_ARM * (T_L * np.cos(a_L) - T_R * np.cos(a_R))
    return np.column_stack([X, Y, N_moment])


def usv_dynamics(s, tau):
    """
    Continuous 3-DOF Fossen planar model (Thesis Section 3.1 & Table 4.1).
    State s in R^6: [x, y, psi, u, v, r]^T
    Input tau in R^3: [X, Y, N]^T
    """
    x, y, psi, u, v, r = s
    X_in, Y_in, N_in = tau
    x_dot = u * np.cos(psi) - v * np.sin(psi)
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    psi_dot = r
    u_dot = (X_in - (X_U_NOMINAL + X_UU_NOMINAL * abs(u)) * u) / MASS_NOMINAL + v * r
    v_dot = (Y_in - (Y_V_NOMINAL + Y_VV_NOMINAL * abs(v)) * v) / MASS_NOMINAL - u * r
    r_dot = (N_in - (N_R_NOMINAL + N_RR_NOMINAL * abs(r)) * r) / IZZ_NOMINAL
    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])


def rk4_step(s, tau1, tau2, dt):
    """Fourth-order Runge-Kutta step with linear interpolation of generalized forces."""
    tau_mid = 0.5 * (tau1 + tau2)
    k1 = usv_dynamics(s, tau1)
    k2 = usv_dynamics(s + 0.5 * dt * k1, tau_mid)
    k3 = usv_dynamics(s + 0.5 * dt * k2, tau_mid)
    k4 = usv_dynamics(s + dt * k3, tau2)
    return s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_full(data, tau):
    """Full-duration open-loop simulation across the complete recorded bag."""
    dt = data['dt']
    N = len(data['t'])
    sim_states = np.zeros((N, 6))
    sim_states[0] = [
        data['x_gt'][0], data['y_gt'][0], data['psi_gt'][0],
        data['u_gt'][0], data['v_gt'][0], data['r_gt'][0]
    ]
    for k in range(N - 1):
        sim_states[k + 1] = rk4_step(sim_states[k], tau[k], tau[k + 1], dt)
        
    return {
        'x_sim': sim_states[:, 0],
        'y_sim': sim_states[:, 1],
        'psi_sim': sim_states[:, 2],
        'u_sim': sim_states[:, 3],
        'v_sim': sim_states[:, 4],
        'r_sim': sim_states[:, 5]
    }


def simulate_horizon(k0, n_h, data, tau):
    """
    Receding horizon open-loop integration over n_h steps.
    Initial state s(0) is taken strictly from ground-truth odometry at k0.
    """
    dt = data['dt']
    s_sim = np.zeros((n_h + 1, 6))
    s_sim[0] = [
        data['x_gt'][k0], data['y_gt'][k0], data['psi_gt'][k0],
        data['u_gt'][k0], data['v_gt'][k0], data['r_gt'][k0]
    ]
    for i in range(n_h):
        k = k0 + i
        s_sim[i + 1] = rk4_step(s_sim[i], tau[k], tau[k + 1], dt)
    return s_sim


def compute_metrics(y_true, y_pred):
    err = y_true - y_pred
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    ss_res = np.sum(err ** 2)
    r2 = 100.0 * (1.0 - ss_res / ss_tot) if ss_tot > 1e-6 else 0.0
    return {
        'rmse': np.sqrt(np.mean(err ** 2)),
        'r2': r2,
        'mae': np.mean(np.abs(err)),
        'max': np.max(np.abs(err))
    }


# ==============================================================================
# PLOTTING FUNCTIONS (IST THESIS STYLE)
# ==============================================================================
def plot_actuator_inputs(data, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 4.4), sharex=True)
    
    # Thruster Forces
    axs[0].plot(t, data['T_L'], color=COLOR_CYCLE[0], label='Port ($T_L$)', linewidth=1.3)
    axs[0].plot(t, data['T_R'], color=COLOR_CYCLE[1], label='Starboard ($T_R$)', linewidth=1.3)
    axs[0].axhline(510.0, color='r', linestyle=':', alpha=0.6, label='Fwd Limit ($510\\,\\mathrm{N}$)')
    axs[0].axhline(-380.0, color='r', linestyle=':', alpha=0.6, label='Rev Limit ($-380\\,\\mathrm{N}$)')
    axs[0].set_ylabel('Thrust [N]')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right', framealpha=0.9, ncol=2)
    
    # Azimuth Angles
    axs[1].plot(t, np.rad2deg(data['alpha_L']), color=COLOR_CYCLE[0], label=r'Port ($\alpha_L$)', linewidth=1.3)
    axs[1].plot(t, np.rad2deg(data['alpha_R']), color=COLOR_CYCLE[1], label=r'Starboard ($\alpha_R$)', linewidth=1.3)
    axs[1].axhline(30.0, color='r', linestyle=':', alpha=0.6, label=r'Limit ($\pm 30^\circ$)')
    axs[1].axhline(-30.0, color='r', linestyle=':', alpha=0.6)
    axs[1].set_ylabel('Azimuth [deg]')
    axs[1].set_xlabel('Time [s]')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='upper right', framealpha=0.9, ncol=2)
    
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'usv_actuator_inputs'), save_png=True)
    plt.close(fig)


def plot_velocities_validation(data, sim_res, out_dirs):
    set_thesis_style()
    t = data['t']
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)
    
    # Surge u
    axs[0].plot(t, data['u_gt'], color=SIMULATION_COLOR, label='Simulation', linewidth=1.5)
    axs[0].plot(t, sim_res['u_sim'], color=COLOR_CYCLE[0], label='Prediction Model', linewidth=1.3, linestyle='-')
    axs[0].set_ylabel('$u$ [m/s]')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right', framealpha=0.9)
    
    # Sway v
    axs[1].plot(t, data['v_gt'], color=SIMULATION_COLOR, label='Simulation', linewidth=1.5)
    axs[1].plot(t, sim_res['v_sim'], color=COLOR_CYCLE[0], label='Prediction Model', linewidth=1.3, linestyle='-')
    axs[1].set_ylabel('$v$ [m/s]')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    
    # Yaw rate r
    axs[2].plot(t, np.rad2deg(data['r_gt']), color=SIMULATION_COLOR, label='Simulation', linewidth=1.5)
    axs[2].plot(t, np.rad2deg(sim_res['r_sim']), color=COLOR_CYCLE[0], label='Prediction Model', linewidth=1.3, linestyle='-')
    axs[2].set_ylabel('$r$ [deg/s]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True, linestyle=':', alpha=0.6)
    
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'usv_velocities_validation'), save_png=True)
    plt.close(fig)


def plot_prediction_error(horizon_t, pos_rmse, vel_rmse, out_dirs):
    set_thesis_style()
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 3.6))
    axs[0].plot(horizon_t, pos_rmse, color=COLOR_CYCLE[0], linewidth=1.5, label='Prediction Model')
    axs[1].plot(horizon_t, vel_rmse, color=COLOR_CYCLE[0], linewidth=1.5, label='Prediction Model')
    axs[0].set_ylabel('Position error RMSE [m]')
    axs[1].set_ylabel('Velocity error RMSE [m/s]')
    for ax in axs:
        ax.set_xlabel('Prediction horizon [s]')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper left', framealpha=0.9)
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'usv_prediction_error'), save_png=True)
    plt.close(fig)


def plot_prediction_examples(data, windows, out_dirs):
    set_thesis_style()
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 3.8), gridspec_kw={'width_ratios': [1.2, 1]})
    
    # 2D Trajectory with prediction chords overlaid
    axs[0].plot(data['x_gt'], data['y_gt'], color=SIMULATION_COLOR, linewidth=1.4, label='Simulation')
    for j, (k0, s) in enumerate(windows):
        axs[0].plot(s[:, 0], s[:, 1], color=COLOR_CYCLE[0], linewidth=2.0,
                    label='2 s predictions' if j == 0 else None)
        axs[0].plot(s[0, 0], s[0, 1], 'o', color=COLOR_CYCLE[0], markersize=3)
    axs[0].plot(data['x_gt'][0], data['y_gt'][0], 's', color='#333333', markersize=5, label='Start')
    axs[0].set_xlabel('$x$ [m]')
    axs[0].set_ylabel('$y$ [m]')
    axs[0].axis('equal')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='best', framealpha=0.9)
    
    # Zoom on the window with highest velocity
    k0, s = max(windows, key=lambda w: np.linalg.norm([data['u_gt'][w[0]], data['v_gt'][w[0]]]))
    n = len(s) - 1
    tt = np.arange(n + 1) * data['dt']
    
    dx_true = data['x_gt'][k0:k0 + n + 1] - data['x_gt'][k0]
    dy_true = data['y_gt'][k0:k0 + n + 1] - data['y_gt'][k0]
    dx_pred = s[:, 0] - s[0, 0]
    dy_pred = s[:, 1] - s[0, 1]
    
    axs[1].plot(tt, dx_true, color=SIMULATION_COLOR, linewidth=1.5, label='Simulation')
    axs[1].plot(tt, dy_true, color=SIMULATION_COLOR, linewidth=1.5)
    axs[1].plot(tt, dx_pred, color=COLOR_CYCLE[0], linewidth=1.3, linestyle='-', label=r'Prediction $x$')
    axs[1].plot(tt, dy_pred, color=COLOR_CYCLE[1], linewidth=1.3, linestyle='-', label=r'Prediction $y$')
    
    axs[1].set_xlabel('Time since window start [s]')
    axs[1].set_ylabel('Displacement [m]')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='best', framealpha=0.9)
    
    plt.tight_layout()
    for d in out_dirs:
        save_figure(fig, os.path.join(d, 'usv_prediction_examples'), save_png=True)
        # Also export as usv_trajectory_validation for backwards-compatibility with LaTeX document
        save_figure(fig, os.path.join(d, 'usv_trajectory_validation'), save_png=True)
    plt.close(fig)


def main():
    args = parse_args()
    data = load_telemetry(args.bag)
    dt = data['dt']
    N_pts = len(data['t'])
    
    tau = compute_thruster_forces(data)
    sim_full = simulate_full(data, tau)
    
    # 1. Continuous Full-Duration Velocity Metrics
    m_u = compute_metrics(data['u_gt'], sim_full['u_sim'])
    m_v = compute_metrics(data['v_gt'], sim_full['v_sim'])
    m_r = compute_metrics(data['r_gt'], sim_full['r_sim'])
    m_psi = compute_metrics(np.rad2deg(data['psi_gt']), np.rad2deg(sim_full['psi_sim']))
    m_x = compute_metrics(data['x_gt'], sim_full['x_sim'])
    m_y = compute_metrics(data['y_gt'], sim_full['y_sim'])
    
    dx = np.diff(data['x_gt'])
    dy = np.diff(data['y_gt'])
    total_distance = np.sum(np.sqrt(dx**2 + dy**2))
    
    print("\n" + "=" * 78)
    print("      THESIS SECTION 3.1 USV NMPC MODEL FIRST-PRINCIPLES VALIDATION")
    print("=" * 78)
    print(f"Nominal Parameters: m = {MASS_NOMINAL:.2f} kg | Iz = {IZZ_NOMINAL:.2f} kg*m^2 | Arms: [{THRUSTER_X_ARM:.2f}, {THRUSTER_Y_ARM:.2f}] m")
    print(f"Hydrodynamics: Xu = {X_U_NOMINAL}, Xuu = {X_UU_NOMINAL} | Yv = {Y_V_NOMINAL}, Yvv = {Y_VV_NOMINAL} | Nr = {N_R_NOMINAL}, Nrr = {N_RR_NOMINAL}")
    print(f"Prediction Horizon: N = {N_HORIZON} steps, Ts = {dt} s -> T_horizon = {N_HORIZON*dt:.2f} s")
    print(f"Telemetry window: {data['t'][-1]:.2f} s ({N_pts} samples at 50 Hz, {total_distance:.2f} m traversed)")
    
    print("\n[1] BODY VELOCITIES & DYNAMICS (First-Principles Model vs Ground Truth)")
    print(f"  Surge u : RMSE = {m_u['rmse']:.4f} m/s, MAE = {m_u['mae']:.4f} m/s, Max = {m_u['max']:.4f} m/s, R2 = {m_u['r2']:.1f}%")
    print(f"  Sway v  : RMSE = {m_v['rmse']:.4f} m/s, MAE = {m_v['mae']:.4f} m/s, Max = {m_v['max']:.4f} m/s, R2 = {m_v['r2']:.1f}%")
    print(f"  Yaw r   : RMSE = {np.rad2deg(m_r['rmse']):.4f} deg/s, MAE = {np.rad2deg(m_r['mae']):.4f} deg/s, Max = {np.rad2deg(m_r['max']):.4f} deg/s, R2 = {m_r['r2']:.1f}%")
    print(f"  Heading : RMSE = {m_psi['rmse']:.4f} deg, MAE = {m_psi['mae']:.4f} deg, Max = {m_psi['max']:.4f} deg, R2 = {m_psi['r2']:.1f}%")
    
    # 2. Multi-Step Receding Horizon Open-Loop Prediction (2.0 s NMPC horizon)
    print("\n[2] MULTI-STEP OPEN-LOOP PREDICTION (N = 100 steps, Horizon = 2.0 s)")
    n_h = N_HORIZON
    starts = np.arange(0, N_pts - n_h - 1, 5)  # slide window every 0.1 s
    sample_windows = []
    
    err_pos = np.zeros((len(starts), n_h + 1))
    err_vel = np.zeros((len(starts), n_h + 1))
    
    for j, k0 in enumerate(starts):
        s_sim = simulate_horizon(k0, n_h, data, tau)
        p_true = np.column_stack([data['x_gt'][k0:k0 + n_h + 1], data['y_gt'][k0:k0 + n_h + 1]])
        err_pos[j] = np.linalg.norm(s_sim[:, :2] - p_true, axis=1)
        v_true = np.column_stack([data['u_gt'][k0:k0 + n_h + 1], data['v_gt'][k0:k0 + n_h + 1]])
        err_vel[j] = np.linalg.norm(s_sim[:, 3:5] - v_true, axis=1)
        if k0 % 250 == 0:
            sample_windows.append((k0, s_sim))
            
    pos_rmse_curve = np.sqrt(np.mean(err_pos ** 2, axis=0))
    vel_rmse_curve = np.sqrt(np.mean(err_vel ** 2, axis=0))
    horizon_t = np.arange(n_h + 1) * dt
    
    print(f"  Horizon 0.5 s: Pos RMSE = {pos_rmse_curve[25]:.4f} m, Vel RMSE = {vel_rmse_curve[25]:.4f} m/s")
    print(f"  Horizon 1.0 s: Pos RMSE = {pos_rmse_curve[50]:.4f} m, Vel RMSE = {vel_rmse_curve[50]:.4f} m/s")
    print(f"  Horizon 1.5 s: Pos RMSE = {pos_rmse_curve[75]:.4f} m, Vel RMSE = {vel_rmse_curve[75]:.4f} m/s")
    print(f"  Horizon 2.0 s: Pos RMSE = {pos_rmse_curve[100]:.4f} m, Vel RMSE = {vel_rmse_curve[100]:.4f} m/s")
    print(f"  Final 2.0 s P95 Pos Error: {np.percentile(err_pos[:, -1], 95):.4f} m | Max Pos Error: {err_pos[:, -1].max():.4f} m")
    print("=" * 78 + "\n")
    
    # Destination directories
    plot_dir = os.path.join(CURRENT_DIR, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    out_dirs = [plot_dir]
    if args.export:
        thesis_dir = os.path.abspath(os.path.join(CURRENT_DIR, "../../latex/tese/Implementation/figures/02_model_validation"))
        os.makedirs(thesis_dir, exist_ok=True)
        out_dirs.append(thesis_dir)
        
    plot_actuator_inputs(data, out_dirs)
    plot_velocities_validation(data, sim_full, out_dirs)
    plot_prediction_error(horizon_t, pos_rmse_curve, vel_rmse_curve, out_dirs)
    plot_prediction_examples(data, sample_windows, out_dirs)
    print(f"[INFO] All USV figures successfully generated and exported to {out_dirs}")


if __name__ == "__main__":
    main()
