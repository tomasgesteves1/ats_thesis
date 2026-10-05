#!/usr/bin/env python3
"""
USV Dynamics Validation Script (Tethered UAV-USV Project)
Validates the first-principles 3-DOF planar Fossen model using nominal simulator
parameters against Gazebo ground truth telemetry from rosbag recordings.
Strictly relies on simulator specifications without empirical data fitting.
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
    COLOR_CYCLE
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


def simulate_model(data):
    t = data['t']
    dt = data['dt']
    N = len(t)
    
    # Thruster allocation (Thesis Eq. 3.18, 3.19 / Control Eq. 4.X)
    T_L = data['T_L']
    T_R = data['T_R']
    a_L = data['alpha_L']
    a_R = data['alpha_R']
    
    X = T_L * np.cos(a_L) + T_R * np.cos(a_R)
    Y = T_L * np.sin(a_L) + T_R * np.sin(a_R)
    N_moment = -THRUSTER_X_ARM * (T_L * np.sin(a_L) + T_R * np.sin(a_R)) - THRUSTER_Y_ARM * (T_L * np.cos(a_L) - T_R * np.cos(a_R))
    
    m = MASS_NOMINAL
    Iz = IZZ_NOMINAL
    Xu = X_U_NOMINAL
    Xuu = X_UU_NOMINAL
    Yv = Y_V_NOMINAL
    Yvv = Y_VV_NOMINAL
    Nr = N_R_NOMINAL
    Nrr = N_RR_NOMINAL
    
    # States: [x, y, psi, u, v, r]
    sim_states = np.zeros((N, 6))
    sim_states[0] = [
        data['x_gt'][0],
        data['y_gt'][0],
        data['psi_gt'][0],
        data['u_gt'][0],
        data['v_gt'][0],
        data['r_gt'][0]
    ]
    
    def f_ode(s, Xin, Yin, Nin):
        x, y, psi, u, v, r = s
        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)
        psi_dot = r
        u_dot = (Xin - (Xu + Xuu * abs(u)) * u) / m + v * r
        v_dot = (Yin - (Yv + Yvv * abs(v)) * v) / m - u * r
        r_dot = (Nin - (Nr + Nrr * abs(r)) * r) / Iz
        return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])
    
    for k in range(N - 1):
        s_k = sim_states[k]
        k1 = f_ode(s_k, X[k], Y[k], N_moment[k])
        k2 = f_ode(s_k + 0.5 * dt * k1, 0.5 * (X[k] + X[k+1]), 0.5 * (Y[k] + Y[k+1]), 0.5 * (N_moment[k] + N_moment[k+1]))
        k3 = f_ode(s_k + 0.5 * dt * k2, 0.5 * (X[k] + X[k+1]), 0.5 * (Y[k] + Y[k+1]), 0.5 * (N_moment[k] + N_moment[k+1]))
        k4 = f_ode(s_k + dt * k3, X[k+1], Y[k+1], N_moment[k+1])
        sim_states[k+1] = s_k + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        
    return {
        'X': X,
        'Y': Y,
        'N_moment': N_moment,
        'x_sim': sim_states[:, 0],
        'y_sim': sim_states[:, 1],
        'psi_sim': sim_states[:, 2],
        'u_sim': sim_states[:, 3],
        'v_sim': sim_states[:, 4],
        'r_sim': sim_states[:, 5]
    }


def compute_metrics(y_true, y_pred):
    rmse = np.sqrt(np.mean((y_true - y_pred)**2))
    ss_tot = np.sum((y_true - np.mean(y_true))**2)
    ss_res = np.sum((y_true - y_pred)**2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 1e-6 else 0.0
    mae = np.mean(np.abs(y_true - y_pred))
    max_err = np.max(np.abs(y_true - y_pred))
    return {'rmse': rmse, 'r2': r2 * 100.0, 'mae': mae, 'max': max_err}


def plot_results(data, sim_res, output_dirs):
    set_thesis_style()
    t = data['t']
    
    # --------------------------------------------------------------------------
    # Figure 1: Body Velocities (u, v, r)
    # --------------------------------------------------------------------------
    fig1, axs1 = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)
    
    # Surge u
    axs1[0].plot(t, data['u_gt'], color=GROUND_TRUTH_COLOR, label='Ground Truth', linewidth=1.4)
    axs1[0].plot(t, sim_res['u_sim'], color=COLOR_CYCLE[0], label='Nominal Model', linewidth=1.4, linestyle='--')
    axs1[0].set_ylabel('$u$ [m/s]')
    axs1[0].grid(True, linestyle=':', alpha=0.6)
    axs1[0].legend(loc='upper right', framealpha=0.9)
    
    # Sway v
    axs1[1].plot(t, data['v_gt'], color=GROUND_TRUTH_COLOR, label='Ground Truth', linewidth=1.4)
    axs1[1].plot(t, sim_res['v_sim'], color=COLOR_CYCLE[0], label='Nominal Model', linewidth=1.4, linestyle='--')
    axs1[1].set_ylabel('$v$ [m/s]')
    axs1[1].grid(True, linestyle=':', alpha=0.6)
    
    # Yaw rate r
    axs1[2].plot(t, np.rad2deg(data['r_gt']), color=GROUND_TRUTH_COLOR, label='Ground Truth', linewidth=1.4)
    axs1[2].plot(t, np.rad2deg(sim_res['r_sim']), color=COLOR_CYCLE[0], label='Nominal Model', linewidth=1.4, linestyle='--')
    axs1[2].set_ylabel('$r$ [deg/s]')
    axs1[2].set_xlabel('Time [s]')
    axs1[2].grid(True, linestyle=':', alpha=0.6)
    
    plt.tight_layout()
    for d in output_dirs:
        save_figure(fig1, os.path.join(d, 'usv_velocities_validation'), save_png=True)
    plt.close(fig1)
    
    # --------------------------------------------------------------------------
    # Figure 2: Trajectory XY and Heading
    # --------------------------------------------------------------------------
    fig2, axs2 = plt.subplots(1, 2, figsize=(7.6, 3.8))
    
    # Trajectory XY
    axs2[0].plot(data['x_gt'], data['y_gt'], color=GROUND_TRUTH_COLOR, label='Ground Truth', linewidth=1.5)
    axs2[0].plot(sim_res['x_sim'], sim_res['y_sim'], color=COLOR_CYCLE[0], label='Nominal Model', linewidth=1.5, linestyle='--')
    axs2[0].plot(data['x_gt'][0], data['y_gt'][0], 'ko', markersize=5, label='Start')
    axs2[0].set_xlabel('$x$ [m]')
    axs2[0].set_ylabel('$y$ [m]')
    axs2[0].grid(True, linestyle=':', alpha=0.6)
    axs2[0].axis('equal')
    axs2[0].legend(loc='lower left', framealpha=0.9)
    
    # Heading psi
    axs2[1].plot(t, np.rad2deg(data['psi_gt']), color=GROUND_TRUTH_COLOR, label='Ground Truth', linewidth=1.4)
    axs2[1].plot(t, np.rad2deg(sim_res['psi_sim']), color=COLOR_CYCLE[0], label='Nominal Model', linewidth=1.4, linestyle='--')
    axs2[1].set_xlabel('Time [s]')
    axs2[1].set_ylabel(r'Heading $\psi$ [deg]')
    axs2[1].grid(True, linestyle=':', alpha=0.6)
    axs2[1].legend(loc='upper right', framealpha=0.9)
    
    plt.tight_layout()
    for d in output_dirs:
        save_figure(fig2, os.path.join(d, 'usv_trajectory_validation'), save_png=True)
    plt.close(fig2)

    # --------------------------------------------------------------------------
    # Figure 3: Actuator Commands
    # --------------------------------------------------------------------------
    fig3, axs3 = plt.subplots(2, 1, figsize=(7.2, 4.4), sharex=True)
    
    # Thruster Forces
    axs3[0].plot(t, data['T_L'], color=COLOR_CYCLE[0], label='Port ($T_L$)', linewidth=1.3)
    axs3[0].plot(t, data['T_R'], color=COLOR_CYCLE[1], label='Starboard ($T_R$)', linewidth=1.3)
    axs3[0].axhline(510.0, color='r', linestyle=':', alpha=0.5, label='Fwd Limit ($510\\,\\mathrm{N}$)')
    axs3[0].axhline(-380.0, color='r', linestyle=':', alpha=0.5, label='Rev Limit ($-380\\,\\mathrm{N}$)')
    axs3[0].set_ylabel('Thrust [N]')
    axs3[0].grid(True, linestyle=':', alpha=0.6)
    axs3[0].legend(loc='upper right', framealpha=0.9, ncol=2)
    
    # Azimuth Angles
    axs3[1].plot(t, np.rad2deg(data['alpha_L']), color=COLOR_CYCLE[0], label=r'Port ($\alpha_L$)', linewidth=1.3)
    axs3[1].plot(t, np.rad2deg(data['alpha_R']), color=COLOR_CYCLE[1], label=r'Starboard ($\alpha_R$)', linewidth=1.3)
    axs3[1].axhline(30.0, color='r', linestyle=':', alpha=0.5)
    axs3[1].axhline(-30.0, color='r', linestyle=':', alpha=0.5)
    axs3[1].set_ylabel('Azimuth [deg]')
    axs3[1].set_xlabel('Time [s]')
    axs3[1].grid(True, linestyle=':', alpha=0.6)
    axs3[1].legend(loc='upper right', framealpha=0.9)
    
    plt.tight_layout()
    for d in output_dirs:
        save_figure(fig3, os.path.join(d, 'usv_actuator_inputs'), save_png=True)
    plt.close(fig3)
    
    print("[INFO] Plots successfully generated and exported.")


def main():
    args = parse_args()
    data = load_telemetry(args.bag)
    sim_res = simulate_model(data)
    
    # Calculate performance metrics
    m_u = compute_metrics(data['u_gt'], sim_res['u_sim'])
    m_v = compute_metrics(data['v_gt'], sim_res['v_sim'])
    m_r = compute_metrics(data['r_gt'], sim_res['r_sim'])
    m_psi = compute_metrics(np.rad2deg(data['psi_gt']), np.rad2deg(sim_res['psi_sim']))
    m_x = compute_metrics(data['x_gt'], sim_res['x_sim'])
    m_y = compute_metrics(data['y_gt'], sim_res['y_sim'])
    
    pos_err = np.sqrt((data['x_gt'] - sim_res['x_sim'])**2 + (data['y_gt'] - sim_res['y_sim'])**2)
    rmse_pos = np.sqrt(np.mean(pos_err**2))
    max_pos = np.max(pos_err)
    
    dx = np.diff(data['x_gt'])
    dy = np.diff(data['y_gt'])
    total_distance = np.sum(np.sqrt(dx**2 + dy**2))
    
    print("\n" + "=" * 65)
    print("      USV DYNAMICS FIRST-PRINCIPLES VALIDATION REPORT")
    print("=" * 65)
    print(f"Trajectory Duration: {data['t'][-1]:.2f} s | Distance: {total_distance:.2f} m")
    print("-" * 65)
    print(f"{'State':<15} | {'RMSE':<12} | {'MAE':<12} | {'R^2 (%)':<10} | {'Max Err':<10}")
    print("-" * 65)
    print(f"{'Surge u (m/s)':<15} | {m_u['rmse']:<12.4f} | {m_u['mae']:<12.4f} | {m_u['r2']:<10.2f} | {m_u['max']:<10.4f}")
    print(f"{'Sway v (m/s)':<15} | {m_v['rmse']:<12.4f} | {m_v['mae']:<12.4f} | {m_v['r2']:<10.2f} | {m_v['max']:<10.4f}")
    print(f"{'Yaw r (deg/s)':<15} | {np.rad2deg(m_r['rmse']):<12.4f} | {np.rad2deg(m_r['mae']):<12.4f} | {m_r['r2']:<10.2f} | {np.rad2deg(m_r['max']):<10.4f}")
    print(f"{'Heading (deg)':<15} | {m_psi['rmse']:<12.4f} | {m_psi['mae']:<12.4f} | {m_psi['r2']:<10.2f} | {m_psi['max']:<10.4f}")
    print(f"{'Position X (m)':<15} | {m_x['rmse']:<12.4f} | {m_x['mae']:<12.4f} | {m_x['r2']:<10.2f} | {m_x['max']:<10.4f}")
    print(f"{'Position Y (m)':<15} | {m_y['rmse']:<12.4f} | {m_y['mae']:<12.4f} | {m_y['r2']:<10.2f} | {m_y['max']:<10.4f}")
    print("-" * 65)
    print(f"Euclidean Position RMSE: {rmse_pos:.3f} m ({rmse_pos/total_distance*100:.2f}% of path) | Max Error: {max_pos:.3f} m")
    print("=" * 65 + "\n")
    
    # Destination directories
    plot_dir = os.path.join(CURRENT_DIR, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    
    out_dirs = [plot_dir]
    if args.export:
        thesis_dir = os.path.abspath(os.path.join(CURRENT_DIR, "../../latex/tese/Implementation/figures/02_model_validation"))
        os.makedirs(thesis_dir, exist_ok=True)
        out_dirs.append(thesis_dir)
        
    plot_results(data, sim_res, out_dirs)


if __name__ == "__main__":
    main()
