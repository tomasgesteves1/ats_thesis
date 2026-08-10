#!/usr/bin/env python3
"""
Model Validation Script for Azimuthal WAM-V (Tethered UAV-USV Project)
This script loads ROS 2 bags (MCAP/SQLite), processes azimuthal thruster commands (forces and angles)
and odometry, runs the Fossen 3-DOF model simulator using Ground Truth parameters, and compares
the predicted vs. actual boat states.
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

typestore = get_typestore(Stores.LATEST)

# Add current directory to path to import wamv_model
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import wamv_model


def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to Euler yaw (psi).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def rk4_step(f, x, u, dt):
    """
    Perform a single step of 4th Order Runge-Kutta integration.
    """
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def parse_ros_bag(bag_path, odom_topic, left_thrust_topic, right_thrust_topic, left_pos_topic, right_pos_topic):
    """
    Reads odometry and motor commands (forces & angles) from a ROS 2 bag.
    """
    print(f"Reading ROS 2 bag from: {bag_path}...")
    
    odom_data = []         # Elements: (t_sec, x, y, psi, u, v, r)
    thrust_left_data = []   # Elements: (t_sec, thrust)
    thrust_right_data = []  # Elements: (t_sec, thrust)
    pos_left_data = []      # Elements: (t_sec, angle_rad)
    pos_right_data = []     # Elements: (t_sec, angle_rad)
    
    with Reader(bag_path) as reader:
        # Check if topics exist in the bag
        available_topics = {conn.topic: conn.msgtype for conn in reader.connections}
        
        print("\nAvailable topics in bag:")
        for t, mtype in available_topics.items():
            print(f"  - {t} ({mtype})")
            
        if odom_topic not in available_topics:
            print(f"\n[WARNING] Odometry topic '{odom_topic}' not found in bag.")
        if left_thrust_topic not in available_topics:
            print(f"[WARNING] Left thrust topic '{left_thrust_topic}' not found in bag.")
        if right_thrust_topic not in available_topics:
            print(f"[WARNING] Right thrust topic '{right_thrust_topic}' not found in bag.")
        if left_pos_topic not in available_topics:
            print(f"[WARNING] Left angle topic '{left_pos_topic}' not found in bag.")
        if right_pos_topic not in available_topics:
            print(f"[WARNING] Right angle topic '{right_pos_topic}' not found in bag.")

        # Build mapping from wall time to simulation time using Odometry messages
        odom_conns = [conn for conn in reader.connections if conn.topic == odom_topic]
        wall_times = []
        sim_times = []
        if odom_conns:
            for connection, timestamp, rawdata in reader.messages(connections=odom_conns):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                wall_times.append(timestamp / 1e9)
                sim_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
        if len(wall_times) > 0:
            wall_times = np.array(wall_times)
            sim_times = np.array(sim_times)
            first_wall = wall_times[0]
            first_sim = sim_times[0]
            
            def get_sim_time(t_wall_ns):
                t_w = t_wall_ns / 1e9
                return np.interp(t_w - first_wall, wall_times - first_wall, sim_times - first_sim)
        else:
            first_timestamp = None
            def get_sim_time(t_wall_ns):
                nonlocal first_timestamp
                if first_timestamp is None:
                    first_timestamp = t_wall_ns
                return (t_wall_ns - first_timestamp) / 1e9

        for connection, timestamp, rawdata in reader.messages():
            t_sec = get_sim_time(timestamp)
            
            if connection.topic == odom_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                # Retrieve orientation
                ox = msg.pose.pose.orientation.x
                oy = msg.pose.pose.orientation.y
                oz = msg.pose.pose.orientation.z
                ow = msg.pose.pose.orientation.w
                psi = euler_from_quaternion(ox, oy, oz, ow)
                
                # Positions
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                
                # Velocities in body frame
                u_vel = msg.twist.twist.linear.x
                v_vel = msg.twist.twist.linear.y
                r_vel = msg.twist.twist.angular.z
                
                odom_data.append((t_sec, px, py, psi, u_vel, v_vel, r_vel))
                
            elif connection.topic == left_thrust_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                thrust_left_data.append((t_sec, msg.data))
                
            elif connection.topic == right_thrust_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                thrust_right_data.append((t_sec, msg.data))
                
            elif connection.topic == left_pos_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                pos_left_data.append((t_sec, msg.data))
                
            elif connection.topic == right_pos_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                pos_right_data.append((t_sec, msg.data))
                
    return (np.array(odom_data), 
            np.array(thrust_left_data), 
            np.array(thrust_right_data),
            np.array(pos_left_data),
            np.array(pos_right_data))


def compute_fit_percentage(y_actual, y_predicted):
    """
    Computes the FIT R^2 percentage metric for model verification.
    """
    mean_actual = np.mean(y_actual)
    numerator = np.sum((y_actual - y_predicted) ** 2)
    denominator = np.sum((y_actual - mean_actual) ** 2)
    
    if denominator == 0:
        return 0.0
    return 100.0 * (1.0 - np.sqrt(numerator / denominator))


def main():
    parser = argparse.ArgumentParser(description="Validate WAM-V Azimuthal Grey-Box model against Gazebo simulation bags.")
    parser.add_argument("bag_path", type=str, help="Path to the ROS 2 bag folder")
    parser.add_argument("--odom", type=str, default="/boat/ground_truth/odometry", help="Boat Odometry topic")
    parser.add_argument("--left_thrust", type=str, default="/boat/thrusters/left/thrust", help="Left motor thrust topic")
    parser.add_argument("--right_thrust", type=str, default="/boat/thrusters/right/thrust", help="Right motor thrust topic")
    parser.add_argument("--left_pos", type=str, default="/boat/thrusters/left/pos", help="Left motor position topic")
    parser.add_argument("--right_pos", type=str, default="/boat/thrusters/right/pos", help="Right motor position topic")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation step size (s)")
    parser.add_argument("--no-show", action="store_true", help="Do not show matplotlib interactive plots (still saves files)")
    
    args = parser.parse_args()
    
    # Read the data from the bag
    odom, t_left, t_right, a_left, a_right = parse_ros_bag(
        args.bag_path, args.odom, args.left_thrust, args.right_thrust, args.left_pos, args.right_pos
    )
    
    if len(odom) == 0:
        print("[ERROR] No odometry data found. Validation cannot proceed.")
        return
        
    # Clean yaw rate outliers (e.g. Gazebo numerical spikes of ~500 rad/s)
    # A physical yaw rate limit for a WAM-V is around 2.0-3.0 rad/s.
    r_threshold = 3.0
    r_raw = odom[:, 6]
    outliers = np.abs(r_raw) > r_threshold
    if np.any(outliers):
        num_outliers = np.sum(outliers)
        print(f"\n[INFO] Detected {num_outliers} yaw rate (r) outliers exceeding {r_threshold} rad/s. Cleaning using interpolation...")
        non_outlier_idx = np.where(~outliers)[0]
        if len(non_outlier_idx) > 0:
            # Interpolate only the outlier values using time-based interpolation
            odom[outliers, 6] = np.interp(odom[outliers, 0], odom[~outliers, 0], r_raw[~outliers])
        else:
            odom[:, 6] = 0.0
            print("[WARNING] All yaw rate data points were classified as outliers!")
            
    print(f"\nRead {len(odom)} odom points, {len(t_left)} left thrust, {len(t_right)} right thrust.")
    print(f"Read {len(a_left)} left angle, {len(a_right)} right angle commands.")
    
    # Define common time grid
    t_start = max(odom[0, 0], t_left[0, 0] if len(t_left) > 0 else 0.0, t_right[0, 0] if len(t_right) > 0 else 0.0)
    t_end = min(odom[-1, 0], t_left[-1, 0] if len(t_left) > 0 else odom[-1, 0], t_right[-1, 0] if len(t_right) > 0 else odom[-1, 0])
    
    t_grid = np.arange(t_start, t_end, args.dt)
    n_steps = len(t_grid)
    
    print(f"Aligning and interpolating signals from t = {t_start:.2f}s to {t_end:.2f}s ({n_steps} steps at dt={args.dt}s)...")
    
    # Interpolate odometry (ground truth comparison)
    x_actual = np.interp(t_grid, odom[:, 0], odom[:, 1])
    y_actual = np.interp(t_grid, odom[:, 0], odom[:, 2])
    psi_actual = np.unwrap(np.interp(t_grid, odom[:, 0], odom[:, 3]))
    u_actual = np.interp(t_grid, odom[:, 0], odom[:, 4])
    v_actual = np.interp(t_grid, odom[:, 0], odom[:, 5])
    r_actual = np.interp(t_grid, odom[:, 0], odom[:, 6])
    
    # Interpolate motor thrusts (inputs to simulation)
    thrust_left = np.interp(t_grid, t_left[:, 0], t_left[:, 1]) if len(t_left) > 0 else np.zeros_like(t_grid)
    thrust_right = np.interp(t_grid, t_right[:, 0], t_right[:, 1]) if len(t_right) > 0 else np.zeros_like(t_grid)
    
    # Interpolate motor angles (inputs to simulation)
    angle_left = np.interp(t_grid, a_left[:, 0], a_left[:, 1]) if len(a_left) > 0 else np.zeros_like(t_grid)
    angle_right = np.interp(t_grid, a_right[:, 0], a_right[:, 1]) if len(a_right) > 0 else np.zeros_like(t_grid)
        
    # Simulate model dynamic response (RK4 integration)
    print("Integrating Fossen dynamic equations for azimuthal configuration...")
    x_sim = np.zeros((n_steps, 6))
    
    # Initialize simulation state with the first odom measurement
    x_sim[0] = np.array([x_actual[0], y_actual[0], psi_actual[0], u_actual[0], v_actual[0], r_actual[0]])
    
    for k in range(n_steps - 1):
        # Apply Thrust Allocation Map (Azimuthal term) to find virtual forces X, Y, N
        u_virtual = wamv_model.thrust_allocation_map(
            thrust_left[k], thrust_right[k], angle_left[k], angle_right[k]
        )
        
        # Integrate forward by dt using Runge-Kutta 4
        x_sim[k+1] = rk4_step(wamv_model.wamv_dynamics_numerical, x_sim[k], u_virtual, args.dt)
        
    # Unpack simulation results
    x_pred = x_sim[:, 0]
    y_pred = x_sim[:, 1]
    psi_pred = np.unwrap(x_sim[:, 2])
    u_pred = x_sim[:, 3]
    v_pred = x_sim[:, 4]
    r_pred = x_sim[:, 5]
    
    # Compute error metrics (RMS & FIT percentage)
    rms_u = np.sqrt(np.mean((u_actual - u_pred)**2))
    rms_v = np.sqrt(np.mean((v_actual - v_pred)**2))
    rms_r = np.sqrt(np.mean((r_actual - r_pred)**2))
    
    fit_u = compute_fit_percentage(u_actual, u_pred)
    fit_v = compute_fit_percentage(v_actual, v_pred)
    fit_r = compute_fit_percentage(r_actual, r_pred)
    
    print("\n" + "="*45)
    print("            VALIDATION METRICS RESULTS (AZIMUTHAL)")
    print("="*45)
    print(f"Surge Velocity (u):  RMS Error = {rms_u:.4f} m/s | FIT = {fit_u:.2f}%")
    print(f"Sway Velocity (v):   RMS Error = {rms_v:.4f} m/s | FIT = {fit_v:.2f}%")
    print(f"Yaw Rate (r):        RMS Error = {rms_r:.4f} rad/s| FIT = {fit_r:.2f}%")
    print("="*45)
    
    # Plot results
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    
    # Plot input forces & angles
    axs[0].plot(t_grid - t_start, thrust_left, 'b--', alpha=0.7, label='Thrust Left (T_L)')
    axs[0].plot(t_grid - t_start, thrust_right, 'r:', alpha=0.7, label='Thrust Right (T_R)')
    # Compute virtual force inputs for plotting
    v_forces = np.array([
        wamv_model.thrust_allocation_map(tl, tr, al, ar) 
        for tl, tr, al, ar in zip(thrust_left, thrust_right, angle_left, angle_right)
    ])
    axs[0].plot(t_grid - t_start, v_forces[:, 0], 'k-', label='Virtual Surge (X)')
    axs[0].plot(t_grid - t_start, v_forces[:, 1], 'c-', label='Virtual Sway (Y)')
    axs[0].set_ylabel('Force [N] / Torque [N*m]')
    axs[0].set_title('Control Inputs (Thruster & Virtual Forces)')
    axs[0].grid(True)
    axs[0].legend()
    
    # Plot Surge velocity comparison
    axs[1].plot(t_grid - t_start, u_actual, 'k-', label='Gazebo (Ground Truth)')
    axs[1].plot(t_grid - t_start, u_pred, 'g--', label=f'Model Prediction (FIT = {fit_u:.1f}%)')
    axs[1].set_ylabel('Surge (u) [m/s]')
    axs[1].grid(True)
    axs[1].legend()
    
    # Plot Sway velocity comparison
    axs[2].plot(t_grid - t_start, v_actual, 'k-', label='Gazebo (Ground Truth)')
    axs[2].plot(t_grid - t_start, v_pred, 'g--', label=f'Model Prediction (FIT = {fit_v:.1f}%)')
    axs[2].set_ylabel('Sway (v) [m/s]')
    axs[2].grid(True)
    axs[2].legend()
    
    # Plot Yaw rate comparison
    axs[3].plot(t_grid - t_start, r_actual, 'k-', label='Gazebo (Ground Truth)')
    axs[3].plot(t_grid - t_start, r_pred, 'g--', label=f'Model Prediction (FIT = {fit_r:.1f}%)')
    axs[3].set_ylabel('Yaw Rate (r) [rad/s]')
    axs[3].set_xlabel('Time [s]')
    axs[3].grid(True)
    axs[3].legend()
    
    plt.tight_layout()
    
    fig_dir = "/home/tomas/ats_ws/src/latex/tese/Figures/modeling"
    os.makedirs(fig_dir, exist_ok=True)
    
    # Save the velocities validation plot
    plot_filename = os.path.join(fig_dir, "usv_validation_velocities.pdf")
    plt.savefig(plot_filename, format='pdf')
    print(f"\nSaved validation plot results to: {plot_filename}")
    
    # Also plot X-Y Trajectory comparison
    plt.figure(figsize=(8, 8))
    plt.plot(x_actual, y_actual, 'k-', label='Gazebo Trajectory')
    plt.plot(x_pred, y_pred, 'g--', label='Model Simulated Trajectory')
    plt.xlabel('X position [m]')
    plt.ylabel('Y position [m]')
    plt.title('Traj. Comparison (Body-to-Global Kinematics Validation)')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    
    traj_filename = os.path.join(fig_dir, "usv_trajectory_validation.pdf")
    plt.savefig(traj_filename, format='pdf')
    print(f"Saved trajectory plot results to: {traj_filename}")
    
    # Generate the excitation plot (thrust and steer angles over time)
    fig_ex, (ax_ex_thrust, ax_ex_angle) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    ax_ex_thrust.plot(t_grid - t_start, thrust_left, 'b-', alpha=0.8, label='Left Thruster ($T_L$)')
    ax_ex_thrust.plot(t_grid - t_start, thrust_right, 'r--', alpha=0.8, label='Right Thruster ($T_R$)')
    ax_ex_thrust.set_ylabel('Thrust [N]')
    ax_ex_thrust.set_title('WAM-V Excitation Signals - Thruster Forces')
    ax_ex_thrust.grid(True)
    ax_ex_thrust.legend()
    
    ax_ex_angle.plot(t_grid - t_start, np.degrees(angle_left), 'b-', alpha=0.8, label='Left Steering Angle ($\\alpha_L$)')
    ax_ex_angle.plot(t_grid - t_start, np.degrees(angle_right), 'r--', alpha=0.8, label='Right Steering Angle ($\\alpha_R$)')
    ax_ex_angle.set_ylabel('Steering Angle [deg]')
    ax_ex_angle.set_xlabel('Time [s]')
    ax_ex_angle.set_title('WAM-V Excitation Signals - Steering Angles')
    ax_ex_angle.grid(True)
    ax_ex_angle.legend()
    
    plt.tight_layout()
    ex_filename = os.path.join(fig_dir, "usv_excitation.pdf")
    plt.savefig(ex_filename, format='pdf')
    print(f"Saved excitation signals plot to: {ex_filename}")
    
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
