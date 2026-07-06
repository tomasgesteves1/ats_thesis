#!/usr/bin/env python3
"""
UAV Offline System Identification and Model Validation Runner
This script parses a recorded ROS 2 bag file using the rosbags library,
identifies the attitude time constants and vertical thrust/drag coefficients,
and saves a validation plot. It runs as pure Python (no ROS 2 dependencies needed).

Author: Tomas (Tethered UAV-USV Project)
Language: Python 3
"""

import sys
import os
import argparse
import matplotlib.pyplot as plt
import numpy as np

from uav_sysid.bag_reader import read_bag
from uav_sysid.optimizer import identify_attitude_axis, identify_vertical_dynamics

def expand_mask(mask, padding=100):
    expanded = mask.copy()
    indices = np.where(mask)[0]
    for idx in indices:
        start = max(0, idx - padding)
        end = min(len(mask), idx + padding + 1)
        expanded[start:end] = True
    return expanded

def main():
    parser = argparse.ArgumentParser(description='UAV Offline System Identification and Model Validation tool.')
    parser.add_argument('--bag', type=str, required=True, help='Path to the ROS 2 bag folder or database file')
    parser.add_argument('--output', type=str, default='uav_sysid_results.png', help='Path to save the validation plot')
    args = parser.parse_args()

    bag_path = args.bag
    if not os.path.exists(bag_path):
        print(f"Error: Bag path '{bag_path}' does not exist.")
        sys.exit(1)

    print(f"Reading bag data from '{bag_path}'...")
    try:
        data = read_bag(bag_path)
    except Exception as e:
        print(f"Error reading bag: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    t = data['time']
    phase = data['phase']

    # Detect if we have valid phase information from diagnostics
    has_phases = np.any(phase > 0)
    
    if has_phases:
        print("Using diagnostics topic for trajectory phase segmentation.")
        roll_mask = (phase >= 5) & (phase <= 7)
        pitch_mask = (phase >= 2) & (phase <= 4)
        z_mask = (phase >= 8) & (phase <= 10)
    else:
        print("No diagnostics phase data found in bag. Segmenting using command thresholds...")
        
        # Roll: active when roll command deviates from 0.0
        roll_active = np.abs(data['roll_cmd']) > 0.005
        roll_mask = expand_mask(roll_active, padding=100)
        
        # Pitch: active when pitch command deviates from 0.0
        pitch_active = np.abs(data['pitch_cmd']) > 0.005
        pitch_mask = expand_mask(pitch_active, padding=100)
        
        # Z: active when thrust command deviates significantly from hover thrust
        z_active = np.abs(data['thrust_cmd'] - data['hover_thrust']) > 0.015
        z_mask = expand_mask(z_active, padding=100)

    # 1. Identify Roll (Phases 5, 6, 7 are Roll Steps)
    if np.any(roll_mask):
        t_roll = t[roll_mask]
        roll_cmd = data['roll_cmd'][roll_mask]
        roll_meas = data['roll'][roll_mask]
        
        tau_roll, r2_roll, roll_sim = identify_attitude_axis(t_roll, roll_cmd, roll_meas, "Roll")
        print(f"\n--- Roll Axis Identification ---")
        print(f"Estimated Time Constant (tau): {tau_roll:.4f} s")
        print(f"Equivalent Bandwidth (1/tau):  {1.0/tau_roll:.2f} rad/s")
        print(f"Fit Metric (R2):               {r2_roll * 100.0:.2f}%")
    else:
        print("\nWarning: No Roll excitation phases found in the bag data.")
        tau_roll, r2_roll = None, None

    # 2. Identify Pitch (Phases 2, 3, 4 are Pitch Steps)
    if np.any(pitch_mask):
        t_pitch = t[pitch_mask]
        pitch_cmd = data['pitch_cmd'][pitch_mask]
        pitch_meas = data['pitch'][pitch_mask]
        
        tau_pitch, r2_pitch, pitch_sim = identify_attitude_axis(t_pitch, pitch_cmd, pitch_meas, "Pitch")
        print(f"\n--- Pitch Axis Identification ---")
        print(f"Estimated Time Constant (tau): {tau_pitch:.4f} s")
        print(f"Equivalent Bandwidth (1/tau):  {1.0/tau_pitch:.2f} rad/s")
        print(f"Fit Metric (R2):               {r2_pitch * 100.0:.2f}%")
    else:
        print("\nWarning: No Pitch excitation phases found in the bag data.")
        tau_pitch, r2_pitch = None, None

    # 3. Identify Vertical Dynamics (Phases 8, 9, 10 are Z Steps)
    if np.any(z_mask):
        t_z = t[z_mask]
        thrust_cmd = data['thrust_cmd'][z_mask]
        z_meas = data['z'][z_mask]
        vz_meas = data['vz'][z_mask]
        
        c1, c2, r2_z, az_meas, az_sim = identify_vertical_dynamics(t_z, thrust_cmd, z_meas, vz_meas)
        print(f"\n--- Vertical Dynamics Identification ---")
        print(f"Thrust Acceleration Gain (c1): {c1:.4f} m/s^2 per unit thrust")
        print(f"Vertical Drag Coefficient (c2): {c2:.4f} s^-1")
        print(f"Acceleration Fit Metric (R2):  {r2_z * 100.0:.2f}%")
        
        # Calculate predicted hover thrust where accel is 0: c1 * T_hover - g = 0 => T_hover = g / c1
        predicted_t_hover = 9.81 / c1 if c1 > 0 else 0.0
        print(f"Predicted Hover Thrust:        {predicted_t_hover:.4f}")
    else:
        print("\nWarning: No Z excitation phases found in the bag data.")
        c1, c2, r2_z = None, None, None

    # --- Plotting Results ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=False)
    
    # Subplot 1: Roll
    if tau_roll is not None:
        axs[0].plot(t_roll - t_roll[0], np.degrees(roll_cmd), '--', color='gray', label='Command (MPC)')
        axs[0].plot(t_roll - t_roll[0], np.degrees(roll_meas), label='Measured (Gazebo)', color='blue', alpha=0.7)
        axs[0].plot(t_roll - t_roll[0], np.degrees(roll_sim), label=f'Model Fit (tau={tau_roll:.3f}s)', color='red', linewidth=2)
        axs[0].set_title(f"Roll Axis Response (R²={r2_roll * 100.0:.1f}%)")
        axs[0].set_ylabel("Roll Angle (deg)")
        axs[0].grid(True)
        axs[0].legend()
        
    # Subplot 2: Pitch
    if tau_pitch is not None:
        axs[1].plot(t_pitch - t_pitch[0], np.degrees(pitch_cmd), '--', color='gray', label='Command (MPC)')
        axs[1].plot(t_pitch - t_pitch[0], np.degrees(pitch_meas), label='Measured (Gazebo)', color='blue', alpha=0.7)
        axs[1].plot(t_pitch - t_pitch[0], np.degrees(pitch_sim), label=f'Model Fit (tau={tau_pitch:.3f}s)', color='red', linewidth=2)
        axs[1].set_title(f"Pitch Axis Response (R²={r2_pitch * 100.0:.1f}%)")
        axs[1].set_ylabel("Pitch Angle (deg)")
        axs[1].grid(True)
        axs[1].legend()

    # Subplot 3: Vertical acceleration
    if c1 is not None:
        axs[2].plot(t_z - t_z[0], az_meas, label='Measured Accel (Gazebo)', color='blue', alpha=0.7)
        axs[2].plot(t_z - t_z[0], az_sim, label=f'Model Accel (c1={c1:.2f}, c2={c2:.2f})', color='red', linewidth=2)
        axs[2].set_title(f"Vertical Dynamics Response (R²={r2_z * 100.0:.1f}%)")
        axs[2].set_ylabel("Acceleration (m/s²)")
        axs[2].set_xlabel("Phase Elapsed Time (s)")
        axs[2].grid(True)
        axs[2].legend()
        
    plt.tight_layout()
    plt.savefig(args.output)
    print(f"\nSaved validation plot to: {args.output}")
    print("Close the plot window to finish script.")
    plt.show()

if __name__ == '__main__':
    main()
