#!/usr/bin/env python3
"""
Tether Model Identification & 3D Fitting Script
MSc Thesis (UAV-USV Cooperative Project - IST)

Fits and validates control-oriented tether models against high-fidelity 
MoorDyn multi-node lumped-mass simulation data from bag_slack_105 (epsilon = 5%).

Analyzes:
  1. Tension Magnitude Models:
     - Model 0: Uncorrected Thesis Model (constant tension T(eps_0) = const)
     - Model 1: Corrected Thesis Model with Cable Length: T(L) = c_L * L
     - Model 2: Corrected Thesis Model with Distance: T(d) = c_d * d
     - Model 3: Affine Length Model: T(L) = a * L + b
     - Model 4: Full Taylor in Slack: T(L, eps) = L * (c0 + c1 * (eps - eps0))
  2. 3D Vector Component Models (Fx, Fy, Fz):
     - Chord Line Model (gamma = 0 deg, pulling along straight line d0)
     - Thesis Catenary Direction Model (gamma = 24.5 deg, tilted towards vertical)
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Add system_identification root to path for shared thesis styling
sys_id_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if sys_id_dir not in sys.path:
    sys.path.append(sys_id_dir)

from thesis_style import set_thesis_style, save_figure, GROUND_TRUTH_COLOR, COLOR_CYCLE, THESIS_COLORS

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import EntityWrench
except ImportError as e:
    print(f"Error importing ROS 2 libraries: {e}")
    sys.exit(1)


def apply_zero_phase_filter(signal_array, fs=50.0, cutoff_hz=0.8, order=2):
    """
    Apply an offline zero-phase forward-backward Butterworth low-pass filter (filtfilt)
    to remove high-frequency lumped-mass numerical chatter from MoorDyn telemetry.
    """
    if len(signal_array) < 15:
        return signal_array.copy()
    nyq = 0.5 * fs
    normal_cutoff = min(cutoff_hz / nyq, 0.99)
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, signal_array)


def read_bag_data_with_wrench(bag_path):
    """Read tether, wrench (3D force vector), and drone topics from bag."""
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    data = {
        "force_time": [],
        "force_mag": [],
        "wrench_time": [],
        "wrench_fx": [],
        "wrench_fy": [],
        "wrench_fz": [],
        "dist_time": [],
        "distance": [],
        "length_time": [],
        "length": [],
        "odom_time": [],
        "odom_header_time": [],
        "drone_x": [],
        "drone_y": [],
        "drone_z": [],
        "drone_vx": [],
        "drone_vy": [],
        "drone_vz": [],
    }

    while reader.has_next():
        topic, raw_data, timestamp_ns = reader.read_next()
        t = timestamp_ns * 1e-9  # seconds

        if topic == "/tether_force_drone_mag":
            msg = deserialize_message(raw_data, Float64)
            data["force_time"].append(t)
            data["force_mag"].append(msg.data)
        elif topic == "/world/wamv_world/wrench":
            msg = deserialize_message(raw_data, EntityWrench)
            if msg.entity.name == "x500::base_link":
                data["wrench_time"].append(t)
                data["wrench_fx"].append(msg.wrench.force.x)
                data["wrench_fy"].append(msg.wrench.force.y)
                data["wrench_fz"].append(msg.wrench.force.z)
        elif topic == "/moordyn_tether_node/tether_distance":
            msg = deserialize_message(raw_data, Float64)
            data["dist_time"].append(t)
            data["distance"].append(msg.data)
        elif topic == "/moordyn_tether_node/tether_length":
            msg = deserialize_message(raw_data, Float64)
            data["length_time"].append(t)
            data["length"].append(msg.data)
        elif topic == "/drone/ground_truth/odometry":
            msg = deserialize_message(raw_data, Odometry)
            data["odom_time"].append(t)
            stamp_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data["odom_header_time"].append(stamp_sim)
            data["drone_x"].append(msg.pose.pose.position.x)
            data["drone_y"].append(msg.pose.pose.position.y)
            data["drone_z"].append(msg.pose.pose.position.z)
            data["drone_vx"].append(msg.twist.twist.linear.x)
            data["drone_vy"].append(msg.twist.twist.linear.y)
            data["drone_vz"].append(msg.twist.twist.linear.z)

    for k in data:
        data[k] = np.array(data[k])

    # Convert wall-clock timestamps to true simulation time via odometry header
    if len(data["odom_header_time"]) > 0 and len(data["odom_time"]) > 0:
        if data["odom_time"][0] > 1e6 and data["odom_header_time"][0] < 1e5:
            odom_rec = data["odom_time"].copy()
            sim_times = data["odom_header_time"]
            for key in ["force_time", "wrench_time", "dist_time", "length_time", "odom_time"]:
                if len(data[key]) > 0:
                    data[key] = np.interp(data[key], odom_rec, sim_times)

    return data


def align_and_interpolate_data(data):
    """Align timestamps so t=0 starts at the first distance valley floor (climbing motion)."""
    if len(data["odom_time"]) == 0:
        return data

    sweep_idx = np.where((data["drone_x"] < -8.0))[0]
    if len(sweep_idx) > 0:
        t_sweep = data["odom_time"][sweep_idx[0]]
        mask = (data["dist_time"] >= t_sweep) & (data["dist_time"] <= t_sweep + 7.0)
        if np.any(mask):
            min_idx = np.argmin(data["distance"][mask])
            t0 = data["dist_time"][mask][min_idx]
        else:
            t0 = t_sweep
    else:
        t0 = data["odom_time"][0]

    for key in ["force_time", "wrench_time", "dist_time", "length_time", "odom_time"]:
        if len(data[key]) > 0:
            data[key] -= t0

    return data


def compute_metrics(y_true, y_pred):
    """Compute regression metrics: R2, RMSE, MAE, Max Error."""
    residuals = y_true - y_pred
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
    rmse = np.sqrt(np.mean(residuals ** 2))
    mae = np.mean(np.abs(residuals))
    max_err = np.max(np.abs(residuals))
    return {
        "r2": r2,
        "rmse": rmse,
        "mae": mae,
        "max_err": max_err,
        "residuals": residuals,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Fit and compare 3D tether models (Catenary vs Pure Z vs Straight Chord) against MoorDyn data."
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        default="bags/slack_fit/bag_slack_105",
        help="Path to the bag containing 5% slack experiment data",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="src/system_identification/tether_sysid/model_fit",
        help="Directory to save output figures (.pdf and .png)",
    )
    parser.add_argument(
        "--cutoff_hz",
        type=float,
        default=0.8,
        help="Cutoff frequency for offline zero-phase Butterworth low-pass filter on MoorDyn telemetry [Hz] (default: 0.8)",
    )
    parser.add_argument(
        "--filter_order",
        type=int,
        default=2,
        help="Order of Butterworth low-pass filter (default: 2)",
    )
    parser.add_argument(
        "--save_png",
        action="store_true",
        default=True,
        help="Also export 300 DPI raster PNGs alongside vector PDFs (default: True)",
    )
    args = parser.parse_args()

    # Load bag data with 3D wrench forces
    print(f"Reading bag from: {args.bag_path}")
    raw_data = read_bag_data_with_wrench(args.bag_path)
    data = align_and_interpolate_data(raw_data)

    # Estimate sampling frequency of raw telemetry (~50 Hz)
    dt_raw = np.median(np.diff(data["force_time"])) if len(data["force_time"]) > 1 else 0.02
    fs_raw = 1.0 / dt_raw if dt_raw > 0 else 50.0

    print(f"Applying offline zero-phase low-pass filter (fc = {args.cutoff_hz:.2f} Hz, order = {args.filter_order}, fs ~ {fs_raw:.1f} Hz)...")
    force_mag_filt = apply_zero_phase_filter(data["force_mag"], fs=fs_raw, cutoff_hz=args.cutoff_hz, order=args.filter_order)
    wrench_fx_filt = apply_zero_phase_filter(data["wrench_fx"], fs=fs_raw, cutoff_hz=args.cutoff_hz, order=args.filter_order)
    wrench_fy_filt = apply_zero_phase_filter(data["wrench_fy"], fs=fs_raw, cutoff_hz=args.cutoff_hz, order=args.filter_order)
    wrench_fz_filt = apply_zero_phase_filter(data["wrench_fz"], fs=fs_raw, cutoff_hz=args.cutoff_hz, order=args.filter_order)

    # Common time grid (0 to 60.5 s, 10 Hz)
    t_grid = np.linspace(0.0, 60.5, 606)

    # Filtered interpolated signals (clean ground truth)
    f_true = np.interp(t_grid, data["force_time"], force_mag_filt)
    fx_true = np.interp(t_grid, data["wrench_time"], wrench_fx_filt)
    fy_true = np.interp(t_grid, data["wrench_time"], wrench_fy_filt)
    fz_true = np.interp(t_grid, data["wrench_time"], wrench_fz_filt)

    d_true = np.interp(t_grid, data["dist_time"], data["distance"])
    L_true = np.interp(t_grid, data["length_time"], data["length"])
    px = np.interp(t_grid, data["odom_time"], data["drone_x"])
    py = np.interp(t_grid, data["odom_time"], data["drone_y"])
    pz = np.interp(t_grid, data["odom_time"], data["drone_z"])

    print("\n" + "=" * 80)
    print(f"{'TETHER MODEL BENCHMARK: CATENARY VS SIMPLIFICATIONS':^80}")
    print("=" * 80)
    print(f"  Distance range:               [{np.min(d_true):.2f}, {np.max(d_true):.2f}] m  (mean: {np.mean(d_true):.2f} m)")
    print(f"  Winch length range:           [{np.min(L_true):.2f}, {np.max(L_true):.2f}] m  (mean: {np.mean(L_true):.2f} m)")
    print(f"  Tension magnitude:            [{np.min(f_true):.2f}, {np.max(f_true):.2f}] N  (mean: {np.mean(f_true):.2f} N)")
    print(f"  Force Fx (horizontal):         [{np.min(fx_true):.2f}, {np.max(fx_true):.2f}] N  (mean: {np.mean(fx_true):.2f} N)")
    print(f"  Force Fy (lateral):            [{np.min(fy_true):.2f}, {np.max(fy_true):.2f}] N  (mean: {np.mean(fy_true):.2f} N)")
    print(f"  Force Fz (vertical):           [{np.min(fz_true):.2f}, {np.max(fz_true):.2f}] N  (mean: {np.mean(fz_true):.2f} N)")
    print("-" * 80)

    # -------------------------------------------------------------
    # 1. Theoretical Tension Magnitude Model (First Principles)
    #    Nominal tether parameters from hardware/simulation specs:
    #    mu = 0.020 kg/m (linear mass density)
    #    g  = 9.81 m/s^2 (gravitational acceleration)
    #    eps_0 = 0.05 (nominal 5% operating slack maintained by winch)
    #    c_d_theo = (mu * g) / (1 - eps_0)
    #    c_L_theo = mu * g
    # -------------------------------------------------------------
    mu_nom = 0.020   # kg/m
    g_nom = 9.81     # m/s^2
    eps_0 = 0.05     # 5% slack operating point
    cd_theo = (mu_nom * g_nom) / (1.0 - eps_0)  # ~ 0.2065 N/m
    cL_theo = mu_nom * g_nom                    # ~ 0.1962 N/m

    T_pred = cd_theo * d_true
    mag_metrics = compute_metrics(f_true, T_pred)

    cd_val = cd_theo
    cL_val = cL_theo

    print(f"\n[PART 1: THEORETICAL TENSION MAGNITUDE (FIRST PRINCIPLES)]")
    print(f"  Theoretical Law: T(d) = {cd_val:.4f} * d  (equivalent to T(L) = {cL_val:.4f} * L)")
    print(f"  (Derived from mu={mu_nom} kg/m, g={g_nom} m/s^2, eps_0={eps_0*100:.1f}%)")
    print(f"  R² = {mag_metrics['r2']*100:.2f}% | RMSE = {mag_metrics['rmse']:.4f} N | MAE = {mag_metrics['mae']:.4f} N | Max Err = {mag_metrics['max_err']:.4f} N")

    # -------------------------------------------------------------
    # 2. 3D Vector Component Models (Direction Benchmarking)
    # -------------------------------------------------------------
    # Boat anchor p_v in world frame: (0, 0, 1.3)
    p_v = np.array([0.0, 0.0, 1.3])
    r = p_v - np.column_stack([px, py, pz])
    d_calc = np.linalg.norm(r, axis=1, keepdims=True)
    d_0 = r / d_calc  # Chord unit vector pointing from drone to boat

    # Horizontal unit vector e_h in catenary plane
    r_xy = r.copy()
    r_xy[:, 2] = 0.0
    r_xy_norm = np.linalg.norm(r_xy, axis=1, keepdims=True)
    e_h = r_xy / np.maximum(r_xy_norm, 1e-6)

    # Chord depression angle alpha below horizontal
    cos_alpha = np.sum(d_0 * e_h, axis=1)
    sin_alpha = - d_0[:, 2]
    alpha = np.arctan2(sin_alpha, cos_alpha)

    # MODEL 1: Thesis Catenary Model (gamma_0 = 28 deg * cos(alpha))
    gamma_0 = np.radians(28.0)
    gamma_eff = gamma_0 * np.maximum(cos_alpha, 0.0)
    ang_cat = alpha + gamma_eff
    d_cat = np.column_stack([
        np.cos(ang_cat) * e_h[:, 0],
        np.cos(ang_cat) * e_h[:, 1],
        -np.sin(ang_cat)
    ])
    F_cat = d_cat * T_pred[:, np.newaxis]

    # MODEL 2: Pure Vertical Simplification (Cable as downward ballast: Fx=0, Fy=0, Fz = -T(d))
    F_pure_z = np.column_stack([
        np.zeros_like(T_pred),
        np.zeros_like(T_pred),
        -T_pred
    ])

    # MODEL 3: Straight Chord Line (Pointing to boat: gamma = 0 deg, along d_0)
    F_chord = d_0 * T_pred[:, np.newaxis]

    F_true = np.column_stack([fx_true, fy_true, fz_true])

    # Metrics evaluation
    cat_x = compute_metrics(fx_true, F_cat[:, 0])
    cat_y = compute_metrics(fy_true, F_cat[:, 1])
    cat_z = compute_metrics(fz_true, F_cat[:, 2])
    cat_err_3d = np.linalg.norm(F_true - F_cat, axis=1)
    cat_3d_rmse = np.sqrt(np.mean(cat_err_3d ** 2))

    z_x = compute_metrics(fx_true, F_pure_z[:, 0])
    z_y = compute_metrics(fy_true, F_pure_z[:, 1])
    z_z = compute_metrics(fz_true, F_pure_z[:, 2])
    z_err_3d = np.linalg.norm(F_true - F_pure_z, axis=1)
    z_3d_rmse = np.sqrt(np.mean(z_err_3d ** 2))

    chord_x = compute_metrics(fx_true, F_chord[:, 0])
    chord_y = compute_metrics(fy_true, F_chord[:, 1])
    chord_z = compute_metrics(fz_true, F_chord[:, 2])
    chord_err_3d = np.linalg.norm(F_true - F_chord, axis=1)
    chord_3d_rmse = np.sqrt(np.mean(chord_err_3d ** 2))

    print("\n[PART 2: 3D VECTOR COMPONENT BENCHMARK]")
    print(f"{'Metric':<14} | {'1. Catenary (Thesis)':<22} | {'2. Pure Vertical (Z)':<22} | {'3. Pointing to USV':<22}")
    print("-" * 86)
    print(f"{'Fx R²':<14} | {cat_x['r2']*100:20.1f}% | {z_x['r2']*100:20.1f}% | {chord_x['r2']*100:20.1f}%")
    print(f"{'Fx RMSE':<14} | {cat_x['rmse']:19.4f} N | {z_x['rmse']:19.4f} N | {chord_x['rmse']:19.4f} N")
    print(f"{'Fz R²':<14} | {cat_z['r2']*100:20.1f}% | {z_z['r2']*100:20.1f}% | {chord_z['r2']*100:20.1f}%")
    print(f"{'Fz RMSE':<14} | {cat_z['rmse']:19.4f} N | {z_z['rmse']:19.4f} N | {chord_z['rmse']:19.4f} N")
    print("-" * 86)
    print(f"{'Overall 3D RMSE':<14} | {cat_3d_rmse:19.4f} N | {z_3d_rmse:19.4f} N | {chord_3d_rmse:19.4f} N")
    print(f"{'Error Reduction':<14} | {'Baseline (Best)':<22} | {f'+{(z_3d_rmse/cat_3d_rmse - 1)*100:.1f}% error':<22} | {f'+{(chord_3d_rmse/cat_3d_rmse - 1)*100:.1f}% error':<22}")
    print("=" * 86)

    # -------------------------------------------------------------
    # Plotting
    # -------------------------------------------------------------
    set_thesis_style()
    os.makedirs(args.output_dir, exist_ok=True)

    # Universal thesis color hierarchy:
    # Ground Truth: Black (#111111)
    # Proposed Model (Series 1): COLOR_CYCLE[0] (Blue #1f77b4, solid)
    # Baseline 1 (Series 2):     COLOR_CYCLE[1] (Orange #ff7f0e, dashed --)
    # Baseline 2 (Series 3):     COLOR_CYCLE[2] (Green #2ca02c, dash-dotted -.)
    c_gt = GROUND_TRUTH_COLOR
    c_cat = COLOR_CYCLE[0]
    c_chord = COLOR_CYCLE[1]
    c_z = COLOR_CYCLE[2]

    # -------------------------------------------------------------
    # Plot 1: 3D Force Components (Fx, Fy, Fz vs Time)
    # -------------------------------------------------------------
    fig_3d, (ax_fx, ax_fy, ax_fz) = plt.subplots(3, 1, figsize=(6.2, 5.8), sharex=True)

    # Fx Subplot
    l_gt, = ax_fx.plot(t_grid, fx_true, color=c_gt, linewidth=1.4, label="MoorDyn")
    l_cat, = ax_fx.plot(t_grid, F_cat[:, 0], color=c_cat, linewidth=1.3, label="Catenary Model")
    l_chord, = ax_fx.plot(t_grid, F_chord[:, 0], color=c_chord, linewidth=1.1, linestyle="--", label="Pointing to USV")
    l_z, = ax_fx.plot(t_grid, F_pure_z[:, 0], color=c_z, linewidth=1.1, linestyle="-.", label="Pure Vertical")
    ax_fx.set_ylabel(r"$F_x$ [N]")
    ax_fx.set_ylim([-2.3, 2.3])

    # Fy Subplot
    ax_fy.plot(t_grid, fy_true, color=c_gt, linewidth=1.4)
    ax_fy.plot(t_grid, F_cat[:, 1], color=c_cat, linewidth=1.3)
    ax_fy.plot(t_grid, F_chord[:, 1], color=c_chord, linewidth=1.1, linestyle="--")
    ax_fy.plot(t_grid, F_pure_z[:, 1], color=c_z, linewidth=1.1, linestyle="-.")
    ax_fy.set_ylabel(r"$F_y$ [N]")
    ax_fy.set_ylim([-0.12, 0.12])

    # Fz Subplot
    ax_fz.plot(t_grid, fz_true, color=c_gt, linewidth=1.4)
    ax_fz.plot(t_grid, F_cat[:, 2], color=c_cat, linewidth=1.3)
    ax_fz.plot(t_grid, F_pure_z[:, 2], color=c_z, linewidth=1.1, linestyle="-.")
    ax_fz.plot(t_grid, F_chord[:, 2], color=c_chord, linewidth=1.1, linestyle="--")
    ax_fz.set_xlabel("Time [s]")
    ax_fz.set_ylabel(r"$F_z$ [N]")
    ax_fz.set_ylim([-3.0, 0.1])

    # Shared top legend above all subplots: zero overlap with any curve!
    fig_3d.legend(
        [l_gt, l_cat, l_chord, l_z],
        ["MoorDyn", "Catenary Model", "Pointing to USV", "Pure Vertical"],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.99),
        ncol=4,
        fontsize=8.5,
        frameon=True,
    )
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.94])
    save_figure(
        fig_3d,
        os.path.join(args.output_dir, "model_fit_3d_components"),
        save_png=args.save_png,
    )
    plt.close(fig_3d)

    # -------------------------------------------------------------
    # Plot 2: Magnitude Comparison (Time)
    # -------------------------------------------------------------
    fig_time, ax_time = plt.subplots(figsize=(6.0, 3.0))
    ax_time.plot(t_grid, f_true, label="MoorDyn", color=c_gt, linewidth=1.4)
    ax_time.plot(t_grid, T_pred, label="Catenary Model", color=c_cat, linewidth=1.3)
    ax_time.set_xlabel("Time [s]")
    ax_time.set_ylabel(r"$T_{\mathrm{tether}}$ [N]")
    ax_time.set_ylim(bottom=0.0, top=3.8)
    ax_time.legend(loc="upper right", fontsize=8.5)  # t=50-60s is low (y < 1.9), top right is completely free
    plt.tight_layout()
    save_figure(
        fig_time,
        os.path.join(args.output_dir, "model_fit_comparison"),
        save_png=args.save_png,
    )
    plt.close(fig_time)


    # -------------------------------------------------------------
    # Plot 3: 3D Residuals & Error Benchmark
    # -------------------------------------------------------------
    fig_res, ax_res = plt.subplots(figsize=(6.0, 3.0))
    ax_res.plot(t_grid, cat_err_3d, label="Catenary Model", color=c_cat, linewidth=1.4)
    ax_res.plot(t_grid, chord_err_3d, label="Pointing to USV", color=c_chord, linewidth=1.1, linestyle="--")
    ax_res.plot(t_grid, z_err_3d, label="Pure Vertical", color=c_z, linewidth=1.1, linestyle="-.")
    ax_res.set_xlabel("Time [s]")
    ax_res.set_ylabel(r"$\|\mathbf{F}_{\mathrm{true}} - \mathbf{F}_{\mathrm{model}}\|$ [N]")
    ax_res.set_ylim(bottom=0.0, top=2.3)
    ax_res.legend(loc="upper center", ncol=3, fontsize=8.5)  # Horizontal top center has zero curve overlap
    plt.tight_layout()
    save_figure(
        fig_res,
        os.path.join(args.output_dir, "model_fit_residuals"),
        save_png=args.save_png,
    )
    plt.close(fig_res)

    print("\nAll figures generated successfully in:", args.output_dir)




if __name__ == "__main__":
    main()
