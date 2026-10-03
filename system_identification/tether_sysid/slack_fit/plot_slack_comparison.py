#!/usr/bin/env python3
"""
Tether Slack Factor Analysis & Comparison Script
MSc Thesis Standardized Plotting (UAV-USV Cooperative Project)

Analyzes tether tension magnitude (T_tether) on UAV and geometric distance (d)
versus unstretched winch length (L_tether) across multiple slack values (e.g. 2%, 5%, 10%).
Exports standalone, publication-quality vector (.pdf) and raster (.png) figures
without in-plot titles (designed for native LaTeX \\subcaption labels).
"""

import os
import sys
import argparse
import glob
import numpy as np
import matplotlib.pyplot as plt

# Add system_identification root to path for shared thesis styling
sys_id_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if sys_id_dir not in sys.path:
    sys.path.append(sys_id_dir)

from thesis_style import set_thesis_style, save_figure, THESIS_COLORS

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64
    from nav_msgs.msg import Odometry
except ImportError as e:
    print(f"Error importing ROS 2 libraries: {e}")
    print("Please ensure your ROS 2 environment is sourced (source install/setup.bash).")
    sys.exit(1)


def read_bag_data(bag_path):
    """Read tether and drone topics from a ROS 2 bag (MCAP or sqlite3)."""
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
        "dist_time": [],
        "distance": [],
        "length_time": [],
        "length": [],
        "odom_time": [],
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
            if "odom_header_time" not in data:
                data["odom_header_time"] = []
            data["odom_header_time"].append(stamp_sim)
            data["drone_x"].append(msg.pose.pose.position.x)
            data["drone_y"].append(msg.pose.pose.position.y)
            data["drone_z"].append(msg.pose.pose.position.z)
            data["drone_vx"].append(msg.twist.twist.linear.x)
            data["drone_vy"].append(msg.twist.twist.linear.y)
            data["drone_vz"].append(msg.twist.twist.linear.z)

    for k in data:
        data[k] = np.array(data[k])

    # If recorded using wall-clock time (epoch > 1e6), interpolate to true simulation time
    if "odom_header_time" in data and len(data["odom_header_time"]) > 0 and len(data["odom_time"]) > 0:
        if data["odom_time"][0] > 1e6 and data["odom_header_time"][0] < 1e5:
            print("  Converting wall-clock timestamps to true simulation time via odometry header...")
            odom_record_times = data["odom_time"].copy()
            sim_times = data["odom_header_time"]
            for key in ["force_time", "dist_time", "length_time", "odom_time"]:
                if len(data[key]) > 0:
                    data[key] = np.interp(data[key], odom_record_times, sim_times)

    return data


def align_and_interpolate(data, align_to_motion=True):
    """Align timestamps so t=0 starts at the first distance/tension valley (climbing upwards)."""
    if len(data["odom_time"]) == 0:
        return data

    t0 = data["odom_time"][0]

    if align_to_motion:
        # Detect start of the first experimental sweep from x ≈ -10m
        sweep_idx = np.where((data["drone_x"] < -8.0) & (data["drone_vx"] > 0.15))[0]
        if len(sweep_idx) > 0:
            t_sweep = data["odom_time"][sweep_idx[0]]
            # Find the first minimum distance (valley floor) within 7s after sweep start
            mask = (data["dist_time"] >= t_sweep) & (data["dist_time"] <= t_sweep + 7.0)
            if np.any(mask):
                min_idx = np.argmin(data["distance"][mask])
                t0 = data["dist_time"][mask][min_idx]
                print(f"  Aligned t=0 to valley floor (d_min ≈ 5.0m, ascending motion) at sim time: {t0:.2f}s")
            else:
                t0 = t_sweep
                print(f"  Aligned t=0 to sweep start at sim time: {t0:.2f}s")
        else:
            motion_idx = np.where(np.abs(data["drone_x"]) > 0.5)[0]
            if len(motion_idx) > 0:
                t0 = data["odom_time"][motion_idx[0]]
                print(f"  Aligned t=0 to horizontal motion start at sim time: {t0:.2f}s")
            else:
                print("  Warning: Motion trigger not found, aligning to first timestamp.")

    data["force_time"] -= t0
    data["dist_time"] -= t0
    data["length_time"] -= t0
    data["odom_time"] -= t0

    # Filter data: start at t = 0 (valley floor) and clamp to the return to center (≈ 60.5s)
    f_mask = (data["force_time"] >= 0.0) & (data["force_time"] <= 60.5)
    data["force_time"] = data["force_time"][f_mask]
    data["force_mag"] = data["force_mag"][f_mask]

    d_mask = (data["dist_time"] >= 0.0) & (data["dist_time"] <= 60.5)
    data["dist_time"] = data["dist_time"][d_mask]
    data["distance"] = data["distance"][d_mask]

    l_mask = (data["length_time"] >= 0.0) & (data["length_time"] <= 60.5)
    data["length_time"] = data["length_time"][l_mask]
    data["length"] = data["length"][l_mask]

    return data


def parse_slack_label(bag_name):
    """Extract formal thesis label with epsilon, slack factor, and standard palette color."""
    base = os.path.basename(os.path.normpath(bag_name))
    if "102" in base or "2" in base:
        return r"$\varepsilon = 2\%$", THESIS_COLORS["slack_2"], "1.02"
    elif "105" in base or "5" in base:
        return r"$\varepsilon = 5\%$", THESIS_COLORS["slack_5"], "1.05"
    elif "110" in base or "10" in base:
        return r"$\varepsilon = 10\%$", THESIS_COLORS["slack_10"], "1.10"
    elif "115" in base or "15" in base:
        return r"$\varepsilon = 15\%$", THESIS_COLORS["slack_15"], "1.15"
    return base, THESIS_COLORS["slack_2"], "unknown"


def main():
    parser = argparse.ArgumentParser(
        description="Plot individual thesis-ready figures for tether slack factor experiments."
    )
    parser.add_argument(
        "--bag_dir",
        type=str,
        default="bags/slack_fit",
        help="Directory containing bag folders",
    )
    parser.add_argument(
        "--bags",
        nargs="*",
        default=[],
        help="Optional list of specific bag paths to analyze",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="src/system_identification/tether_sysid/slack_fit",
        help="Directory to save output figures (.pdf and .png)",
    )
    parser.add_argument(
        "--no-align",
        action="store_true",
        help="Do not align t=0 to motion start",
    )
    args = parser.parse_args()

    # Find bags
    bag_paths = args.bags
    if not bag_paths:
        if os.path.exists(args.bag_dir):
            all_entries = sorted(glob.glob(os.path.join(args.bag_dir, "*")))
            bag_paths = [p for p in all_entries if os.path.isdir(p)]

    if not bag_paths:
        print(f"Error: No bag directories found in '{args.bag_dir}'.")
        sys.exit(1)

    print(f"Found {len(bag_paths)} bag(s) to process:")
    for b in bag_paths:
        print(f"  - {b}")

    dataset = []
    for bag_path in bag_paths:
        print(f"\nProcessing '{bag_path}'...")
        try:
            raw_data = read_bag_data(bag_path)
            aligned_data = align_and_interpolate(raw_data, align_to_motion=not args.no_align)
            label, color, slack_val = parse_slack_label(bag_path)
            dataset.append({
                "path": bag_path,
                "label": label,
                "color": color,
                "slack_val": slack_val,
                "data": aligned_data,
            })
        except Exception as e:
            print(f"  Failed to read bag '{bag_path}': {e}")

    if not dataset:
        print("No valid data loaded. Exiting.")
        sys.exit(1)

    # Print statistical summary
    print("\n" + "=" * 78)
    print(f"{'Experiment':<26} | {'Mean Tension':<12} | {'Max Tension':<12} | {'Std Tension':<12} | {'Max Dist':<9}")
    print("=" * 78)
    for item in dataset:
        f = item["data"]["force_mag"]
        d = item["data"]["distance"]
        if len(f) > 0 and len(d) > 0:
            print(
                f"{item['label']:<26} | {np.mean(f):8.2f} N   | {np.max(f):8.2f} N   | {np.std(f):8.2f} N   | {np.max(d):6.2f} m"
            )
    print("=" * 78 + "\n")

    # Apply global thesis style
    set_thesis_style()

    # Dimensions tuned for a standalone figure or LaTeX subfigure (e.g. 0.49\textwidth)
    fig_size = (4.5, 2.9)

    # -------------------------------------------------------------
    # 1. Individual Plot: Tether Tension Magnitude
    # -------------------------------------------------------------
    fig_tension, ax_tension = plt.subplots(figsize=fig_size)
    for item in dataset:
        d = item["data"]
        if len(d["force_time"]) > 0:
            ax_tension.plot(
                d["force_time"],
                d["force_mag"],
                label=item["label"],
                color=item["color"],
                linewidth=1.2,
            )
    ax_tension.set_xlabel("Time [s]")
    ax_tension.set_ylabel(r"$T_{\mathrm{tether}}$ [N]")
    ax_tension.legend(loc="upper right")
    ax_tension.set_ylim(bottom=0.0)
    plt.tight_layout()

    tension_path = os.path.join(args.output_dir, "slack_tension")
    save_figure(fig_tension, tension_path, save_png=False)
    plt.close(fig_tension)

    # -------------------------------------------------------------
    # 2. Individual Plot: Distance vs. Winch Length
    # -------------------------------------------------------------
    fig_geom, ax_geom = plt.subplots(figsize=fig_size)
    for item in dataset:
        d = item["data"]
        # Distance (dashed, same color as slack, no label)
        if len(d["dist_time"]) > 0:
            ax_geom.plot(
                d["dist_time"],
                d["distance"],
                color=item["color"],
                linestyle="--",
                linewidth=1.1,
            )
        # Winch unstretched length (solid, same color as slack, labeled with epsilon)
        if len(d["length_time"]) > 0:
            ax_geom.plot(
                d["length_time"],
                d["length"],
                label=item["label"],
                color=item["color"],
                linestyle="-",
                linewidth=1.2,
            )
    ax_geom.set_xlabel("Time [s]")
    ax_geom.set_ylabel("Length [m]")
    ax_geom.legend(loc="upper right")
    plt.tight_layout()

    geom_path = os.path.join(args.output_dir, "slack_geometry")
    save_figure(fig_geom, geom_path, save_png=False)
    plt.close(fig_geom)

    print("\nIndividual thesis figures exported successfully!")


if __name__ == "__main__":
    main()
