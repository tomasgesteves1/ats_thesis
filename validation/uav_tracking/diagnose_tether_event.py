#!/usr/bin/env python3
"""
Diagnoses the largest tether tension event of a tracking bag: forces at both ends of the cable, geometry (d, L, slack
ratio) and the angular velocity of the boat odometry that feeds the cable-end velocity.
Usage: python3 diagnose_tether_event.py [--bag PATH | --scenario reachable|infeasible] [--window SECONDS]
"""
import argparse, os, sys
import numpy as np
import matplotlib.pyplot as plt
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__)); sys.path.append(os.path.dirname(CURRENT_DIR))
from thesis_style import set_thesis_style, save_figure, COLOR_CYCLE
from tracking_common import latest_bag, read_bag, odometry_arrays, bag_to_sim_time

p = argparse.ArgumentParser(); p.add_argument("--bag"); p.add_argument("--scenario", default="reachable")
p.add_argument("--window", type=float, default=1.0); a = p.parse_args()
bag = a.bag or latest_bag(f"tracking_{a.scenario}_*")
raw = read_bag(bag, ["/drone/ground_truth/odometry", "/boat/ground_truth/odometry", "/tether_force_drone_mag",
                     "/tether_force_boat_mag", "/moordyn_tether_node/tether_length", "/moordyn_tether_node/tether_distance"])
td, od = raw["/drone/ground_truth/odometry"]; t_d, *_ = odometry_arrays(td, od)
to = lambda t: bag_to_sim_time(t, td, t_d)
g = lambda k: (to(raw[k][0]), np.array([m.data for m in raw[k][1]]))
tF, F = g("/tether_force_drone_mag"); tB, FB = g("/tether_force_boat_mag"); tL, L = g("/moordyn_tether_node/tether_length"); tD, D = g("/moordyn_tether_node/tether_distance")
ob = raw["/boat/ground_truth/odometry"][1]
t_o = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in ob])
w = np.array([np.linalg.norm([m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z]) for m in ob])
tk = tB[np.argmax(FB)]
sel = lambda t: (t > tk - a.window) & (t < tk + a.window)
d_i = np.interp(tL, tD, D); eps = (L - d_i) / L
set_thesis_style(); c = COLOR_CYCLE
fig, axs = plt.subplots(4, 1, figsize=(6.0, 6.6), sharex=True)
axs[0].semilogy(t_o[sel(t_o)] - tk, np.maximum(w[sel(t_o)], 1e-3), color=c[0]); axs[0].set_ylabel("Boat $\\|\\omega\\|$ (rad/s)")
axs[1].plot(tB[sel(tB)] - tk, FB[sel(tB)], color=c[1], label="Boat end"); axs[1].plot(tF[sel(tF)] - tk, F[sel(tF)], color=c[0], label="UAV end")
axs[1].set_ylabel("Tension (N)"); axs[1].legend()
axs[2].plot(tL[sel(tL)] - tk, d_i[sel(tL)], color=c[0], label="$d$"); axs[2].plot(tL[sel(tL)] - tk, L[sel(tL)], color=c[2], label="$L$")
axs[2].set_ylabel("Length (m)"); axs[2].legend(ncol=2)
axs[3].plot(tL[sel(tL)] - tk, eps[sel(tL)], color=c[0]); axs[3].set_ylabel("$\\varepsilon$"); axs[3].set_xlabel("Time from the force peak at the boat end (s)")
save_figure(fig, os.path.join(CURRENT_DIR, "plots", a.scenario, "uav_tether_event"))
print("largest boat-end tension %.1f N at t=%.1f s; max boat |omega| in window %.1f rad/s" % (FB.max(), tk, w[sel(t_o)].max()))
