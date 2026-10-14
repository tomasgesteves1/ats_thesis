#!/usr/bin/env python3
"""
Evaluates the tracking of the USV NMPC and the respect of its constraints (Table 4.11) from a bag recorded with
validation/record_tracking.sh. Figures are saved to usv_tracking/plots/<scenario>/ (and optionally to the thesis).

Usage: python3 evaluate_usv_tracking.py [--bag PATH | --scenario reachable|infeasible] [--skip SECONDS]
"""
import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))
from thesis_style import set_thesis_style, save_figure, COLOR_CYCLE, SIMULATION_COLOR
from tracking_common import (USV_LIMITS as L, UAV_LIMITS, latest_bag, read_bag, odometry_arrays, euler_zyx,
                             bag_to_sim_time, resample, wrap_angle, rms, violation, tidy_limits)

TOPICS = ["/boat/ground_truth/odometry", "/drone/ground_truth/odometry", "/usv/reference_path",
          "/boat/thrusters/left/thrust", "/boat/thrusters/right/thrust",
          "/boat/thrusters/left/pos", "/boat/thrusters/right/pos", "/usv_solve_time_ms"]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", default=None)
    p.add_argument("--scenario", default="reachable", choices=["reachable", "infeasible", "aggressive", "tether_edge"])
    p.add_argument("--show-limits", action="store_true", help="draw every constraint line even when far from the data")
    p.add_argument("--skip", type=float, default=15.0, help="seconds skipped after the UAV reaches its flight altitude")
    p.add_argument("--out", default=None, help="output directory (default: usv_tracking/plots/<scenario>)")
    return p.parse_args()


def load(bag):
    raw = read_bag(bag, TOPICS)
    tb, ob = raw["/boat/ground_truth/odometry"]
    t_boat, p_boat, q_boat, v_boat = odometry_arrays(tb, ob)
    td, od = raw["/drone/ground_truth/odometry"]
    t_drone, p_drone, _, _ = odometry_arrays(td, od)
    to_sim = lambda t: bag_to_sim_time(t, tb, t_boat)

    # start of the analysis: UAV at flight altitude + skip
    t_fly = t_drone[np.argmax(p_drone[:, 2] >= 6.5)] if (p_drone[:, 2] >= 6.5).any() else t_boat[0]
    return raw, to_sim, (t_boat, p_boat, q_boat, v_boat), (t_drone, p_drone), t_fly


def main():
    a = parse_args()
    bag = a.bag or latest_bag(f"tracking_{a.scenario}_*")
    scenario = a.scenario if a.bag is None else os.path.basename(bag)
    out = a.out or os.path.join(CURRENT_DIR, "plots", scenario)
    print(f"[INFO] bag: {bag}")
    raw, to_sim, (t_b, p_b, q_b, v_b), (t_d, p_d), t_fly = load(bag)
    set_thesis_style()
    save = lambda fig, name: (tidy_limits(fig, a.show_limits), save_figure(fig, os.path.join(out, name)))

    t0 = t_fly + a.skip
    t_end = t_b[-1]
    ts = L["Ts"]
    t = np.arange(t0, t_end, ts)
    tt = t - t0
    pos = resample(t_b, p_b[:, :2], t)
    yaw_b, _, _ = euler_zyx(q_b)
    yaw = resample(t_b, np.unwrap(yaw_b), t)   # unwrapped before resampling (no interpolation across +-pi)

    # reference (first pose of the reference path = current setpoint)
    tr, mr = raw["/usv/reference_path"]
    tr = to_sim(tr)
    ref = np.array([[m.poses[0].pose.position.x, m.poses[0].pose.position.y] for m in mr])
    ref_yaw = np.unwrap(np.array([2 * np.arctan2(m.poses[0].pose.orientation.z, m.poses[0].pose.orientation.w) for m in mr]))
    ref_i, ref_yaw_i = resample(tr, ref, t), resample(tr, ref_yaw, t)
    e_pos = np.linalg.norm(pos - ref_i, axis=1)
    e_yaw = np.degrees(wrap_angle(yaw - ref_yaw_i))

    # thruster commands
    def series(topic):
        tc, mc = raw[topic]
        return tc, np.array([m.data for m in mc])
    tcmd = {}
    cmd = {}
    for k, topic in {"TL": "/boat/thrusters/left/thrust", "TR": "/boat/thrusters/right/thrust",
                     "aL": "/boat/thrusters/left/pos", "aR": "/boat/thrusters/right/pos"}.items():
        tcmd[k], cmd[k] = series(topic)
        tcmd[k] = to_sim(tcmd[k])
    TL, TR = (resample(tcmd[k], cmd[k], t) for k in ("TL", "TR"))
    aL, aR = (resample(tcmd[k], cmd[k], t) for k in ("aL", "aR"))
    # rates from the commands as published (10 Hz), on their own time base
    rates = {}
    for k in ("TL", "TR", "aL", "aR"):
        m = tcmd[k] >= t0
        # the commands are integrated over the nominal sampling time, so the rate is the increment over T_s
        # (the arrival times of the messages have jitter); irregular intervals are discarded
        dt = np.diff(tcmd[k][m]); good = (dt > 0.5 * ts) & (dt < 1.5 * ts)
        rates[k] = ((tcmd[k][m][1:] - t0)[good], (np.diff(cmd[k][m]) / ts)[good])

    # tether distance seen by the USV controller: attachment point (x, y, z_t) to the UAV
    p_dr = resample(t_d, p_d, t)
    d = np.sqrt(np.sum((p_dr[:, :2] - pos) ** 2, axis=1) + (p_dr[:, 2] - L["z_t"]) ** 2)

    # solve time
    tsol, msol = raw["/usv_solve_time_ms"]
    solve = np.array([m.data for m in msol])

    # --------------------------------------------------------------------------------------------------- metrics
    print("\n=== USV tracking (t > %.0f s after the UAV reaches altitude, %d s) ===" % (a.skip, tt[-1]))
    print("position error   : RMSE %.3f m | max %.3f m" % (rms(e_pos), e_pos.max()))
    print("heading error    : RMSE %.2f deg | max %.2f deg" % (rms(e_yaw), np.abs(e_yaw).max()))
    print("--- constraints (max violation | fraction of samples outside) ---")
    for name, val, lo, up, unit in [("T_L", TL, L["T_min"], L["T_max"], "N"), ("T_R", TR, L["T_min"], L["T_max"], "N"),
                                    ("alpha_L", np.degrees(aL), -30, 30, "deg"), ("alpha_R", np.degrees(aR), -30, 30, "deg")]:
        mv, fr = violation(val, lo, up)
        print("%-9s in [%7.1f, %6.1f] %-3s: max viol %.3g | %.1f%%  (range seen %.1f..%.1f)" % (name, lo, up, unit, mv, 100 * fr, val.min(), val.max()))
    for k, lim, scale, unit in [("TL", L["T_dot_max"], 1, "N/s"), ("TR", L["T_dot_max"], 1, "N/s"),
                                ("aL", L["alpha_dot_max"], 180 / np.pi, "deg/s"), ("aR", L["alpha_dot_max"], 180 / np.pi, "deg/s")]:
        r = np.abs(rates[k][1]) * scale
        print("|d%s/dt| max %.1f %s (limit %.1f) -> %s" % (k, r.max(), unit, lim * scale, "OK" if r.max() <= lim * scale * 1.001 else "VIOLATED"))
    mv, fr = violation(d, None, L["d_max"])
    print("tether distance  : max %.2f m (limit %.1f) | violation %.3f m | %.1f%% of samples above" % (d.max(), L["d_max"], mv, 100 * fr))
    print("solve time       : mean %.2f ms | p95 %.2f | max %.2f (sampling time %.0f ms)" % (solve.mean(), np.percentile(solve, 95), solve.max(), 1000 * ts))

    # ----------------------------------------------------------------------------------------------------- plots
    c = COLOR_CYCLE
    fig, ax = plt.subplots(figsize=(4.0, 4.0))
    ax.plot(ref_i[:, 0], ref_i[:, 1], color=SIMULATION_COLOR, lw=1.8, label="Reference")
    ax.plot(pos[:, 0], pos[:, 1], color=c[0], lw=1.0, label="USV")
    ax.set_aspect("equal"); ax.set_xlabel("$x$ (m)"); ax.set_ylabel("$y$ (m)"); ax.legend()
    save(fig, "usv_tracking_xy")

    fig, axs = plt.subplots(2, 1, figsize=(6.0, 3.6), sharex=True)
    axs[0].plot(tt, e_pos, color=c[0]); axs[0].set_ylabel("Position error (m)")
    axs[1].plot(tt, e_yaw, color=c[1]); axs[1].set_ylabel("Heading error (deg)"); axs[1].set_xlabel("Time (s)")
    save(fig, "usv_tracking_error")

    fig, axs = plt.subplots(3, 1, figsize=(6.0, 5.2), sharex=True)
    for ax_, i, nm in zip(axs[:2], (0, 1), ("x", "y")):
        ax_.plot(tt, ref_i[:, i], color=SIMULATION_COLOR, lw=1.8, label="Reference"); ax_.plot(tt, pos[:, i], color=c[0], lw=1.0, label="USV")
        ax_.set_ylabel(f"${nm}$ (m)")
    axs[0].legend(ncol=2)
    axs[2].plot(tt, np.degrees(ref_yaw_i), color=SIMULATION_COLOR, lw=1.8); axs[2].plot(tt, np.degrees(ref_yaw_i + wrap_angle(yaw - ref_yaw_i)), color=c[0], lw=1.0)
    axs[2].set_ylabel("$\\psi$ (deg)"); axs[2].set_xlabel("Time (s)")
    save(fig, "usv_tracking_signals")

    fig, axs = plt.subplots(2, 1, figsize=(6.0, 3.8), sharex=True)
    axs[0].plot(tt, TL, color=c[0], label="$T_L$"); axs[0].plot(tt, TR, color=c[1], label="$T_R$")
    axs[0].axhline(L["T_max"], color=c[3], ls="--", lw=0.9, label="Limits"); axs[0].axhline(L["T_min"], color=c[3], ls="--", lw=0.9)
    axs[0].set_ylabel("Thrust (N)"); axs[0].legend(ncol=3)
    axs[1].plot(tt, np.degrees(aL), color=c[0], label="$\\alpha_L$"); axs[1].plot(tt, np.degrees(aR), color=c[1], label="$\\alpha_R$")
    axs[1].axhline(30, color=c[3], ls="--", lw=0.9); axs[1].axhline(-30, color=c[3], ls="--", lw=0.9)
    axs[1].set_ylabel("Azimuth (deg)"); axs[1].set_xlabel("Time (s)"); axs[1].legend(ncol=2)
    save(fig, "usv_constraints_thrusters")

    fig, axs = plt.subplots(2, 1, figsize=(6.0, 3.8), sharex=True)
    axs[0].plot(rates["TL"][0], rates["TL"][1], color=c[0], label="$\\dot T_L$"); axs[0].plot(rates["TR"][0], rates["TR"][1], color=c[1], label="$\\dot T_R$")
    axs[0].axhline(L["T_dot_max"], color=c[3], ls="--", lw=0.9); axs[0].axhline(-L["T_dot_max"], color=c[3], ls="--", lw=0.9)
    axs[0].set_ylabel("Thrust rate (N/s)"); axs[0].legend(ncol=2)
    axs[1].plot(rates["aL"][0], np.degrees(rates["aL"][1]), color=c[0], label="$\\dot\\alpha_L$")
    axs[1].plot(rates["aR"][0], np.degrees(rates["aR"][1]), color=c[1], label="$\\dot\\alpha_R$")
    axs[1].axhline(10, color=c[3], ls="--", lw=0.9); axs[1].axhline(-10, color=c[3], ls="--", lw=0.9)
    axs[1].set_ylabel("Azimuth rate (deg/s)"); axs[1].set_xlabel("Time (s)"); axs[1].legend(ncol=2)
    save(fig, "usv_constraints_rates")

    fig, ax = plt.subplots(figsize=(6.0, 2.2))
    ax.plot(tt, d, color=c[0], label="$d$"); ax.axhline(L["d_max"], color=c[3], ls="--", lw=0.9, label="$d_{max}$")
    ax.set_ylabel("Tether distance (m)"); ax.set_xlabel("Time (s)"); ax.legend(ncol=2)
    save(fig, "usv_constraint_tether")

    fig, ax = plt.subplots(figsize=(6.0, 2.2))
    ax.plot(np.arange(len(solve)) * ts, solve, color=c[0], lw=0.6)
    ax.axhline(1000 * ts, color=c[3], ls="--", lw=0.9, label="Sampling time")
    ax.set_ylabel("Solve time (ms)"); ax.set_xlabel("Time (s)"); ax.legend()
    save(fig, "usv_solve_time")
    print("[INFO] figures saved to", out)


if __name__ == "__main__":
    main()
