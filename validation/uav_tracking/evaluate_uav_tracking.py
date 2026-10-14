#!/usr/bin/env python3
"""
Evaluates the tracking of the UAV NMPC and the respect of its constraints (Table 4.12) from a bag recorded with
validation/record_tracking.sh. Figures are saved to uav_tracking/plots/<scenario>/.

Usage: python3 evaluate_uav_tracking.py [--bag PATH | --scenario reachable|infeasible] [--skip SECONDS]
"""
import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))
from thesis_style import set_thesis_style, save_figure, COLOR_CYCLE, SIMULATION_COLOR
from tracking_common import (UAV_LIMITS as L, UAV_TELEMETRY as K, latest_bag, read_bag, odometry_arrays,
                             bag_to_sim_time, resample, wrap_angle, rms, violation, tidy_limits, odometry_arrays as _oa, px4_attitude_to_enu_euler)

TOPICS = ["/mpc_states_and_inputs", "/drone/ground_truth/odometry", "/boat/ground_truth/odometry",
          "/virtual_tether_distance", "/virtual_tether_separation_speed", "/tether_force_drone_mag",
          "/moordyn_tether_node/tether_distance", "/moordyn_tether_node/tether_length", "/uav_solve_time_ms",
          "/px4_1/fmu/out/vehicle_attitude", "/px4_1/fmu/out/vehicle_attitude_setpoint"]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", default=None)
    p.add_argument("--scenario", default="reachable", choices=["reachable", "infeasible", "aggressive", "tether_edge"])
    p.add_argument("--show-limits", action="store_true", help="draw every constraint line even when far from the data")
    p.add_argument("--skip", type=float, default=15.0, help="seconds skipped after the UAV reaches its flight altitude")
    p.add_argument("--out", default=None, help="output directory (default: uav_tracking/plots/<scenario>)")
    return p.parse_args()


def main():
    a = parse_args()
    bag = a.bag or latest_bag(f"tracking_{a.scenario}_*")
    scenario = a.scenario if a.bag is None else os.path.basename(bag)
    out = a.out or os.path.join(CURRENT_DIR, "plots", scenario)
    print(f"[INFO] bag: {bag}")
    raw = read_bag(bag, TOPICS)
    td, od = raw["/drone/ground_truth/odometry"]
    t_d, p_d, _, _ = odometry_arrays(td, od)
    tb, ob = raw["/boat/ground_truth/odometry"]
    t_b, p_b, _, _ = odometry_arrays(tb, ob)
    to_sim = lambda t: bag_to_sim_time(t, td, t_d)
    set_thesis_style()
    save = lambda fig, name: (tidy_limits(fig, a.show_limits), save_figure(fig, os.path.join(out, name)))

    # telemetry (no header: bag time mapped to simulation time)
    tt_raw, mt = raw["/mpc_states_and_inputs"]
    tel = np.array([m.data for m in mt]); t_tel = to_sim(tt_raw)
    fly = tel[:, K["z"]] >= 6.5
    t_fly = t_tel[np.argmax(fly)] if fly.any() else t_tel[0]
    t0 = t_fly + a.skip
    t = np.arange(t0, t_tel[-1], L["Ts"]); tt = t - t0
    g = lambda name: resample(t_tel, tel[:, K[name]], t)
    pos = np.column_stack([g("x"), g("y"), g("z")]); ref = np.column_stack([g("ref_x"), g("ref_y"), g("ref_z")])
    vel = np.column_stack([g("vx"), g("vy"), g("vz")])
    roll, pitch, thrust = g("roll"), g("pitch"), g("thrust")
    yaw = resample(t_tel, np.unwrap(tel[:, K["yaw"]]), t)   # unwrapped before resampling (no interpolation across +-pi)
    roll_c, pitch_c = g("roll_cmd"), g("pitch_cmd")
    yaw_c = resample(t_tel, np.unwrap(tel[:, K["yaw_cmd"]]), t)
    # The commanded tilt is expressed in the frame rotated by the commanded yaw; it is re-expressed in the frame of the
    # actual yaw so that it can be compared with the measured roll and pitch
    nc = np.column_stack([np.sin(pitch_c) * np.cos(roll_c), -np.sin(roll_c), np.cos(pitch_c) * np.cos(roll_c)])
    dpsi = wrap_angle(yaw_c - yaw)
    nl = np.column_stack([np.cos(dpsi) * nc[:, 0] - np.sin(dpsi) * nc[:, 1], np.sin(dpsi) * nc[:, 0] + np.cos(dpsi) * nc[:, 1], nc[:, 2]])
    roll_c, pitch_c = np.arcsin(-nl[:, 1]), np.arctan2(nl[:, 0], nl[:, 2])
    boat = resample(t_b, p_b[:, :2], t)
    e = pos - ref; e_pos = np.linalg.norm(e, axis=1)
    bearing = np.arctan2(boat[:, 1] - pos[:, 1], boat[:, 0] - pos[:, 0])
    e_yaw = np.degrees(wrap_angle(yaw - bearing))
    v_xy = np.hypot(vel[:, 0], vel[:, 1])

    # command rates on the telemetry time base (50 Hz)
    mm = t_tel >= t0
    tr_ = t_tel[mm][1:] - t0; dtm = np.diff(t_tel[mm])
    Ts = L["Ts"]
    ok = (dtm > 0.5 * Ts) & (dtm < 1.5 * Ts)   # commands are integrated over T_s: rate = increment / T_s (arrival jitter ignored)
    rate = {"phi": np.diff(tel[mm, K["roll_cmd"]])[ok] / Ts, "theta": np.diff(tel[mm, K["pitch_cmd"]])[ok] / Ts,
            "psi": np.diff(np.unwrap(tel[mm, K["yaw_cmd"]]))[ok] / Ts}
    tr_ = tr_[ok]

    # tether quantities as seen by the controller / simulator
    def sc(topic):
        tc, mc = raw[topic]; return to_sim(tc), np.array([m.data for m in mc])
    t_vd, vd = sc("/virtual_tether_distance"); t_vs, vsep = sc("/virtual_tether_separation_speed")
    t_F, F = sc("/tether_force_drone_mag"); t_L, Lt = sc("/moordyn_tether_node/tether_length")
    d = resample(t_vd, vd, t); d_dot = resample(t_vs, vsep, t); Fm = resample(t_F, F, t); Ltether = resample(t_L, Lt, t)
    # slack ratio from the length and distance computed by MoorDyn in the same step (pairing samples of different
    # topics gives spurious spikes when the distance changes quickly)
    t_Ld, Ld = sc("/moordyn_tether_node/tether_distance"); n_ = min(len(Ld), len(Lt))
    eps = resample(t_L[:n_], (Lt[:n_] - Ld[:n_]) / Lt[:n_], t)
    tsol, msol = raw["/uav_solve_time_ms"]; solve = np.array([m.data for m in msol])

    # --------------------------------------------------------------------------------------------------- metrics
    print("\n=== UAV tracking (t > %.0f s after the UAV reaches altitude, %d s) ===" % (a.skip, tt[-1]))
    print("position error   : RMSE %.3f m | max %.3f m | per axis RMSE x %.3f y %.3f z %.3f" % (rms(e_pos), e_pos.max(), rms(e[:, 0]), rms(e[:, 1]), rms(e[:, 2])))
    print("yaw - bearing    : mean %.2f deg | RMSE %.2f | max |.| %.2f" % (e_yaw.mean(), rms(e_yaw), np.abs(e_yaw).max()))
    print("--- constraints (max violation | fraction of samples outside) ---")
    checks = [("z >= z_min", pos[:, 2], L["z_min"], None, "m"), ("|v_xy| <= v_xy_max", v_xy, None, L["v_xy_max"], "m/s"),
              ("|v_z| <= v_z_max", vel[:, 2], -L["v_z_max"], L["v_z_max"], "m/s"),
              ("|roll| <= phi_max", roll, -L["phi_max"], L["phi_max"], "rad"), ("|pitch| <= theta_max", pitch, -L["theta_max"], L["theta_max"], "rad"),
              ("T in [T_min,T_max]", thrust, L["T_min"], L["T_max"], "N"), ("d <= d_max", d, None, L["d_max"], "m"),
              ("d_dot <= d_dot_max", d_dot, None, L["d_dot_max"], "m/s")]
    for name, val, lo, up, unit in checks:
        mv, fr = violation(val, lo, up)
        print("%-22s: max viol %.3g %s | %.1f%%  (range seen %.3g..%.3g)" % (name, mv, unit, 100 * fr, val.min(), val.max()))
    for k, lim in (("phi", L["phi_dot_max"]), ("theta", L["theta_dot_max"]), ("psi", L["psi_dot_max"])):
        print("|d%s_cmd/dt| max %.3f rad/s (limit %.2f) -> %s" % (k, np.abs(rate[k]).max(), lim, "OK" if np.abs(rate[k]).max() <= lim * 1.001 else "VIOLATED"))
    print("tether           : tension mean %.2f N, max %.2f N | length %.2f..%.2f m | slack ratio %.4f..%.4f" % (Fm.mean(), Fm.max(), Ltether.min(), Ltether.max(), eps.min(), eps.max()))
    print("solve time       : mean %.2f ms | p95 %.2f | max %.2f (sampling time %.0f ms)" % (solve.mean(), np.percentile(solve, 95), solve.max(), 1000 * L["Ts"]))

    # ----------------------------------------------------------------------------------------------------- plots
    c = COLOR_CYCLE
    fig, ax = plt.subplots(figsize=(4.0, 4.0))
    ax.plot(ref[:, 0], ref[:, 1], color=SIMULATION_COLOR, lw=1.8, label="Reference")
    ax.plot(pos[:, 0], pos[:, 1], color=c[0], lw=1.0, label="UAV")
    ax.plot(boat[:, 0], boat[:, 1], color=c[1], lw=0.9, label="USV")
    ax.set_aspect("equal"); ax.set_xlabel("$x$ (m)"); ax.set_ylabel("$y$ (m)"); ax.legend()
    save(fig, "uav_tracking_xy")

    fig, axs = plt.subplots(2, 1, figsize=(6.0, 3.6), sharex=True)
    axs[0].plot(tt, e_pos, color=c[0], label="$\\|\\mathbf{p}-\\mathbf{p}_{ref}\\|$")
    for i, nm in enumerate("xyz"): axs[0].plot(tt, e[:, i], lw=0.7, color=c[i + 1], label=f"${nm}$")
    axs[0].set_ylabel("Position error (m)"); axs[0].legend(ncol=4)
    axs[1].plot(tt, e_yaw, color=c[0]); axs[1].set_ylabel("$\\psi-$bearing (deg)"); axs[1].set_xlabel("Time (s)")
    save(fig, "uav_tracking_error")

    fig, axs = plt.subplots(3, 1, figsize=(6.0, 5.2), sharex=True)
    for i, nm in enumerate("xyz"):
        axs[i].plot(tt, ref[:, i], color=SIMULATION_COLOR, lw=1.8, label="Reference"); axs[i].plot(tt, pos[:, i], color=c[0], lw=1.0, label="UAV")
        axs[i].set_ylabel(f"${nm}$ (m)")
    axs[0].legend(ncol=2); axs[2].set_xlabel("Time (s)")
    save(fig, "uav_tracking_signals")

    fig, axs = plt.subplots(3, 1, figsize=(6.0, 5.2), sharex=True)
    axs[0].plot(tt, np.degrees(roll_c), color=SIMULATION_COLOR, lw=1.8, label="Command"); axs[0].plot(tt, np.degrees(roll), color=c[0], lw=1.0, label="Actual"); axs[0].set_ylabel("Roll (deg)"); axs[0].legend(ncol=2)
    axs[1].plot(tt, np.degrees(pitch_c), color=SIMULATION_COLOR, lw=1.8); axs[1].plot(tt, np.degrees(pitch), color=c[0], lw=1.0); axs[1].set_ylabel("Pitch (deg)")
    bearing_u = np.unwrap(bearing); k_ = lambda y: y - 2 * np.pi * np.round((y[0] - bearing_u[0]) / (2 * np.pi))   # continuous yaw, same branch as the bearing
    axs[2].plot(tt, np.degrees(bearing_u), color=SIMULATION_COLOR, lw=1.8, label="Bearing to the USV"); axs[2].plot(tt, np.degrees(k_(yaw_c)), color=c[1], lw=1.0, label="Command")
    axs[2].plot(tt, np.degrees(k_(yaw)), color=c[0], lw=1.0, label="Actual"); axs[2].set_ylabel("Yaw (deg)"); axs[2].set_xlabel("Time (s)"); axs[2].legend(ncol=3)
    save(fig, "uav_attitude")

    # PX4 view of the attitude loop: setpoint sent by the NMPC, attitude estimated by PX4 and simulator ground truth
    if "/px4_1/fmu/out/vehicle_attitude" in raw and "/px4_1/fmu/out/vehicle_attitude_setpoint" in raw:
        def px4(topic):
            tq, mq = raw[topic]; q = np.array([m.q_d if hasattr(m, "q_d") else m.q for m in mq])
            r_, p_, y_ = px4_attitude_to_enu_euler(q); return to_sim(tq), r_, p_, y_
        te_, re_, pe_, ye_ = px4("/px4_1/fmu/out/vehicle_attitude"); tsp_, rsp_, psp_, ysp_ = px4("/px4_1/fmu/out/vehicle_attitude_setpoint")
        est = lambda x, tx: np.degrees(np.interp(t, tx, x))
        fig, axs = plt.subplots(2, 1, figsize=(6.0, 4.4), sharex=True)
        for ax_, nm, sp, es, gt in ((axs[0], "Roll", rsp_, re_, roll), (axs[1], "Pitch", psp_, pe_, pitch)):
            ax_.plot(tt, est(sp, tsp_), color=SIMULATION_COLOR, lw=1.8, label="PX4 setpoint"); ax_.plot(tt, est(es, te_), color=c[1], lw=1.0, label="PX4 estimate")
            ax_.plot(tt, np.degrees(gt), color=c[0], lw=1.0, label="Simulator"); ax_.set_ylabel(f"{nm} (deg)")
        axs[0].legend(ncol=3); axs[1].set_xlabel("Time (s)")
        save(fig, "uav_px4_attitude")
        for nm, sp, es, gt in (("roll", rsp_, re_, roll), ("pitch", psp_, pe_, pitch)):
            print("%-5s: setpoint-estimate (PX4) mean %.3f deg | estimate-simulator mean %.3f deg | setpoint-simulator mean %.3f deg"
                  % (nm, np.mean(est(sp, tsp_) - est(es, te_)), np.mean(est(es, te_) - np.degrees(gt)), np.mean(est(sp, tsp_) - np.degrees(gt))))

    fig, axs = plt.subplots(3, 1, figsize=(6.0, 5.2), sharex=True)
    axs[0].plot(tt, pos[:, 2], color=c[0]); axs[0].axhline(L["z_min"], color=c[3], ls="--", lw=0.9, label="$z_{min}$")
    axs[0].set_ylabel("$z$ (m)"); axs[0].legend()
    axs[1].plot(tt, v_xy, color=c[0], label="$\\|v_{xy}\\|$"); axs[1].plot(tt, np.abs(vel[:, 2]), color=c[1], label="$|v_z|$")
    axs[1].axhline(L["v_xy_max"], color=c[3], ls="--", lw=0.9, label="Limit"); axs[1].set_ylabel("Speed (m/s)"); axs[1].legend(ncol=3)
    axs[2].plot(tt, np.degrees(roll), color=c[0], label="$\\phi$"); axs[2].plot(tt, np.degrees(pitch), color=c[1], label="$\\theta$")
    for s in (1, -1): axs[2].axhline(s * np.degrees(L["phi_max"]), color=c[3], ls="--", lw=0.9)
    axs[2].set_ylabel("Attitude (deg)"); axs[2].set_xlabel("Time (s)"); axs[2].legend(ncol=2)
    save(fig, "uav_constraints_state")

    fig, axs = plt.subplots(2, 1, figsize=(6.0, 3.8), sharex=True)
    axs[0].plot(tt, thrust, color=c[0]); axs[0].axhline(L["T_max"], color=c[3], ls="--", lw=0.9); axs[0].axhline(L["T_min"], color=c[3], ls="--", lw=0.9)
    axs[0].axhline(L["mass"] * L["gravity"], color=SIMULATION_COLOR, ls=":", lw=0.9, label="$mg$"); axs[0].set_ylabel("Thrust (N)"); axs[0].legend()
    for k, col in (("phi", c[0]), ("theta", c[1]), ("psi", c[2])): axs[1].plot(tr_, rate[k], lw=0.6, color=col, label="$\\dot\\%s_{cmd}$" % k)
    for s in (1, -1):
        axs[1].axhline(s * L["phi_dot_max"], color=c[3], ls="--", lw=0.9, label="$\\dot\\phi_{max},\\dot\\theta_{max}$" if s > 0 else None)
        axs[1].axhline(s * L["psi_dot_max"], color=c[2], ls=":", lw=0.9, label="$\\dot\\psi_{max}$" if s > 0 else None)
    axs[1].set_ylabel("Command rate (rad/s)"); axs[1].set_xlabel("Time (s)"); axs[1].legend(ncol=3)
    save(fig, "uav_constraints_input")

    fig, axs = plt.subplots(4, 1, figsize=(6.0, 6.4), sharex=True)
    axs[0].plot(tt, d, color=c[0], label="$d$"); axs[0].axhline(L["d_max"], color=c[3], ls="--", lw=0.9, label="$d_{max}$")
    axs[0].plot(tt, Ltether, color=SIMULATION_COLOR, lw=0.8, label="Tether length $L$"); axs[0].set_ylabel("Distance (m)"); axs[0].legend(ncol=3)
    axs[1].plot(tt, d_dot, color=c[0]); axs[1].axhline(L["d_dot_max"], color=c[3], ls="--", lw=0.9, label="$\\dot d_{max}$")
    axs[1].set_ylabel("$\\dot d$ (m/s)"); axs[1].legend()
    axs[2].plot(tt, eps, color=c[0], label="$\\varepsilon=(L-d)/L$"); axs[2].axhline(0.05, color=c[3], ls="--", lw=0.9, label="$\\varepsilon_0$ of the model")
    axs[2].set_ylabel("Slack ratio"); axs[2].legend(ncol=2); axs[2].set_ylim(0.04, 0.06); axs[2].ticklabel_format(axis="y", useOffset=False)
    axs[3].plot(tt, Fm, color=c[0]); axs[3].set_ylabel("Tension (N)"); axs[3].set_xlabel("Time (s)")
    save(fig, "uav_constraints_tether")

    fig, ax = plt.subplots(figsize=(6.0, 2.2))
    ax.plot(np.arange(len(solve)) * L["Ts"], solve, color=c[0], lw=0.6)
    ax.axhline(1000 * L["Ts"], color=c[3], ls="--", lw=0.9, label="Sampling time")
    ax.set_ylabel("Solve time (ms)"); ax.set_xlabel("Time (s)"); ax.legend()
    save(fig, "uav_solve_time")
    print("[INFO] figures saved to", out)


if __name__ == "__main__":
    main()
