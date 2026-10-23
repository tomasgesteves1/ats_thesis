#!/usr/bin/env python3
"""
Section 5.1.1: nominal tracking of the tilted circle by the NMPC and by the PX4 position controller (reachable
scenario of record_tracking.sh). Data: data/tracking_reachable_{nmpc,px4}.npz (extract.py). Prints the metrics of the
section and writes the figures to Results/figures/01_tracking.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import (load, interp, rms, set_thesis_style, save_figure, FIGS, UAV, USV, PX4, REF, LIMIT)

SKIP = 15.0          # s after the start of the circle
START_ALT = 28.0     # m, altitude at which the circle starts
DURATION = 79.0      # s, longest window; shortened to the common length of the two runs
OUT = os.path.join(FIGS, '01_tracking')


def window(d):
    """Error of the UAV against the published reference, from the ground truth, over the analysis window."""
    dr, ref = d['drone'], d['uav_ref']
    t0 = dr[np.argmax(dr[:, 3] - 0.265 > START_ALT), 0] + SKIP
    m = (dr[:, 0] >= t0) & (dr[:, 0] <= t0 + DURATION)
    t = dr[m, 0]
    e = dr[m, 1:4] - interp(ref, t, (1, 2, 3))
    return t0, t, e


def main():
    set_thesis_style()
    global DURATION
    runs = {'NMPC': load('tracking_reachable_nmpc'), 'PX4': load('tracking_reachable_px4')}
    res = {k: window(d) for k, d in runs.items()}
    DURATION = min(DURATION, min(t[-1] - t0 for t0, t, _ in res.values()))
    res = {k: window(d) for k, d in runs.items()}

    print('=== UAV tracking over %.0f s ===' % DURATION)
    for k, (t0, t, e) in res.items():
        n = np.linalg.norm(e, axis=1)
        print('%-5s RMSE %.1f cm | max %.1f cm | x %.1f y %.1f z %.1f cm' % (k, 100 * rms(n), 100 * n.max(),
              *(100 * np.sqrt(np.mean(e ** 2, axis=0)))))

    # USV, tension, solve times of the NMPC run
    d = runs['NMPC']; t0, t, _ = res['NMPC']
    b, br = d['boat'], d['usv_ref']
    mb = (b[:, 0] >= t0) & (b[:, 0] <= t0 + DURATION)
    eb = b[mb, 1:3] - interp(br, b[mb, 0], (1, 2))
    yref = np.unwrap(br[:, 4]); eyaw = np.degrees(np.angle(np.exp(1j * (b[mb, 4] - np.interp(b[mb, 0], br[:, 0], yref)))))
    print('USV   position RMSE %.1f cm max %.1f cm | heading RMSE %.2f deg max %.2f' % (
        100 * rms(np.linalg.norm(eb, axis=1)), 100 * np.linalg.norm(eb, axis=1).max(), rms(eyaw), np.abs(eyaw).max()))
    f = d['s_tether_force_drone_mag']; mf = (f[:, 0] >= t0) & (f[:, 0] <= t0 + DURATION)
    sep = np.linalg.norm(d['drone'][:, 1:4] - np.column_stack([interp(b, d['drone'][:, 0], (1, 2)),
                                                                np.full(len(d['drone']), 1.25)]), axis=1)
    ms = (d['drone'][:, 0] >= t0) & (d['drone'][:, 0] <= t0 + DURATION)
    print('tension %.2f..%.2f N (mean %.2f) | separation %.1f..%.1f m' % (f[mf, 1].min(), f[mf, 1].max(), f[mf, 1].mean(),
                                                                        sep[ms].min(), sep[ms].max()))
    for k in ('uav', 'usv'):
        s = d['s_%s_solve_time_ms' % k]; s = s[(s[:, 0] >= t0) & (s[:, 0] <= t0 + DURATION), 1]
        print('%s solve time mean %.2f p95 %.2f max %.2f ms' % (k.upper(), s.mean(), np.percentile(s, 95), s.max()))

    # --- Figures: the scenario (3D, one lap) and the tether over time, side by side in the thesis -----------------
    fig = plt.figure(figsize=(3.1, 2.9))
    ax = fig.add_subplot(projection='3d')
    lap = 2 * np.pi / 0.3
    dr = d['drone']; ml = (dr[:, 0] >= t0) & (dr[:, 0] <= t0 + lap)
    r = d['uav_ref']; mr = (r[:, 0] >= t0) & (r[:, 0] <= t0 + lap)
    ax.plot(r[mr, 1], r[mr, 2], r[mr, 3], color=REF, ls='--', lw=1.0, label='UAV reference')
    ax.plot(dr[ml, 1], dr[ml, 2], dr[ml, 3], color=UAV, lw=1.0, label='UAV')
    mbl = (b[:, 0] >= t0) & (b[:, 0] <= t0 + DURATION)
    ax.plot(b[mbl, 1], b[mbl, 2], 0 * b[mbl, 1], color=USV, lw=1.2, label='USV')
    for tk in np.linspace(t0, t0 + lap, 5)[:-1]:
        pu = interp(dr, [tk], (1, 2, 3))[0]; pb = interp(b, [tk], (1, 2))[0]
        ax.plot([pb[0], pu[0]], [pb[1], pu[1]], [1.25, pu[2]], color='#999999', lw=0.6)
    ax.set_xlabel('$x$ (m)', labelpad=-4); ax.set_ylabel('$y$ (m)', labelpad=-4); ax.set_zlabel('$z$ (m)', labelpad=-4)
    ax.tick_params(pad=-2, labelsize=7)
    ax.set_zlim(0, 30); ax.view_init(elev=18, azim=-60)
    ax.legend(loc='upper left', fontsize=7, bbox_to_anchor=(-0.02, 1.02))
    fig.subplots_adjust(left=0.0, right=0.92, bottom=0.04, top=1.0)
    save_figure(fig, os.path.join(OUT, 'nominal_scenario_3d'))

    fig, ax2 = plt.subplots(figsize=(3.1, 2.9))
    tt = d['drone'][ms, 0] - t0
    ax2.plot(tt, sep[ms], color='#333333', lw=1.0, label='separation')
    ax2.set_ylabel('separation (m)'); ax2.set_xlabel('time (s)')
    ax3 = ax2.twinx()
    ax3.plot(f[mf, 0] - t0, f[mf, 1], color=UAV, lw=0.9, label='tension')
    ax3.set_ylabel('tension at the UAV (N)', color=UAV); ax3.tick_params(axis='y', colors=UAV); ax3.grid(False)
    ax2.set_xlim(0, DURATION)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'nominal_scenario_tension'))

    # --- Figure: tracking error of the two controllers ---------------------------------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(6.3, 3.4), sharex=True)
    for k, c in (('PX4', PX4), ('NMPC', UAV)):
        t0k, t, e = res[k]
        axs[0].plot(t - t0k, 100 * np.linalg.norm(e, axis=1), color=c, lw=0.9, label=k)
        axs[1].plot(t - t0k, 100 * e[:, 2], color=c, lw=0.9)
    axs[0].set_ylabel(r'$\|\mathbf{p}-\mathbf{p}_{ref}\|$ (cm)'); axs[1].set_ylabel(r'$z-z_{ref}$ (cm)')
    axs[1].set_xlabel('time (s)'); axs[1].set_xlim(0, DURATION)
    axs[0].legend(loc='upper right', ncol=2)
    fig.align_ylabels(axs)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'nominal_error'))


if __name__ == '__main__':
    main()
