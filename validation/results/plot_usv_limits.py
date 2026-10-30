#!/usr/bin/env python3
"""
Section 5.1: actuator limits of the USV (usv_limits scenario of record_tracking.sh), with the UAV resting on the deck.
Data: data/tracking_usv_limits_nmpc.npz (extract.py). Prints how close each limit comes and writes the top view and the stacked
inputs to Results/figures/01_tracking. The time axis starts when the reference leaves its first pose.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from plot_mission import boat
from common import load, interp, set_thesis_style, save_figure, fig_size, FIGS, USV, REF, LIMIT, USV_LIM, COLOR_CYCLE

OUT = os.path.join(FIGS, '01_tracking')
K = 1.7
SIZE = (3.1 * K, 2.4 * K)
RIGHT = COLOR_CYCLE[5]   # right thruster, apart from the gray of the references
TS = 0.1                 # s, sampling time of the USV NMPC
U_MIN = -0.3             # m/s, minimum surge speed (soft)
PHASES = [('sprint', 0.0), ('turn', 15.0), ('hold', 25.0), ('astern', 40.0), ('abeam', 80.0)]   # from the table
END = 120.0
GLYPHS = [12.0, 27.0, 60.0, 88.0, 105.0]   # s, instants at which the USV is drawn
GLYPH = dict(L=2.9, W=1.4)                                        # m, drawn smaller than the USV to keep the path visible
HOLDS = [30.0, 60.0, 100.0]                                       # s, the reference holds a point


def main():
    set_thesis_style()
    d = load('tracking_usv_limits_nmpc')
    b, ref = d['boat'], d['usv_ref']
    ref = ref[ref[:, 0] > 1.0]
    rv = np.hypot(*np.gradient(ref[:, 1:3], ref[:, 0], axis=0).T)
    t0 = ref[np.flatnonzero(rv > 0.05)[0], 0]
    m = (b[:, 0] >= t0) & (b[:, 0] <= t0 + END)
    t = b[m, 0] - t0
    surge = b[m, 5]                                 # body-frame velocity as published by the odometry
    speed = np.hypot(*np.gradient(b[m, 1:3], b[m, 0], axis=0).T)
    print('speed max %.2f m/s, surge min %.2f m/s (u_min %.1f)' % (speed.max(), surge.min(), U_MIN))
    series = {}
    for side in ('left', 'right'):
        T = d['s_boat_thrusters_%s_thrust' % side]; A = d['s_boat_thrusters_%s_pos' % side]
        mt = (T[:, 0] >= t0) & (T[:, 0] <= t0 + END); ma = (A[:, 0] >= t0) & (A[:, 0] <= t0 + END)
        tT, vT, tA, vA = T[mt, 0] - t0, T[mt, 1], A[ma, 0] - t0, np.degrees(A[ma, 1])
        series[side] = (tT, vT, tA, vA)
        print('%-5s thrust %.0f..%.0f N (%.0f..%.0f), rate max %.0f N/s (%.0f) | azimuth %.1f..%.1f deg (+-%.0f), '
              'rate max %.1f deg/s (%.0f)' % (side, vT.min(), vT.max(), USV_LIM['T_min'], USV_LIM['T_max'],
                                              np.abs(np.diff(vT) / TS).max(), USV_LIM['T_rate_max'], vA.min(), vA.max(),
                                              np.degrees(USV_LIM['alpha_max']), np.abs(np.diff(vA) / TS).max(),
                                              np.degrees(USV_LIM['alpha_rate_max'])))

    # Stacked on a shared time axis, so that the limits reached at the same instant line up
    L, R = series['left'], series['right']
    rows = [('Thrust\n(N)', [(L[0], L[1], USV), (R[0], R[1], RIGHT)], [USV_LIM['T_min'], USV_LIM['T_max']]),
            ('Thrust rate\n(N/s)', [(L[0][1:], np.diff(L[1]) / TS, USV), (R[0][1:], np.diff(R[1]) / TS, RIGHT)],
             [-USV_LIM['T_rate_max'], USV_LIM['T_rate_max']]),
            ('Azimuth\n(deg)', [(L[2], L[3], USV), (R[2], R[3], RIGHT)],
             [-np.degrees(USV_LIM['alpha_max']), np.degrees(USV_LIM['alpha_max'])]),
            ('Azimuth rate\n(deg/s)', [(L[2][1:], np.diff(L[3]) / TS, USV), (R[2][1:], np.diff(R[3]) / TS, RIGHT)],
             [-np.degrees(USV_LIM['alpha_rate_max']), np.degrees(USV_LIM['alpha_rate_max'])]),
            ('Surge speed\n(m/s)', [(t, surge, 'k')], [U_MIN])]
    fig, axs = plt.subplots(len(rows), 1, figsize=fig_size('usv_limits_inputs'), sharex=True)
    for ax, (ylabel, data, limits) in zip(axs, rows):
        ax.grid(True, alpha=0.3)
        for _, ts in PHASES[1:]:
            ax.axvline(ts, color=REF, lw=0.6, alpha=0.5)
        for tx, yx, col in data:
            ax.plot(tx, yx, color=col, lw=0.9)
        for y in limits:
            ax.axhline(y, color=LIMIT, ls='--', lw=0.9)
        ax.set_ylabel(ylabel)
    axs[-1].set_xlabel('Time (s)'); axs[-1].set_xlim(0, END)
    handles = [plt.Line2D([], [], color=USV, lw=0.9), plt.Line2D([], [], color=RIGHT, lw=0.9),
               plt.Line2D([], [], color=LIMIT, ls='--', lw=0.9)]
    axs[0].legend(handles, ['Left', 'Right', 'Limit'], loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=3,
                  fontsize=7, frameon=False, borderaxespad=0.1, handlelength=2.5)
    fig.align_ylabels(axs); fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'usv_limits_inputs'))

    fig, ax = plt.subplots(figsize=fig_size('usv_limits_top'))
    ax.grid(True, alpha=0.3)
    mr = (ref[:, 0] >= t0) & (ref[:, 0] <= t0 + END)
    ax.plot(ref[mr, 1], ref[mr, 2], 'o', color=REF, ms=3, alpha=0.5, label='Reference')
    ax.plot(b[m, 1], b[m, 2], color=USV, lw=0.9, label='USV')
    # Heading of the USV at a few instants, and of the reference at the points it holds
    for tb in GLYPHS:
        j = np.argmin(np.abs(b[:, 0] - t0 - tb))
        boat(ax, b[j, 1], b[j, 2], b[j, 4], alpha=0.8, **GLYPH)
    for th in HOLDS:
        j = np.argmin(np.abs(ref[:, 0] - t0 - th))
        boat(ax, ref[j, 1], ref[j, 2], ref[j, 4], color='none', alpha=1.0, **GLYPH)
        ax.patches[-1].set(edgecolor=REF, linestyle='--', linewidth=0.8, zorder=4)
    p0, p1 = b[m, 1:3][0], b[m, 1:3][-1]
    ax.annotate('Start', p0, xytext=(0, -9), textcoords='offset points', ha='center', va='top', fontsize=7)
    ax.annotate('End', p1, xytext=(-9, 0), textcoords='offset points', ha='right', va='center', fontsize=7)
    ax.set_aspect('equal'); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)'); ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'usv_limits_top'))


if __name__ == '__main__':
    main()
