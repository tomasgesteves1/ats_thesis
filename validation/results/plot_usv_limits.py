#!/usr/bin/env python3
"""
Section 5.1: actuator limits of the USV (usv_limits scenario of record_tracking.sh), with the UAV resting on the deck.
Data: data/tracking_usv_limits_nmpc.npz (extract.py). Prints how close each limit comes and writes one figure per panel
to Results/figures/01_tracking. The time axis starts when the reference leaves its first pose.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, set_thesis_style, save_figure, FIGS, USV, REF, LIMIT, USV_LIM

OUT = os.path.join(FIGS, '01_tracking')
K = 1.7
SIZE = (3.1 * K, 2.4 * K)
TS = 0.1                 # s, sampling time of the USV NMPC
U_MIN = -0.3             # m/s, minimum surge speed (soft)
PHASES = [('sprint', 0.0), ('turn', 15.0), ('hold', 25.0), ('astern', 40.0), ('abeam', 80.0)]   # from the table
END = 120.0


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

    def panel(name, ylabel, data, limits):
        fig, ax = plt.subplots(figsize=SIZE)
        ax.grid(True, alpha=0.3)
        for _, ts in PHASES[1:]:
            ax.axvline(ts, color=REF, lw=0.6, alpha=0.5)
        for tx, yx, lab, col in data:
            ax.plot(tx, yx, color=col, lw=0.9, label=lab)
        for y in limits:
            ax.axhline(y, color=LIMIT, ls='--', lw=0.9)
        ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel); ax.set_xlim(0, END)
        if len(data) > 1:
            ax.legend(loc='best', fontsize=7)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))

    L, R = series['left'], series['right']
    panel('usv_limits_thrust', 'Thrust (N)', [(L[0], L[1], 'Left', USV), (R[0], R[1], 'Right', REF)],
          [USV_LIM['T_min'], USV_LIM['T_max']])
    panel('usv_limits_azimuth', 'Azimuth (deg)', [(L[2], L[3], 'Left', USV), (R[2], R[3], 'Right', REF)],
          [-np.degrees(USV_LIM['alpha_max']), np.degrees(USV_LIM['alpha_max'])])
    panel('usv_limits_azimuth_rate', 'Azimuth rate (deg/s)',
          [(L[2][1:], np.diff(L[3]) / TS, 'Left', USV), (R[2][1:], np.diff(R[3]) / TS, 'Right', REF)],
          [-np.degrees(USV_LIM['alpha_rate_max']), np.degrees(USV_LIM['alpha_rate_max'])])
    panel('usv_limits_surge', 'Surge speed (m/s)', [(t, surge, 'Surge', USV)], [U_MIN])

    fig, ax = plt.subplots(figsize=(3.6 * K, 2.6 * K))
    ax.grid(True, alpha=0.3)
    mr = (ref[:, 0] >= t0) & (ref[:, 0] <= t0 + END)
    ax.plot(ref[mr, 1], ref[mr, 2], 'o', color=REF, ms=3, alpha=0.5, label='Reference')
    ax.plot(b[m, 1], b[m, 2], color=USV, lw=0.9, label='USV')
    ax.set_aspect('equal'); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)'); ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'usv_limits_top'))


if __name__ == '__main__':
    main()
