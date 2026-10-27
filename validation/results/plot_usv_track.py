#!/usr/bin/env python3
"""
Section 5.1: accuracy of the USV on the figure-eight (usv_track scenario of record_tracking.sh), with the UAV resting
on the deck. Data: data/tracking_usv_track_nmpc.npz (extract.py). The error is measured from the ground truth against
the published reference at the same simulation instant, while the reference moves. Prints the metrics of the section
and writes one figure per panel to Results/figures/01_tracking.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, rms, set_thesis_style, save_figure, FIGS, USV, REF, LIMIT, USV_LIM

OUT = os.path.join(FIGS, '01_tracking')
K = 1.7
SIZE = (3.1 * K, 2.4 * K)


def wrap(a):
    return np.angle(np.exp(1j * a))


def main():
    set_thesis_style()
    d = load('tracking_usv_track_nmpc')
    b, ref = d['boat'], d['usv_ref']
    # The reference moves between its first and its last pose (holds before and after are left out)
    rv = np.hypot(*np.gradient(ref[:, 1:3], ref[:, 0], axis=0).T)
    moving = np.flatnonzero(rv > 0.05)
    t0, t1 = ref[moving[0], 0], ref[moving[-1], 0]
    m = (b[:, 0] >= t0) & (b[:, 0] <= t1)
    t = b[m, 0] - t0
    p = b[m, 1:3]
    r = interp(ref, b[m, 0], (1, 2))
    e = np.linalg.norm(p - r, axis=1)
    psi_ref = np.interp(b[m, 0], ref[:, 0], np.unwrap(ref[:, 4]))
    e_psi = np.degrees(wrap(b[m, 4] - psi_ref))
    speed = np.hypot(*np.gradient(p, b[m, 0], axis=0).T)
    speed_ref = np.hypot(*np.gradient(r, b[m, 0], axis=0).T)
    print('=== USV tracking over %.0f s ===' % (t1 - t0))
    print('position RMSE %.1f cm, max %.1f cm | heading RMSE %.2f deg, max %.2f deg | speed max %.2f m/s' % (
        100 * rms(e), 100 * e.max(), rms(e_psi), np.abs(e_psi).max(), speed.max()))

    th = {}
    for side in ('left', 'right'):
        T = d['s_boat_thrusters_%s_thrust' % side]; A = d['s_boat_thrusters_%s_pos' % side]
        mt = (T[:, 0] >= t0) & (T[:, 0] <= t1); ma = (A[:, 0] >= t0) & (A[:, 0] <= t1)
        th[side] = (T[mt, 0] - t0, T[mt, 1], A[ma, 0] - t0, A[ma, 1])
        print('%-5s thrust %.0f..%.0f N (limits %.0f..%.0f), azimuth %.1f..%.1f deg (limit %.0f)' % (
            side, T[mt, 1].min(), T[mt, 1].max(), USV_LIM['T_min'], USV_LIM['T_max'],
            np.degrees(A[ma, 1].min()), np.degrees(A[ma, 1].max()), np.degrees(USV_LIM['alpha_max'])))
    s = d['s_usv_solve_time_ms']; s = s[(s[:, 0] >= t0) & (s[:, 0] <= t1), 1]
    print('USV solve time mean %.2f max %.2f ms' % (s.mean(), s.max()))

    # --- Paths seen from above ----------------------------------------------------------------------------------
    # Drawn as the top views of the UAV: x up, y to the left, so that the figure-eight lies across the page
    fig, ax = plt.subplots(figsize=(4.4 * K, 2.0 * K))
    ax.grid(True, alpha=0.3)
    ax.plot(r[:, 1], r[:, 0], color=REF, lw=2.5, alpha=0.45, label='Reference')
    ax.plot(p[:, 1], p[:, 0], color=USV, lw=0.9, label='USV')
    ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('$y$ (m)'); ax.set_ylabel('$x$ (m)')
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=7, frameon=False)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'usv_track_top'))

    # --- Position and heading error, with the speed of the reference --------------------------------------------
    for name, y, ylabel, scale in (('usv_track_error', 100 * e, 'Position error (cm)', 1),
                                   ('usv_track_heading', e_psi, 'Heading error (deg)', 1)):
        fig, ax = plt.subplots(figsize=SIZE)
        ax.grid(True, alpha=0.3)
        ax.plot(t, y, color=USV, lw=0.9)
        ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel); ax.set_xlim(0, t[-1])
        ax2 = ax.twinx()
        ax2.fill_between(t, speed_ref, color=REF, alpha=0.15, lw=0)
        ax2.set_ylabel('Reference speed (m/s)'); ax2.set_ylim(0, 4)
        ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))

    # --- Thrusters against their limits ----------------------------------------------------------------------------
    for name, k, ylabel, lims, conv in (('usv_track_thrust', 1, 'Thrust (N)', [USV_LIM['T_min'], USV_LIM['T_max']], 1.0),
                                        ('usv_track_azimuth', 3, 'Azimuth (deg)',
                                         [-np.degrees(USV_LIM['alpha_max']), np.degrees(USV_LIM['alpha_max'])],
                                         np.degrees(1.0))):
        fig, ax = plt.subplots(figsize=SIZE)
        ax.grid(True, alpha=0.3)
        for side, col in (('left', USV), ('right', REF)):
            ax.plot(th[side][k - 1], conv * th[side][k], color=col, lw=0.9, label=side.capitalize())
        for y in lims:
            ax.axhline(y, color=LIMIT, ls='--', lw=0.9)
        ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel); ax.set_xlim(0, t[-1]); ax.legend(loc='best', fontsize=7)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))


if __name__ == '__main__':
    main()
