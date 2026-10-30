#!/usr/bin/env python3
"""
Sections 5.1.2 and 5.1.3: step references (waypoints held fixed over the horizon) for both vehicles, steps scenario of
record_tracking.sh. The NMPCs plan the motion within the tether reach and pay-out rate (5.1.2) and within the actuator
limits (5.1.3). Data: data/tracking_steps.npz (extract.py). Prints the metrics and writes the figures.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import (load, interp, set_thesis_style, save_figure, FIGS, EA_FIGS, UAV, USV, REF, LIMIT, UAV_LIM, USV_LIM)
from thesis_style import EA_COLUMN_WIDTH, MM

OUT = os.path.join(FIGS, '01_tracking')
D_MAX = 15.0
Z_ANCHOR = 1.25


def jumps(ref, cols):
    i = np.where(np.linalg.norm(np.diff(ref[:, cols], axis=0), axis=1) > 0.5)[0] + 1
    return ref[i, 0], ref[i][:, cols]


def tilt_of(q):
    x, y = q[:, 1], q[:, 2]   # q = [w, x, y, z]
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def rate(t, x, ts):
    """Rate from the change between consecutive commands over the sampling time of the controller (the recorded
    times jitter, which inflates the rates computed from them)."""
    return t[1:], np.diff(x) / ts


def jerk_rms(t, p, dt=0.1):
    g = np.arange(t[0], t[-1], dt)
    q = np.column_stack([np.interp(g, t, p[:, k]) for k in range(p.shape[1])])
    j = np.diff(q, n=3, axis=0) / dt ** 3
    return float(np.sqrt(np.mean(np.sum(j ** 2, axis=1))))


def main():
    set_thesis_style()
    d = load('tracking_steps')
    dr, b, tel = d['drone'], d['boat'], d['tel']
    tu, wu = jumps(d['uav_ref'], [1, 2, 3])
    tb, wb = jumps(d['usv_ref'], [1, 2])
    t0 = tu[0] - 5.0                      # a few seconds before the first step (after the takeoff)
    keep = tb >= t0; tb, wb = tb[keep], wb[keep]   # the first change of the USV reference is only the start
    t1 = dr[-1, 0]
    print('UAV steps at', np.round(tu - t0, 1), '| USV steps at', np.round(tb - t0, 1))

    # ---- 5.1.2: tether ---------------------------------------------------------------------------------------
    sep = np.linalg.norm(dr[:, 1:4] - np.column_stack([interp(b, dr[:, 0], (1, 2)), np.full(len(dr), Z_ANCHOR)]), axis=1)
    m = dr[:, 0] >= t0
    v = d['s_virtual_tether_separation_speed']; mv = v[:, 0] >= t0
    print('separation max %.2f m (d_max %.1f) | separation speed max %.2f m/s (limit %.1f)' % (
        sep[m].max(), D_MAX, v[mv, 1].max(), UAV_LIM['d_dot_max']))
    f = d['s_tether_force_drone_mag']; mf = f[:, 0] >= t0
    print('tension at the UAV max %.2f N' % f[mf, 1].max())
    print('UAV jerk RMS %.2f m/s^3 | USV jerk RMS %.2f m/s^3' % (jerk_rms(dr[m, 0], dr[m, 1:4]),
                                                                 jerk_rms(b[b[:, 0] >= t0, 0], b[b[:, 0] >= t0, 1:3])))

    # paths and separation, side by side in the thesis
    fig, ax = plt.subplots(figsize=(3.0, 3.0))
    mb = b[:, 0] >= t0
    ax.plot(dr[m, 1], dr[m, 2], color=UAV, lw=1.0, label='UAV')
    ax.plot(b[mb, 1], b[mb, 2], color=USV, lw=1.2, label='USV')
    ax.plot(wu[:, 0], wu[:, 1], 'o', mfc='none', mec=UAV, ms=5, label='UAV waypoints')
    ax.plot(wb[:, 0], wb[:, 1], 's', mfc='none', mec=USV, ms=5, label='USV waypoints')
    for k, w in enumerate(wu):
        ax.annotate('%d (%.0f m)' % (k + 1, w[2]), w[:2], textcoords='offset points', xytext=(-30, 5 + 9 * (k % 2)),
                    fontsize=7, color=UAV)
    for k, w in enumerate(wb):
        ax.annotate(chr(ord('A') + k), w[:2], textcoords='offset points', xytext=(4, -10), fontsize=7, color=USV)
    ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)'); ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.12)
    ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'steps_paths'))

    # thesis, and a column of the extended abstract
    for size, path in (((3.3, 3.0), os.path.join(OUT, 'steps_separation')),
                       ((EA_COLUMN_WIDTH, 52 * MM), os.path.join(EA_FIGS, 'ea_steps_separation'))):
        fig, (a1, a2) = plt.subplots(2, 1, figsize=size, sharex=True)
        a1.plot(dr[m, 0] - t0, sep[m], color='#333333', lw=1.0, label='$d$')
        a1.axhline(D_MAX, color=LIMIT, ls='--', lw=0.9, label='$d_{max}$')
        a1.set_ylabel('$d$ (m)'); a1.legend(loc='lower right', fontsize=7); a1.set_ylim(top=1.08 * D_MAX)
        a2.plot(v[mv, 0] - t0, v[mv, 1], color='#333333', lw=0.9, label=r'$\dot d$')
        a2.axhline(UAV_LIM['d_dot_max'], color=LIMIT, ls='--', lw=0.9, label=r'$\dot d_{max}$')
        a2.set_ylabel(r'$\dot d$ (m/s)'); a2.set_xlabel('time (s)'); a2.legend(loc='lower right', fontsize=7)
        for a in (a1, a2):
            for t in tu:
                a.axvline(t - t0, color=UAV, lw=0.5, alpha=0.5)
            for t in tb:
                a.axvline(t - t0, color=USV, lw=0.5, alpha=0.5, ls=':')
        a1.set_xlim(0, t1 - t0)
        fig.align_ylabels([a1, a2])
        fig.tight_layout()
        save_figure(fig, path)

    # ---- 5.1.3: actuators --------------------------------------------------------------------------------------
    att = d['att']; ma = att[:, 0] >= t0
    tilt = tilt_of(att[ma, 1:5])
    mt = tel['t'] >= t0
    T = tel['thrust'][mt]
    print('UAV tilt max %.1f deg (limit %.1f) | time above %.2f s' % (tilt.max(), np.degrees(UAV_LIM['phi_max']),
          np.sum(tilt > np.degrees(UAV_LIM['phi_max'])) * np.mean(np.diff(att[ma, 0]))))
    for k in ('roll', 'pitch'):
        x = np.degrees(np.abs(tel[k][mt])); dt = np.mean(np.diff(tel['t'][mt]))
        print('UAV |%s| max %.1f deg, %.2f s above the limit' % (k, x.max(), np.sum(x > np.degrees(UAV_LIM['phi_max'])) * dt))
    print('UAV thrust %.2f..%.2f N (limits %.2f..%.2f)' % (T.min(), T.max(), UAV_LIM['T_min'], UAV_LIM['T_max']))
    for k in ('roll_cmd', 'pitch_cmd'):
        _, r = rate(tel['t'][mt], tel[k][mt], 0.02)
        print('%s rate max %.3f rad/s (limit %.2f)' % (k, np.abs(r).max(), UAV_LIM['rate_max']))
    _, r = rate(tel['t'][mt], T, 0.02)
    print('thrust rate max %.1f N/s (limit %.1f)' % (np.abs(r).max(), UAV_LIM['T_rate_max']))
    zmin = dr[m, 3].min(); print('UAV height min %.2f m' % zmin)
    vxy = np.hypot(dr[m, 5], dr[m, 6]); print('UAV horizontal speed max %.2f m/s' % vxy.max())
    usv = {}
    for side in ('left', 'right'):
        th, al = d['s_boat_thrusters_%s_thrust' % side], d['s_boat_thrusters_%s_pos' % side]
        th, al = th[th[:, 0] >= t0], al[al[:, 0] >= t0]
        _, rt = rate(th[:, 0], th[:, 1], 0.1); _, ra = rate(al[:, 0], al[:, 1], 0.1)
        usv[side] = (th, al)
        print('USV %-5s thrust %.0f..%.0f N (limits %.0f..%.0f), rate max %.0f N/s (%.0f) | angle max %.1f deg (%.0f), '
              'rate max %.1f deg/s (%.0f)' % (side, th[:, 1].min(), th[:, 1].max(), USV_LIM['T_min'], USV_LIM['T_max'],
              np.abs(rt).max(), USV_LIM['T_rate_max'], np.degrees(np.abs(al[:, 1]).max()),
              np.degrees(USV_LIM['alpha_max']), np.degrees(np.abs(ra).max()), np.degrees(USV_LIM['alpha_rate_max'])))

    # UAV and USV, each a stack on a shared time axis, side by side in the thesis
    def steps_lines(axs):
        for a in axs:
            for t in tu:
                a.axvline(t - t0, color=UAV, lw=0.5, alpha=0.5)
            for t in tb:
                a.axvline(t - t0, color=USV, lw=0.5, alpha=0.5, ls=':')
        axs[1].set_xlabel('time (s)'); axs[0].set_xlim(0, t1 - t0)

    fig, axs = plt.subplots(2, 1, figsize=(3.15, 3.4), sharex=True)
    axs[0].plot(tel['t'][mt] - t0, np.degrees(np.abs(tel['roll'][mt])), color=UAV, lw=0.9, label=r'$|\phi|$')
    axs[0].plot(tel['t'][mt] - t0, np.degrees(np.abs(tel['pitch'][mt])), color=UAV, lw=0.9, ls='--',
                label=r'$|\theta|$')
    axs[0].axhline(np.degrees(UAV_LIM['phi_max']), color=LIMIT, ls='--', lw=0.9)
    axs[0].set_ylabel('UAV tilt (deg)'); axs[0].legend(loc='upper right', fontsize=7, ncol=2)
    axs[1].plot(tel['t'][mt] - t0, T, color=UAV, lw=0.9)
    for lim in (UAV_LIM['T_min'], UAV_LIM['T_max']):
        axs[1].axhline(lim, color=LIMIT, ls='--', lw=0.9)
    axs[1].set_ylabel('UAV thrust (N)')
    steps_lines(axs)
    fig.align_ylabels(axs); fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'steps_actuators_uav'))

    fig, axs = plt.subplots(2, 1, figsize=(3.15, 3.4), sharex=True)
    for side, ls in (('left', '-'), ('right', '--')):
        th, al = usv[side]
        axs[0].plot(th[:, 0] - t0, th[:, 1], color=USV, lw=0.9, ls=ls, label=side)
        axs[1].plot(al[:, 0] - t0, np.degrees(al[:, 1]), color=USV, lw=0.9, ls=ls)
    for lim in (USV_LIM['T_min'], USV_LIM['T_max']):
        axs[0].axhline(lim, color=LIMIT, ls='--', lw=0.9)
    for lim in (-30, 30):
        axs[1].axhline(lim, color=LIMIT, ls='--', lw=0.9)
    axs[0].set_ylabel('USV thrust (N)'); axs[1].set_ylabel('USV azimuth (deg)')
    axs[0].legend(loc='upper right', fontsize=7, ncol=2)
    steps_lines(axs)
    fig.align_ylabels(axs); fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'steps_actuators_usv'))


if __name__ == '__main__':
    main()
