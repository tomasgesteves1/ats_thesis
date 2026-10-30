#!/usr/bin/env python3
"""
Section 5.1: actuator limits of the UAV (limits scenario of record_tracking.sh). Data: data/tracking_limits_nmpc.npz
(extract.py). Prints how close each limit comes and writes the top view and two stacks
(horizontal and vertical motion) to Results/figures/01_tracking.
The time axis starts when the reference leaves its first point; the phases of the reference are marked.
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from common import load, interp, set_thesis_style, save_figure, fig_size, FIGS, HERE, UAV, USV, REF, LIMIT, UAV_LIM, COLOR_CYCLE

sys.path.insert(0, os.path.join(HERE, '..', 'uav_tracking'))
from limits_reference import phases  # noqa: E402

TABLE = os.path.join(HERE, '..', '..', 'mission', 'test_uav_circle', 'config', 'limits.csv')
OUT = os.path.join(FIGS, '01_tracking')
Z_WINCH = 1.3
Z_MIN = 2.0
TS = 0.02            # s, sampling time of the NMPC (rates from consecutive commands, see the pipeline notes)
RAMP = 8.0           # s, entry from hover of the reference (test_uav_circle ramp_time): tau = t - RAMP/2 after it
K = 1.7
MEDIAN = 9           # samples (0.18 s): removes the fast ripple that the tether force leaves on the roll and pitch rates
                     # while keeping their plateaus at the limit (a low-pass filter overshoots the limit at their edges);
                     # the thrust rate is left unfiltered, since it touches its limit for only 2-3 samples on each step
                     # and has little ripple there


def phase_marks():
    """Start time of each phase on the time axis of the run (time of the reference, shifted by the entry ramp)."""
    marks, tau = [], 0.0
    for name, T, _ in phases():
        marks.append((name, tau + RAMP / 2 if tau > 0 else 0.0)); tau += round(T / 0.01) * 0.01
    return marks, tau + RAMP / 2


def main():
    set_thesis_style()
    tab = np.loadtxt(TABLE, delimiter=',', skiprows=1)
    d = load('tracking_limits_nmpc')
    ref, dr, tel = d['uav_ref'], d['drone'], d['tel']
    # Start of the table: the reference leaves its first point at the end of the first phase (hover), stretched by the
    # entry ramp, so the table starts that long before the last sample on the first point (the reference also sits on
    # this point before the first odometry, so the search starts a few seconds after it)
    on_first = np.linalg.norm(ref[:, 1:4] - tab[0, 1:4], axis=1) < 1e-4
    idx = np.where(on_first & (ref[:, 0] > dr[0, 0] + 5.0))[0]
    cut = np.where(np.diff(idx) > 1)[0]
    marks, t_end = phase_marks()
    t0 = ref[idx[cut[0]] if len(cut) else idx[-1], 0] - marks[1][1]
    t1 = t0 + t_end

    m = (dr[:, 0] >= t0) & (dr[:, 0] <= t1)
    t = dr[m, 0] - t0
    p = dr[m, 1:4]
    v = np.gradient(p, dr[m, 0], axis=0)
    v_xy = np.hypot(v[:, 0], v[:, 1])
    b = interp(d['boat'], dr[m, 0], (1, 2))
    r = p - np.column_stack([b, np.full(len(b), Z_WINCH)])
    dist = np.linalg.norm(r, axis=1)
    d_dot = np.gradient(dist, dr[m, 0])

    mt = (tel['t'] >= t0) & (tel['t'] <= t1)
    tt = tel['t'][mt] - t0
    tilt = np.degrees(np.arccos(np.cos(tel['roll'][mt]) * np.cos(tel['pitch'][mt])))
    tilt_cmd = np.degrees(np.arccos(np.cos(tel['roll_cmd'][mt]) * np.cos(tel['pitch_cmd'][mt])))
    roll_rate = np.degrees(np.diff(tel['roll_cmd'][mt]) / TS)
    pitch_rate = np.degrees(np.diff(tel['pitch_cmd'][mt]) / TS)
    thrust = tel['thrust'][mt]
    thrust_rate = np.diff(thrust) / TS

    lim_tilt = np.degrees(UAV_LIM['phi_max']); lim_rate = np.degrees(UAV_LIM['rate_max'])
    print('tilt max %.1f deg (cmd %.1f), limit %.1f' % (tilt.max(), tilt_cmd.max(), lim_tilt))
    print('roll/pitch command rate max %.1f / %.1f deg/s, limit %.1f' % (np.abs(roll_rate).max(), np.abs(pitch_rate).max(),
                                                                         lim_rate))
    print('thrust %.2f..%.2f N, limits %.2f..%.2f; rate max %.1f N/s, limit %.0f' % (
        thrust.min(), thrust.max(), UAV_LIM['T_min'], UAV_LIM['T_max'], np.abs(thrust_rate).max(), UAV_LIM['T_rate_max']))
    print('horizontal speed max %.2f m/s, limit %.0f' % (v_xy.max(), UAV_LIM['v_max']))
    print('vertical speed %.2f..%.2f m/s, limit %.0f' % (v[:, 2].min(), v[:, 2].max(), UAV_LIM['vz_max']))
    print('height min %.2f m, limit %.0f' % (p[:, 2].min(), Z_MIN))
    print('tether: separation %.1f..%.1f m, |d_dot| max %.2f m/s' % (dist.min(), dist.max(), np.abs(d_dot).max()))
    for name, ts in marks:
        print('  phase %-30s from %.1f s' % (name, ts))

    # Stacked on a shared time axis, so that the limits reached at the same instant line up
    def stack(name, rows, size, xlim):
        fig, axs = plt.subplots(len(rows), 1, figsize=size, sharex=True)
        for ax, (ylabel, series, limits) in zip(axs, rows):
            ax.grid(True, alpha=0.3)
            for _, ts in marks[1:]:
                ax.axvline(ts, color=REF, lw=0.6, alpha=0.5)
            for (tx, yx, lab, col) in series:
                ax.plot(tx, yx, color=col, lw=0.9, label=lab)
            for i, y in enumerate(limits):
                ax.axhline(y, color=LIMIT, ls='--', lw=0.9, label='Limit' if i == 0 else None)
            ax.set_ylabel(ylabel)
            if len(series) > 1:
                ax.legend(loc='best', fontsize=7, ncol=3)
        axs[-1].set_xlabel('Time (s)'); axs[-1].set_xlim(*xlim)
        fig.align_ylabels(axs); fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))

    # Each stack spans only its manoeuvres: the arcs, then the move down and the steps in height
    t_split = dict(marks)['move out and down']
    stack('limits_horizontal', [
        ('Tilt\n(deg)', [(tt, tilt, 'Measured', UAV), (tt, tilt_cmd, 'Commanded', REF)], [lim_tilt]),
        ('Roll and pitch\nrate (deg/s)', [(tt[1:], medfilt(roll_rate, MEDIAN), 'Roll', UAV),
                                         (tt[1:], medfilt(pitch_rate, MEDIAN), 'Pitch', COLOR_CYCLE[1])],
         [lim_rate, -lim_rate]),
        ('Horizontal\nspeed (m/s)', [(t, v_xy, 'Speed', UAV)], [UAV_LIM['v_max']])], fig_size('limits_horizontal'), (0, t_split))
    stack('limits_vertical', [
        ('Height\n(m)', [(t, p[:, 2], 'Height', UAV)], [Z_MIN]),
        ('Vertical\nspeed (m/s)', [(t, v[:, 2], 'Vertical speed', UAV)], [UAV_LIM['vz_max'], -UAV_LIM['vz_max']]),
        ('Thrust\n(N)', [(tt, thrust, 'Thrust', UAV)], [UAV_LIM['T_min'], UAV_LIM['T_max']]),
        ('Thrust rate\n(N/s)', [(tt[1:], thrust_rate, 'Thrust rate', UAV)],
         [UAV_LIM['T_rate_max'], -UAV_LIM['T_rate_max']])], fig_size('limits_vertical'), (t_split - 2.0, t_end))

    fig, ax = plt.subplots(figsize=fig_size('limits_top'))
    ax.grid(True, alpha=0.3)
    mr = (ref[:, 0] >= t0) & (ref[:, 0] <= t1)
    ax.plot(ref[mr, 2], ref[mr, 1], color=REF, lw=0.9, ls='--', label='Reference')
    ax.plot(p[:, 1], p[:, 0], color=UAV, lw=0.9, label='UAV')
    ax.plot(b[:, 1].mean(), b[:, 0].mean(), 's', color=USV, ms=5, label='USV')
    ax.annotate('Start', (p[0, 1], p[0, 0]), xytext=(-9, 0), textcoords='offset points', ha='right', va='center', fontsize=7)
    ax.annotate('End', (p[-1, 1], p[-1, 0]), xytext=(9, 0), textcoords='offset points', ha='left', va='center', fontsize=7)
    ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('$y$ (m)'); ax.set_ylabel('$x$ (m)')
    ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'limits_top'))


if __name__ == '__main__':
    main()
