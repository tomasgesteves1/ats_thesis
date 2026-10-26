#!/usr/bin/env python3
"""
Section 5.1: accuracy of the UAV on the crescent, with the NMPC and with the PX4 position controller (crescent scenario of
record_tracking.sh). Data: data/tracking_crescent_{nmpc,px4}.npz (extract.py). The error is measured from the ground
truth against the published reference at the same simulation instant, over the last three complete laps of each run.
Prints the metrics of the section and writes one figure per panel to Results/figures/01_tracking.
"""
import os
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from common import load, interp, rms, set_thesis_style, save_figure, FIGS, HERE, UAV, USV, PX4, REF, LIMIT, UAV_LIM

TABLE = os.path.join(HERE, '..', '..', 'mission', 'test_uav_circle', 'config', 'crescent.csv')
OUT = os.path.join(FIGS, '01_tracking')
Z_WINCH = 1.3        # m, height of the winch
D_MAX = 35.0         # m, maximum separation in this test
K_F, W_MIN, W_MAX = 1.11e-5, 150.0, 1000.0   # rotor thrust coefficient and speed range of the x500 (Section 4.1)
CMD_CUTOFF = 1.0     # Hz, low-pass filter of the plotted commands
LAPS = 3
# Figures are drawn about 1.7 times larger than they appear in the thesis, as those of Chapter 4, so that their text
# matches once LaTeX scales them down
K = 1.7
SIZE = (3.1 * K, 2.4 * K)    # one panel; two side by side in the thesis
NAMES = {'NMPC': 'nmpc', 'PX4': 'px4'}
COLORS = {'NMPC': UAV, 'PX4': PX4}


def lap_passes(d, tab):
    """Instants at which the published reference passes on the first point of the table (start of each lap)."""
    ref = d['uav_ref']
    idx = np.where(np.linalg.norm(ref[:, 1:4] - tab[0, 1:4], axis=1) < 1e-4)[0]
    cuts = np.where(np.diff(idx) > 1)[0]
    return ref[np.r_[idx[cuts], idx[-1]], 0]


def analyse(d, tab):
    passes = lap_passes(d, tab)
    t0, t1 = passes[-1 - LAPS], passes[-1]
    dr, ref, b = d['drone'], d['uav_ref'], d['boat']
    m = (dr[:, 0] >= t0) & (dr[:, 0] <= t1)
    t = dr[m, 0]
    p = dr[m, 1:4]
    r = interp(ref, t, (1, 2, 3))
    e = p - r
    # Along-track and cross-track components, from the direction of motion of the reference
    vr = np.gradient(r, t, axis=0)
    sp = np.linalg.norm(vr, axis=1)
    tang = vr / np.maximum(sp, 1e-6)[:, None]
    e_al = np.sum(e * tang, axis=1)
    e_cr = np.linalg.norm(e - e_al[:, None] * tang, axis=1)
    anchor = np.column_stack([interp(b, t, (1, 2)), np.full(len(t), Z_WINCH)])
    dist = np.linalg.norm(p - anchor, axis=1)
    ma = (d['att'][:, 0] >= t0) & (d['att'][:, 0] <= t1)
    q = d['att'][ma, 1:5]                     # PX4 quaternion [w, x, y, z]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    # Commands sent to the autopilot (same topic for both controllers): tilt of the commanded attitude and thrust,
    # converted from the normalised throttle with the relation of the rotors used by the NMPC (Section 4.3)
    ms = (d['att_sp'][:, 0] >= t0) & (d['att_sp'][:, 0] <= t1)
    qs = d['att_sp'][ms, 1:5]
    tilt_cmd = np.degrees(np.arccos(np.clip(1 - 2 * (qs[:, 1] ** 2 + qs[:, 2] ** 2), -1, 1)))
    w = W_MIN + d['att_sp'][ms, 5] * (W_MAX - W_MIN)
    thrust_cmd = 4 * K_F * w ** 2
    return dict(t_cmd=d['att_sp'][ms, 0] - t0, tilt_cmd=tilt_cmd, thrust_cmd=thrust_cmd, t0=t0, t=t - t0, p=p, r=r, e=e, n=np.linalg.norm(e, axis=1), e_al=e_al, e_cr=e_cr,
                sp_ref=np.hypot(vr[:, 0], vr[:, 1]), sp=np.hypot(*np.gradient(p, t, axis=0)[:, :2].T),
                dist=dist, d_dot=np.gradient(dist, t), t_att=d['att'][ma, 0] - t0, tilt=tilt)


def panel():
    fig, ax = plt.subplots(figsize=SIZE)
    ax.grid(True, alpha=0.3)
    return fig, ax


def main():
    set_thesis_style()
    tab = np.loadtxt(TABLE, delimiter=',', skiprows=1)
    period = tab[-1, 0] + (tab[1, 0] - tab[0, 0])
    runs = {k: load('tracking_crescent_' + v) for k, v in NAMES.items()}
    res = {k: analyse(d, tab) for k, d in runs.items()}

    print('=== UAV tracking over the last %d laps (%.1f s) ===' % (LAPS, LAPS * period))
    for k, a in res.items():
        hor = np.hypot(a['e'][:, 0], a['e'][:, 1])
        lag = -np.mean(a['e_al'][a['sp_ref'] > 3.0] / a['sp_ref'][a['sp_ref'] > 3.0])
        print('%-5s RMSE %.1f cm | max %.1f cm | horizontal %.1f vertical %.1f cm | along %.1f cross %.1f cm | '
              'lag on the fast arcs %.0f ms' % (k, 100 * rms(a['n']), 100 * a['n'].max(), 100 * rms(hor),
                                                100 * rms(a['e'][:, 2]), 100 * rms(a['e_al']), 100 * rms(a['e_cr']),
                                                1000 * lag))
        print('      speed max %.2f m/s | d %.1f..%.1f m | |d_dot| max %.2f m/s | tilt max %.1f deg' % (
            a['sp'].max(), a['dist'].min(), a['dist'].max(), np.abs(a['d_dot']).max(), a['tilt'].max()))
    for k, a in res.items():
        tc = a['t_cmd']
        print('%-5s command: tilt max %.1f mean %.1f deg, rate rms %.1f deg/s | thrust %.1f..%.1f N, rate rms %.1f N/s' % (
            k, a['tilt_cmd'].max(), a['tilt_cmd'].mean(), rms(np.gradient(a['tilt_cmd'], tc)),
            a['thrust_cmd'].min(), a['thrust_cmd'].max(), rms(np.gradient(a['thrust_cmd'], tc))))
    tel = runs['NMPC']['tel']
    a = res['NMPC']
    mt = (tel['t'] >= a['t0']) & (tel['t'] <= a['t0'] + LAPS * period)
    print('NMPC  thrust %.1f..%.1f N' % (tel['thrust'][mt].min(), tel['thrust'][mt].max()))
    f = runs['NMPC']['s_tether_force_drone_mag']
    mf = (f[:, 0] >= a['t0']) & (f[:, 0] <= a['t0'] + LAPS * period)
    print('tension %.2f..%.2f N' % (f[mf, 1].min(), f[mf, 1].max()))
    for k in ('uav', 'usv'):
        s = runs['NMPC']['s_%s_solve_time_ms' % k]
        s = s[(s[:, 0] >= a['t0']) & (s[:, 0] <= a['t0'] + LAPS * period), 1]
        print('%s solve time mean %.2f max %.2f ms' % (k.upper(), s.mean(), s.max()))

    # --- Scenario: the crescent seen from above, coloured by the speed of the reference -------------------------------
    # Top views are rotated by 90 deg so that the crescent lies horizontally: y to the left, x upwards
    fig, ax = plt.subplots(figsize=(4.4 * K, 2.6 * K))
    sp = np.hypot(tab[:, 4], tab[:, 5])
    sc = ax.scatter(tab[:, 2], tab[:, 1], c=sp, s=1.5, cmap='viridis', rasterized=True)
    for k in np.linspace(0, len(tab), 9, endpoint=False).astype(int):
        j = (k + 30) % len(tab)
        ax.annotate('', xy=tab[j, [2, 1]], xytext=tab[k, [2, 1]], arrowprops=dict(arrowstyle='->', color='k', lw=0.7))
    ax.plot(0, 0, 's', color=USV, ms=6)
    ax.annotate('USV', (0, 0), xytext=(6, 0), textcoords='offset points', va='center', fontsize=9)
    fig.colorbar(sc, ax=ax, label='Reference speed (m/s)', shrink=0.7, pad=0.02)
    ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('$y$ (m)'); ax.set_ylabel('$x$ (m)')
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_top'))

    # --- Overlaid tracking: x, y, z against the reference over the middle lap, and the paths seen from above ---------
    for c, lab in enumerate(('x', 'y', 'z')):
        fig, ax = plt.subplots(figsize=(2.05 * K, 1.75 * K))   # three side by side in the thesis
        ax.grid(True, alpha=0.3)
        a = res['NMPC']
        mid = (a['t'] >= period) & (a['t'] <= 2 * period)
        ax.plot(a['t'][mid] - period, a['r'][mid, c], color=REF, lw=2.5, alpha=0.45, label='Reference')
        for k, a in res.items():
            mid = (a['t'] >= period) & (a['t'] <= 2 * period)
            ax.plot(a['t'][mid] - period, a['p'][mid, c], color=COLORS[k], lw=0.9, label=k)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('$%s$ (m)' % lab); ax.set_xlim(0, period)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, 'crescent_track_%s' % lab))

    # Seen from above, rotated as the scenario figure
    fig, ax = plt.subplots(figsize=(4.4 * K, 2.4 * K))
    ax.grid(True, alpha=0.3)
    ax.plot(tab[:, 2], tab[:, 1], color=REF, lw=2.5, alpha=0.45, label='Ref.')
    for k, a in res.items():
        ax.plot(a['p'][:, 1], a['p'][:, 0], color=COLORS[k], lw=0.8, label=k)
    ax.plot(0, 0, 's', color=USV, ms=5)
    ax.annotate('USV', (0, 0), xytext=(6, 0), textcoords='offset points', va='center', fontsize=9)
    ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('$y$ (m)'); ax.set_ylabel('$x$ (m)')
    # The legend, also of the x, y, z panels, to the right of the crescent
    leg = ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5), fontsize=7, frameon=False)
    for h in leg.get_lines()[1:]:
        h.set_linewidth(1.5)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_track_top'))

    # --- Commands over the middle lap: tilt and thrust sent to the autopilot ----------------------------------------
    # Both are low-pass filtered (zero phase, CMD_CUTOFF), the same for both controllers, to remove the vibration of
    # the simulated tether (~1.5 Hz, on the thrust of both) and the sample-to-sample oscillation of the PX4 output
    # (50 Hz), which hide the commands that the trajectory requires (below ~0.7 Hz)
    def lowpass(t, x):
        fs = 1.0 / np.median(np.diff(t))
        tu = np.arange(t[0], t[-1], 1.0 / fs)
        b, a_ = butter(2, CMD_CUTOFF / (fs / 2))
        return tu, filtfilt(b, a_, np.interp(tu, t, x))

    for name, key, ylabel in (('crescent_cmd_tilt', 'tilt_cmd', 'Commanded tilt (deg)'),
                              ('crescent_cmd_thrust', 'thrust_cmd', 'Commanded thrust (N)')):
        fig, ax = panel()
        for k, a in res.items():
            tf, xf = lowpass(a['t_cmd'], a[key])
            mid = (tf >= period) & (tf <= 2 * period)
            ax.plot(tf[mid] - period, xf[mid], color=COLORS[k], lw=0.9, label=k)
        ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel); ax.set_xlim(0, period)
        ax.legend(loc='best', fontsize=7)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))

    # --- Error over the three laps, with the reference speed -------------------------------------------------------
    fig, ax = plt.subplots(figsize=(6.3 * K, 2.4 * K))
    ax.grid(True, alpha=0.3)
    for k, a in res.items():
        ax.plot(a['t'], 100 * a['n'], color=COLORS[k], lw=0.9, label=k)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Position error (cm)'); ax.set_xlim(0, LAPS * period); ax.set_ylim(bottom=0)
    ax2 = ax.twinx()
    ax2.fill_between(res['NMPC']['t'], res['NMPC']['sp_ref'], color=REF, alpha=0.15, lw=0)
    ax2.set_ylabel('Reference speed (m/s)'); ax2.set_ylim(0, 12)
    ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
    ax.legend(loc='upper left', fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_error'))

    # --- The tether: separation and its rate of change of both runs against the limits of the tether ----------------
    def limits_panel(name, ylabel, series, limits):
        fig, ax = panel()
        for k, (tt, yy) in series.items():
            ax.plot(tt, yy, color=COLORS[k], lw=0.9, label=k)
        for y in limits:
            ax.axhline(y, color=LIMIT, ls='--', lw=0.9)
        ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel); ax.set_xlim(0, LAPS * period)
        ax.legend(loc='best', fontsize=7)
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, name))

    limits_panel('crescent_distance', 'Separation $d$ (m)', {k: (a['t'], a['dist']) for k, a in res.items()},
                 [D_MAX])
    limits_panel('crescent_ddot', r'$\dot d$ (m/s)', {k: (a['t'], a['d_dot']) for k, a in res.items()},
                 [UAV_LIM['d_dot_max'], -UAV_LIM['d_dot_max']])

if __name__ == '__main__':
    main()
