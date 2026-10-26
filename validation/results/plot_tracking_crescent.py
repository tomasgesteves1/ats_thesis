#!/usr/bin/env python3
"""
Section 5.1: accuracy of the UAV on the crescent, with the NMPC and with the PX4 position controller (crescent scenario of
record_tracking.sh). Data: data/tracking_crescent_{nmpc,px4}.npz (extract.py). The error is measured from the ground
truth against the published reference at the same simulation instant, over the last three complete laps of each run.
Prints the metrics of the section and writes one figure per panel to Results/figures/01_tracking.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, rms, set_thesis_style, save_figure, FIGS, HERE, UAV, USV, PX4, REF, LIMIT, UAV_LIM

TABLE = os.path.join(HERE, '..', '..', 'mission', 'test_uav_circle', 'config', 'crescent.csv')
OUT = os.path.join(FIGS, '01_tracking')
Z_WINCH = 1.3        # m, height of the winch
C_D = 0.2065         # N/m, tension of the tether per metre of separation (Section 4.2)
LAPS = 3
SIZE = (3.1, 2.4)    # one panel; two side by side in the thesis
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
    return dict(t0=t0, t=t - t0, p=p, r=r, e=e, n=np.linalg.norm(e, axis=1), e_al=e_al, e_cr=e_cr,
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

    # --- Scenario: the crescent seen from above, coloured by the speed of the reference, and the tether over a lap ---
    fig, ax = plt.subplots(figsize=(3.1, 3.4))
    sp = np.hypot(tab[:, 4], tab[:, 5])
    sc = ax.scatter(tab[:, 1], tab[:, 2], c=sp, s=1.5, cmap='viridis', rasterized=True)
    for k in np.linspace(0, len(tab), 9, endpoint=False).astype(int):
        j = (k + 30) % len(tab)
        ax.annotate('', xy=tab[j, 1:3], xytext=tab[k, 1:3], arrowprops=dict(arrowstyle='->', color='k', lw=0.7))
    ax.plot(0, 0, 's', color=USV, ms=6, label='USV')
    fig.colorbar(sc, ax=ax, label='Reference speed (m/s)', shrink=0.85)
    ax.set_aspect('equal'); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)')
    ax.legend(loc='center', bbox_to_anchor=(0.32, 0.5), fontsize=7)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_top'))

    one = res['NMPC']['t'] <= period
    fig, ax = panel()
    ax.plot(res['NMPC']['t'][one], res['NMPC']['dist'][one], color=UAV)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Separation $d$ (m)'); ax.set_xlim(0, period)
    ax2 = ax.twinx()
    ax2.set_ylim(np.array(ax.get_ylim()) * C_D); ax2.set_ylabel('Tension (N)')
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_separation'))

    # --- Overlaid tracking: x, y, z against the reference over the middle lap, and the paths seen from above ---------
    for c, lab in enumerate(('x', 'y', 'z')):
        fig, ax = plt.subplots(figsize=(2.05, 1.75))   # three side by side in the thesis
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

    # Same height as the other panels, narrower (equal axes)
    fig, ax = plt.subplots(figsize=(2.7, SIZE[1]))
    ax.grid(True, alpha=0.3)
    ax.plot(tab[:, 1], tab[:, 2], color=REF, lw=2.5, alpha=0.45, label='Ref.')
    for k, a in res.items():
        ax.plot(a['p'][:, 0], a['p'][:, 1], color=COLORS[k], lw=0.8, label=k)
    ax.plot(0, 0, 's', color=USV, ms=5)
    ax.set_aspect('equal'); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)')
    # The legend, also of the x, y, z panels, to the right of the crescent
    leg = ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=7, frameon=False)
    for h in leg.get_lines()[1:]:
        h.set_linewidth(1.5)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'crescent_track_top'))

    # --- Error over the three laps, with the reference speed -------------------------------------------------------
    fig, ax = plt.subplots(figsize=(6.3, 2.4))
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

    # --- Feasibility: speed, separation rate, tilt and thrust of both runs against the limits -----------------------
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

    limits_panel('crescent_speed', 'Horizontal speed (m/s)', {k: (a['t'], a['sp']) for k, a in res.items()},
                 [UAV_LIM['v_max']])
    limits_panel('crescent_ddot', r'$\dot d$ (m/s)', {k: (a['t'], a['d_dot']) for k, a in res.items()},
                 [UAV_LIM['d_dot_max'], -UAV_LIM['d_dot_max']])
    limits_panel('crescent_tilt', 'Tilt (deg)', {k: (a['t_att'], a['tilt']) for k, a in res.items()},
                 [np.degrees(UAV_LIM['phi_max'])])
    a = res['NMPC']
    limits_panel('crescent_thrust', 'Thrust (N)', {'NMPC': (tel['t'][mt] - a['t0'], tel['thrust'][mt])},
                 [UAV_LIM['T_min'], UAV_LIM['T_max']])


if __name__ == '__main__':
    main()
