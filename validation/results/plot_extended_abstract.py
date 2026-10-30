#!/usr/bin/env python3
"""
Figures of the tracking results of the extended abstract (latex/extended_abstract/figures), drawn at the width of a
column in its typeface (set_ea_style), from the same data and analysis as Section 5.1 of the thesis:
  ea_uav_accuracy  - position error of the NMPC and of PX4 over three laps of the crescent, with the reference speed;
  ea_uav_limits    - tilt, horizontal speed and height of the UAV in the limits test, against their limits;
  ea_usv_limits    - thrust and azimuth of both thrusters of the USV in its limits test, against their limits.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, EA_FIGS, UAV, USV, PX4, REF, LIMIT, UAV_LIM, USV_LIM
from thesis_style import EA_COLUMN_WIDTH, set_ea_style, normalise_labels
import plot_tracking_crescent as C
import plot_tracking_limits as L
import plot_usv_limits as U

MM = 1 / 25.4


def save(fig, name):
    os.makedirs(EA_FIGS, exist_ok=True)
    path = os.path.join(EA_FIGS, name + '.pdf')
    normalise_labels(fig)
    fig.savefig(path)              # drawn at its printed size: included without scaling
    plt.close(fig)
    print('saved', path)


def style_axis(ax):
    ax.grid(True, alpha=0.35)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)


def uav_accuracy():
    tab = np.loadtxt(C.TABLE, delimiter=',', skiprows=1)
    period = tab[-1, 0] + (tab[1, 0] - tab[0, 0])
    res = {k: C.analyse(load('tracking_crescent_' + v), tab) for k, v in C.NAMES.items()}
    fig, ax = plt.subplots(figsize=(EA_COLUMN_WIDTH, 40 * MM), layout='constrained')
    ax2 = ax.twinx()
    ax2.fill_between(res['NMPC']['t'], res['NMPC']['sp_ref'], color=REF, alpha=0.15, lw=0)
    ax2.set_ylim(0, 12); ax2.set_ylabel('Reference speed (m/s)', color='#777777')
    ax2.tick_params(axis='y', colors='#777777'); ax2.spines['top'].set_visible(False)
    for k in ('PX4', 'NMPC'):
        a = res[k]
        ax.plot(a['t'], 100 * a['n'], color=C.COLORS[k], label=k)
    ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
    ax.grid(True, alpha=0.35); ax.spines['top'].set_visible(False)
    ax.set_xlim(0, C.LAPS * period); ax.set_ylim(0, 60)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Position error (cm)')
    ax.legend(loc='upper left', ncol=2, frameon=False)
    save(fig, 'ea_uav_accuracy')


def uav_limits():
    tab = np.loadtxt(L.TABLE, delimiter=',', skiprows=1)
    d = load('tracking_limits_nmpc')
    ref, dr, tel = d['uav_ref'], d['drone'], d['tel']
    on_first = np.linalg.norm(ref[:, 1:4] - tab[0, 1:4], axis=1) < 1e-4
    idx = np.where(on_first & (ref[:, 0] > dr[0, 0] + 5.0))[0]
    cut = np.where(np.diff(idx) > 1)[0]
    marks, t_end = L.phase_marks()
    t0 = ref[idx[cut[0]] if len(cut) else idx[-1], 0] - marks[1][1]
    m = (dr[:, 0] >= t0) & (dr[:, 0] <= t0 + t_end)
    t = dr[m, 0] - t0
    v = np.gradient(dr[m, 1:4], dr[m, 0], axis=0)
    mt = (tel['t'] >= t0) & (tel['t'] <= t0 + t_end)
    tilt = np.degrees(np.arccos(np.cos(tel['roll'][mt]) * np.cos(tel['pitch'][mt])))

    fig, axs = plt.subplots(3, 1, figsize=(EA_COLUMN_WIDTH, 72 * MM), sharex=True, layout='constrained')
    rows = [(tel['t'][mt] - t0, tilt, 'Tilt (deg)', [np.degrees(UAV_LIM['phi_max'])], (0, 32)),
            (t, np.hypot(v[:, 0], v[:, 1]), 'Hor. speed (m/s)', [UAV_LIM['v_max']], (0, 11.5)),
            (t, dr[m, 3], 'Height (m)', [L.Z_MIN], (0, 11))]
    spans = [('Arc A', 10.0, 16.0), ('Arc B', 47.3, 56.8), ('Steps', 71.5, t_end)]
    for ax, (tx, y, label, lims, ylim) in zip(axs, rows):
        style_axis(ax)
        for _, x0, x1 in spans:
            ax.axvspan(x0, x1, color=REF, alpha=0.10, lw=0)
        ax.plot(tx, y, color=UAV)
        for lim in lims:
            ax.axhline(lim, color=LIMIT, ls='--', lw=0.8)
        ax.set_ylabel(label); ax.set_ylim(*ylim)
    for txt, x0, x1 in spans:
        axs[0].text((x0 + x1) / 2, 1.02, txt, transform=axs[0].get_xaxis_transform(), ha='center', va='bottom',
                    fontsize=7, color='#555555')
    axs[-1].set_xlabel('Time (s)'); axs[-1].set_xlim(0, t_end)
    save(fig, 'ea_uav_limits')


def usv_limits():
    d = load('tracking_usv_limits_nmpc')
    ref = d['usv_ref']
    ref = ref[ref[:, 0] > 1.0]
    rv = np.hypot(*np.gradient(ref[:, 1:3], ref[:, 0], axis=0).T)
    t0 = ref[np.flatnonzero(rv > 0.05)[0], 0]
    fig, axs = plt.subplots(2, 1, figsize=(EA_COLUMN_WIDTH, 52 * MM), sharex=True, layout='constrained')
    for side, col in (('left', USV), ('right', REF)):
        for ax, sig, scale in ((axs[0], 'thrust', 1.0), (axs[1], 'pos', 180 / np.pi)):
            s = d['s_boat_thrusters_%s_%s' % (side, sig)]
            m = (s[:, 0] >= t0) & (s[:, 0] <= t0 + U.END)
            ax.plot(s[m, 0] - t0, scale * s[m, 1], color=col, label=side.capitalize())
    for ax, lims, label in ((axs[0], (USV_LIM['T_min'], USV_LIM['T_max']), 'Thrust (N)'),
                            (axs[1], (-30.0, 30.0), 'Azimuth (deg)')):
        style_axis(ax)
        for _, ts in U.PHASES[1:]:
            ax.axvline(ts, color=REF, lw=0.4, alpha=0.4)
        for lim in lims:
            ax.axhline(lim, color=LIMIT, ls='--', lw=0.8)
        ax.set_ylabel(label)
    for name, ts in U.PHASES:
        axs[0].annotate(name.capitalize(), (ts + 1.0, 560), fontsize=7, color='#555555', va='bottom')
    axs[0].set_ylim(-450, 650)
    axs[1].set_ylim(-36, 36)
    axs[1].legend(loc='upper right', frameon=False)
    axs[-1].set_xlabel('Time (s)'); axs[-1].set_xlim(0, U.END)
    save(fig, 'ea_usv_limits')


def main():
    set_ea_style()
    uav_accuracy()
    uav_limits()
    usv_limits()


if __name__ == '__main__':
    main()
