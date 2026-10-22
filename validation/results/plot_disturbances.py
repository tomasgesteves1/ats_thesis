#!/usr/bin/env python3
"""
Section 5.3: landing on the moving boat (landing_demo mission) under waves (gain of the wavefield 0, 1, 2) and under
wind on the UAV (5, 8, 12 m/s) with and without the disturbance observer. Data: data/landing_*.npz (extract.py).
Prints the metrics and writes the figures to Results/figures/03_disturbances.
"""
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, rms, set_thesis_style, save_figure, FIGS, UAV, USV, LIMIT, COLOR_CYCLE

OUT = os.path.join(FIGS, '03_disturbances')
ANCHOR = 1.3        # height of the winch above the origin of the boat (m)
GAINS = (0, 1, 2)
WINDS = (5, 8, 12)


def ev_time(d, text):
    return next((t for t, e in zip(d['ev_t'], d['ev']) if text in e), None)


def winch(d, t):
    """Position of the winch at the times t, from the pose of the boat (the winch is ANCHOR above its origin)."""
    b = d['boat']
    x, y, z, yaw, roll, pitch = (np.interp(t, b[:, 0], b[:, k]) for k in (1, 2, 3, 4, 8, 9))
    # offset (0, 0, ANCHOR) in the boat axes, rotated by roll, pitch, yaw
    ox = ANCHOR * (np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll))
    oy = ANCHOR * (np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll))
    oz = ANCHOR * np.cos(pitch) * np.cos(roll)
    return np.column_stack([x + ox, y + oy, z + oz])


def landing(d):
    """Metrics of the landing: heave of the winch, offset when the descent starts, touchdown offset and speed."""
    t_land = ev_time(d, 'uav.land: started')
    t_desc = ev_time(d, 'descending with the landing pull')
    t_td = ev_time(d, 'touchdown')
    td_text = next(e for e in d['ev'] if 'touchdown' in e)
    offset = float(re.findall(r'[-0-9.]+$', td_text)[0])
    dr = d['drone']
    m = (dr[:, 0] >= t_land) & (dr[:, 0] <= t_td + 3.0)
    t = dr[m, 0]; w = winch(d, t)
    rel = dr[m, 1:4] - w
    w_all = winch(d, d['boat'][:, 0])
    mb = (d['boat'][:, 0] >= t_land) & (d['boat'][:, 0] <= t_td)
    heave = w_all[mb, 2]
    i_desc = np.searchsorted(t, t_desc)
    # contact speed: largest approach speed of the UAV relative to the winch over the 2 s before the touchdown is
    # declared (it is declared after the UAV has rested on the pad for a moment)
    dz = np.gradient(rel[:, 2], t)
    mc = (t >= t_td - 2.0) & (t <= t_td)
    vrel = -dz[mc].min()
    return dict(t=t - t_land, rel=rel, wz=w[:, 2], t_desc=t_desc - t_land, t_td=t_td - t_land, offset=offset,
                heave=0.5 * (heave.max() - heave.min()), heave_std=heave.std(),
                centring=np.hypot(*rel[i_desc, :2]), contact=vrel, duration=t_td - t_land)


def hover(d):
    """Horizontal error of the UAV to its reference over the last seconds of the transfer away from the boat, and
    observer estimate against the applied force over the whole flight."""
    t_away = ev_time(d, 'uav.away: started'); t_land = ev_time(d, 'uav.land: started')
    dr, ref = d['drone'], d['uav_ref']
    m = (dr[:, 0] >= t_land - 4.0) & (dr[:, 0] <= t_land)
    e = dr[m, 1:3] - interp(ref, dr[m, 0], (1, 2))
    return np.linalg.norm(e, axis=1).mean(), t_away, t_land


def main(part):
    set_thesis_style()
    if part in ('waves', 'all'):
        waves()
    if part in ('wind', 'all'):
        wind()


def waves():
    runs = {g: landing(load('landing_gain%d' % g)) for g in GAINS}
    print('=== waves ===')
    for g, r in runs.items():
        print('gain %d: heave amplitude %.2f m (std %.2f) | landing %.1f s | offset at descent %.2f m | touchdown offset '
              '%.2f m | contact speed %.2f m/s' % (g, r['heave'], r['heave_std'], r['duration'], r['centring'],
                                                  r['offset'], r['contact']))
    fig, axs = plt.subplots(3, 1, figsize=(6.3, 4.2), sharex=True)
    for k, g in enumerate(GAINS):
        r = runs[g]; c = COLOR_CYCLE[k]; tt = r['t'] - r['t_td']
        axs[0].plot(tt, r['wz'], color=c, lw=0.9, label='gain %d' % g)
        axs[1].plot(tt, r['rel'][:, 2], color=c, lw=0.9)
        axs[2].plot(tt, 100 * np.hypot(r['rel'][:, 0], r['rel'][:, 1]), color=c, lw=0.9)
    t_from = max(runs[g]['t_desc'] - runs[g]['t_td'] for g in GAINS) - 2.0
    axs[0].set_ylabel('winch\nheight (m)'); axs[1].set_ylabel('height above\nthe winch (m)')
    axs[2].set_ylabel('horizontal\noffset (cm)'); axs[2].set_xlabel('time to touchdown (s)')
    axs[0].legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=3, fontsize=7, frameon=False, borderaxespad=0.1)
    axs[2].set_xlim(t_from, 3); axs[2].set_ylim(0, 15)
    fig.align_ylabels(axs); fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'waves_landing'))


def wind():
    print('=== wind ===')
    table = {}
    for w in WINDS:
        for obs in ('obs', 'noobs'):
            d = load('landing_wind%d_%s' % (w, obs))
            r = landing(d); h, _, _ = hover(d)
            table[(w, obs)] = (h, r['centring'], r['offset'])
            print('wind %2d m/s %-5s: hover error %.2f m | offset at descent %.2f m | touchdown offset %.2f m' % (
                w, obs, h, r['centring'], r['offset']))
    d = load('landing_wind12_obs')
    ext, dist = d['ext'], d['dist']
    t_to = ev_time(d, 'uav.takeoff: done'); t_td = ev_time(d, 'touchdown')
    fig, axs = plt.subplots(1, 2, figsize=(6.3, 2.6), gridspec_kw={'width_ratios': [1.0, 1.3]})
    x = np.arange(len(WINDS))
    for k, (obs, lab) in enumerate((('noobs', 'without observer'), ('obs', 'with observer'))):
        axs[0].bar(x + (k - 0.5) * 0.36, [100 * table[(w, obs)][1] for w in WINDS], 0.36,
                   color=[COLOR_CYCLE[1], COLOR_CYCLE[0]][k], label=lab)
    axs[0].set_xticks(x); axs[0].set_xticklabels(['%d' % w for w in WINDS]); axs[0].set_xlabel('wind (m/s)')
    axs[0].set_ylabel('offset above the pad (cm)'); axs[0].legend(fontsize=7, loc='upper left')
    axs[0].set_title('(a)', fontsize=9, y=-0.42)
    me = (ext[:, 0] >= t_to) & (ext[:, 0] <= t_td); md = (dist[:, 0] >= t_to) & (dist[:, 0] <= t_td)
    t0 = t_to
    axs[1].plot(ext[me, 0] - t0, np.hypot(ext[me, 1], ext[me, 2]), color='#333333', lw=0.9, label='applied')
    axs[1].plot(dist[md, 0] - t0, np.hypot(dist[md, 1], dist[md, 2]), color=UAV, lw=0.9, label='estimated')
    axs[1].set_xlabel('time (s)'); axs[1].set_ylabel('horizontal force (N)'); axs[1].legend(fontsize=7)
    axs[1].set_title('(b)', fontsize=9, y=-0.42)
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'wind'))


if __name__ == '__main__':
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')
