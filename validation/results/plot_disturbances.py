#!/usr/bin/env python3
"""
Section 5.3: landing on the moving boat (landing_demo mission) under waves (gain of the wavefield 0, 1, 2) and under
wind on the UAV (5, 8 m/s; the 12 m/s runs are not used) with and without the disturbance observer. Data: data/landing_*.npz (extract.py).
Prints the metrics and writes the figures to Results/figures/03_disturbances.
"""
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from common import load, interp, rms, legend_above, set_thesis_style, save_figure, fig_size, FIGS, EA_FIGS, UAV, USV, LIMIT, COLOR_CYCLE

OUT = os.path.join(FIGS, '03_disturbances')
ANCHOR = 1.3        # height of the winch above the origin of the boat (m)
GAINS = (0, 1, 2)
WINDS = (5, 8)
WIND_FIG = 8   # wind of the landing figure (the strongest in the thesis)
# runs repeated with the final controller (tilt limits also on the command)
RUN = {(12, 'obs'): 'landing_wind12_obs_rerun', (12, 'noobs'): 'landing_wind12_noobs_rerun'}


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
    t_desc = ev_time(d, 'descending')   # centred, or the centring timed out
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


def centring(d):
    """Horizontal offset of the UAV from the winch when the descent starts (also when the landing then fails)."""
    t_desc = ev_time(d, 'descending')
    w = winch(d, [t_desc])[0]
    p = interp(d['drone'], [t_desc], (1, 2, 3))[0]
    return float(np.hypot(*(p - w)[:2]))


def hover(d):
    """Horizontal error of the UAV to its reference over the last seconds of the transfer away from the boat, and
    observer estimate against the applied force over the whole flight."""
    t_away = ev_time(d, 'uav.away: started'); t_land = ev_time(d, 'uav.land: started')
    dr, ref = d['drone'], d['uav_ref']
    m = (dr[:, 0] >= t_land - 4.0) & (dr[:, 0] <= t_land)
    e = dr[m, 1:3] - interp(ref, dr[m, 0], (1, 2))
    return np.linalg.norm(e, axis=1).mean(), t_away, t_land


PHASE_SHADES = ['#eef3f9', '#dce6f2', '#c6d7ec', '#b0c8e5']
DESCENT_DURATION = 12.0   # s, duration of the descent of the landing manoeuvre (landing_demo.yaml)


def nominal():
    """Landing without waves and without wind: height of the UAV above the winch and horizontal offset from it, with
    the four phases of the manoeuvre (Section 4.4) shaded."""
    d = load('landing_gain0')
    r = landing(d)
    t_land = ev_time(d, 'uav.land: started')
    ref = d['uav_ref']; m = (ref[:, 0] >= t_land) & (ref[:, 0] <= t_land + r['t_td'])
    w = winch(d, ref[m, 0])
    above = np.hypot(*(ref[m, 1:3] - w[:, :2]).T) < 0.05   # the reference is above the winch
    t_above = ref[m, 0][np.argmax(above)] - t_land
    t_final = r['t_desc'] + DESCENT_DURATION
    bounds = [0.0, t_above, r['t_desc'], t_final, r['t_td']]
    print('=== nominal ===\nphases (s since the landing started):', np.round(bounds, 1))
    print('landing %.1f s | offset at descent %.1f cm | touchdown offset %.1f cm | contact speed %.2f m/s'
          % (r['duration'], 100 * r['centring'], 100 * r['offset'], r['contact']))
    fig, axs = plt.subplots(2, 1, figsize=fig_size('landing_nominal'), sharex=True)
    for a in axs:
        for k in range(4):
            a.axvspan(bounds[k], bounds[k + 1], color=PHASE_SHADES[k], lw=0)
    for k in range(4):
        axs[0].text(0.5 * (bounds[k] + bounds[k + 1]), 1.02, str(k + 1), transform=axs[0].get_xaxis_transform(),
                    ha='center', va='bottom', fontsize=7)
    axs[0].plot(r['t'], r['rel'][:, 2], color=UAV, lw=0.9)
    axs[1].plot(r['t'], 100 * np.hypot(r['rel'][:, 0], r['rel'][:, 1]), color=UAV, lw=0.9)
    axs[0].set_ylabel('Height above\nthe winch (m)'); axs[0].set_ylim(0, None)
    axs[1].set_ylabel('Horizontal\noffset (cm)'); axs[1].set_ylim(0, 30)
    axs[1].set_xlabel('Time since the landing started (s)'); axs[1].set_xlim(0, r['t_td'] + 3.0)
    fig.align_ylabels(axs); fig.tight_layout(h_pad=0.4)
    save_figure(fig, os.path.join(OUT, 'landing_nominal'))


def main(part):
    set_thesis_style()
    if part in ('nominal', 'all'):
        nominal()
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
    # thesis, and a column of the extended abstract
    for size, path in ((fig_size('waves_landing'), os.path.join(OUT, 'waves_landing')),
                       ((3.1, 3.6), os.path.join(EA_FIGS, 'waves_landing_column'))):
        fig, axs = plt.subplots(3, 1, figsize=size, sharex=True)
        for k, g in enumerate(GAINS):
            r = runs[g]; c = COLOR_CYCLE[k]; tt = r['t'] - r['t_td']
            axs[0].plot(tt, r['wz'], color=c, lw=0.9, label='gain %d' % g)
            axs[1].plot(tt, r['rel'][:, 2], color=c, lw=0.9)
            axs[2].plot(tt, 100 * np.hypot(r['rel'][:, 0], r['rel'][:, 1]), color=c, lw=0.9)
        t_from = max(runs[g]['t_desc'] - runs[g]['t_td'] for g in GAINS) - 2.0
        axs[0].set_ylabel('winch\nheight (m)'); axs[1].set_ylabel('height above\nthe winch (m)')
        axs[2].set_ylabel('horizontal\noffset (cm)'); axs[2].set_xlabel('time to touchdown (s)')
        leg = axs[0].legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=3, fontsize=7, frameon=False,
                            borderaxespad=0.1, handlelength=2.5)
        for line in leg.get_lines():
            line.set_linewidth(2.0)
        axs[2].set_xlim(t_from, 3); axs[2].set_ylim(0, 15)
        fig.align_ylabels(axs); fig.tight_layout()
        save_figure(fig, path)


def wind():
    print('=== wind ===')
    table = {}
    for w in WINDS:
        for obs in ('obs', 'noobs'):
            d = load(RUN.get((w, obs), 'landing_wind%d_%s' % (w, obs)))
            if ev_time(d, 'touchdown') is None:
                h, _, _ = hover(d)
                table[(w, obs)] = (h, centring(d), np.nan)
                print('wind %2d m/s %-5s: hover error %.2f m | offset at descent %.2f m | no touchdown' % (
                    w, obs, h, table[(w, obs)][1]))
                continue
            r = landing(d); h, _, _ = hover(d)
            table[(w, obs)] = (h, r['centring'], r['offset'])
            print('wind %2d m/s %-5s: hover error %.2f m | offset at descent %.2f m | touchdown offset %.2f m' % (
                w, obs, h, r['centring'], r['offset']))
    d = load(RUN.get((WIND_FIG, 'obs'), 'landing_wind%d_obs' % WIND_FIG))
    ext, dist = d['ext'], d['dist']
    t_to = ev_time(d, 'uav.takeoff: done'); t_td = ev_time(d, 'descending')   # until the winch starts to pull
    # offset and force as two figures, side by side in the thesis
    fig, ax = plt.subplots(figsize=fig_size('wind_offset'))
    axs = [ax]
    x = np.arange(len(WINDS))
    for k, (obs, lab) in enumerate((('noobs', 'without observer'), ('obs', 'with observer'))):
        axs[0].bar(x + (k - 0.5) * 0.36, [100 * table[(w, obs)][1] for w in WINDS], 0.36,
                   color=[COLOR_CYCLE[1], COLOR_CYCLE[0]][k], label=lab)
    for k, w in enumerate(WINDS):
        if np.isnan(table[(w, 'noobs')][2]):   # no touchdown: the bar is the offset when the descent started
            axs[0].text(k - 0.18, min(100 * table[(w, 'noobs')][1], 85) + 1.0, 'no\nlanding', ha='center',
                        va='bottom', fontsize=7, color=COLOR_CYCLE[1])
    axs[0].set_xticks(x); axs[0].set_xticklabels(['%d' % w for w in WINDS]); axs[0].set_xlabel('wind (m/s)')
    axs[0].set_ylabel('offset at the descent (cm)'); axs[0].set_ylim(0, 100)
    axs[0].legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'wind_offset'))

    # landing in the strongest wind, with and without the estimate: height above the winch and offset from it (same
    # order as the landing without waves and wind)
    fig, axs2 = plt.subplots(2, 1, figsize=fig_size('wind_landing'), sharex=True)
    for obs, c, lab in (('noobs', COLOR_CYCLE[1], 'without observer'), ('obs', COLOR_CYCLE[0], 'with observer')):
        dd = load(RUN.get((WIND_FIG, obs), 'landing_wind%d_%s' % (WIND_FIG, obs))); dr = dd['drone']
        tl = ev_time(dd, 'uav.land: started'); td = ev_time(dd, 'touchdown')
        te = td + 3.0 if td is not None else dr[-1, 0]
        m = (dr[:, 0] >= tl) & (dr[:, 0] <= te)
        t = dr[m, 0]; rel = dr[m, 1:4] - winch(dd, t)
        if td is None:   # no landing: the UAV falls off the deck; cut when it first drops below the winch
            below = (t > ev_time(dd, 'descending')) & (rel[:, 2] < 0.0)
            k = t <= t[np.argmax(below)]
            t, rel = t[k], rel[k]
        axs2[0].plot(t - tl, rel[:, 2], color=c, lw=0.9, label=lab)
        axs2[1].plot(t - tl, 100 * np.hypot(rel[:, 0], rel[:, 1]), color=c, lw=0.9)
        for a in axs2:
            a.axvline(ev_time(dd, 'descending') - tl, color=c, ls=':', lw=0.8)
    axs2[0].set_ylabel('height above\nthe winch (m)'); legend_above(axs2[0], ncol=2)
    axs2[1].set_ylim(0, 100); axs2[1].set_ylabel('horizontal\noffset (cm)'); axs2[1].set_xlabel('time since the landing started (s)')
    axs2[1].set_xlim(0, None)
    fig.align_ylabels(axs2); fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'wind_landing'))

    fig, ax = plt.subplots(figsize=fig_size('wind_force'))
    axs = [None, ax]
    me = (ext[:, 0] >= t_to) & (ext[:, 0] <= t_td); md = (dist[:, 0] >= t_to) & (dist[:, 0] <= t_td)
    t0 = t_to
    axs[1].plot(ext[me, 0] - t0, np.hypot(ext[me, 1], ext[me, 2]), color='#333333', lw=0.9, label='applied')
    axs[1].plot(dist[md, 0] - t0, np.hypot(dist[md, 1], dist[md, 2]), color=UAV, lw=0.9, label='estimated')
    axs[1].set_xlabel('time (s)'); axs[1].set_ylabel('horizontal force (N)'); axs[1].legend(fontsize=7, loc='upper center')
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'wind_force'))


if __name__ == '__main__':
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')
