#!/usr/bin/env python3
"""
Section 5.2: one figure per mission, drawn from the extracted mission bag (record_mission.sh, extract.py): a top view
of both vehicles, with their positions and the tether at a few instants, and a figure with the manoeuvres of each vehicle over
time, the separation against its limit and the height of the UAV. Prints the metrics of the mission.
Usage: plot_mission.py <name> [d_max=15] [usv_margin=0] (the tether bound of the USV is d_max + usv_margin)
"""
import os
import re
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from common import load, interp, set_thesis_style, save_figure, FIGS, UAV, USV, TGT, LIMIT

OUT = os.path.join(FIGS, '02_missions')
Z_ANCHOR = 1.25
NAMES = {'takeoff': 'Takeoff', 'formation': 'Formation', 'scout': 'Formation', 'watch': 'Formation',
         'orbit': 'Orbit', 'chase': 'Follow', 'land': 'Landing', 'wait': 'Hold', 'route': 'Route', 'stop': 'Hold',
         'support': 'Support', 'pursuit': 'Follow', 'held': 'Formation', 'stay': 'Formation', 'return': 'Return',
         'lap1': 'Lap 1', 'lap2': 'Lap 2', 'lap3': 'Lap 3'}
SHADES = ['#dce6f2', '#b9cde6', '#94b3d9', '#6f99cc', '#4a7fbf']


def phases(d, vehicle, t_first):
    """[(name, t_start, t_end)] of the manoeuvres of one vehicle, from the mission events (a step that started
    before the events were recorded starts at t_first)."""
    out, cur = [], None
    for t, e in zip(d['ev_t'], d['ev']):
        m = re.match(r'%s\.(\w+): (started|done)' % vehicle, e)
        if not m:
            continue
        if m.group(2) == 'started':
            cur = (m.group(1), t)
        elif cur and cur[0] == m.group(1):
            out.append((NAMES.get(cur[0], cur[0]), cur[1], t)); cur = None
        elif not out:   # the first step started before the events were recorded
            out.append((NAMES.get(m.group(1), m.group(1)), t_first, t))
    if cur:
        out.append((NAMES.get(cur[0], cur[0]), cur[1], d['drone'][-1, 0]))
    merged = []   # consecutive steps with the same name (e.g. scout + watch) are one manoeuvre
    for p in out:
        if merged and merged[-1][0] == p[0] and abs(merged[-1][2] - p[1]) < 0.5:
            merged[-1] = (p[0], merged[-1][1], p[2])
        elif p[2] - p[1] > 0.3:
            merged.append(p)
    return merged


def boat(ax, x, y, yaw, color=USV, alpha=0.9, L=4.9, W=2.4):
    hull = np.array([[-L / 2, -W / 2], [L / 2 - 0.8, -W / 2], [L / 2, 0], [L / 2 - 0.8, W / 2], [-L / 2, W / 2]])
    c, s = np.cos(yaw), np.sin(yaw)
    ax.add_patch(Polygon(hull @ np.array([[c, s], [-s, c]]) + [x, y], closed=True, fc=color, ec='k', lw=0.4,
                         alpha=alpha, zorder=3))


FIG_NAMES = {'wait': 'recovery'}   # wait.yaml is the recovery mission of the thesis


def main(name, d_max, usv_margin=0.0):
    set_thesis_style()
    d = load(name)
    dr, b, it = d['drone'], d['boat'], d['intruder']
    # start of the mission: the first event, or the liftoff of the UAV if the first events were not recorded
    z = dr[:, 3]; t_lift = dr[np.argmax(z > z[:50].mean() + 0.3), 0] - 1.0
    t0 = min(d['ev_t'][0], t_lift)
    t_end = next((t for t, e in zip(d['ev_t'], d['ev']) if 'mission complete' in e or 'abort sequence' in e), dr[-1, 0])
    pu, pv = phases(d, 'uav', t0), phases(d, 'usv', t0)
    for t, e in zip(d['ev_t'], d['ev']):
        if not re.search(r': (started|done)$|started \(', e):
            print('  %6.1f  %s' % (t - t0, e))
    m = (dr[:, 0] >= t0) & (dr[:, 0] <= t_end)
    mb = (b[:, 0] >= t0) & (b[:, 0] <= t_end)
    sep = np.linalg.norm(dr[:, 1:4] - np.column_stack([interp(b, dr[:, 0], (1, 2)), np.full(len(dr), Z_ANCHOR)]), axis=1)
    air = m & (dr[:, 3] > 1.5)
    f = d['s_tether_force_drone_mag']; mf = (f[:, 0] >= t0) & (f[:, 0] <= t_end)
    v = d['s_virtual_tether_separation_speed']; mv = (v[:, 0] >= t0) & (v[:, 0] <= t_end)
    tel = d['tel']; mt = (tel['t'] >= t0) & (tel['t'] <= t_end) & (np.interp(tel['t'], dr[:, 0], dr[:, 3]) > 1.5)
    s = d['s_uav_solve_time_ms']; ms = (s[:, 0] >= t0) & (s[:, 0] <= t_end)
    print('%s: duration %.0f s | separation max %.2f m (d_max %.0f) | separation speed max %.2f m/s | tension max %.2f N'
          % (name, t_end - t0, sep[air].max(), d_max, v[mv, 1].max() if mv.any() else np.nan, f[mf, 1].max()))
    print('  UAV |roll| max %.1f |pitch| max %.1f deg | horizontal speed max %.2f m/s | UAV solve max %.1f ms' % (
        np.degrees(np.abs(tel['roll'][mt]).max()), np.degrees(np.abs(tel['pitch'][mt]).max()),
        np.hypot(dr[m, 5], dr[m, 6]).max(), s[ms, 1].max()))
    print('  UAV phases:', [(p[0], round(p[1] - t0, 1), round(p[2] - t0, 1)) for p in pu])
    print('  USV phases:', [(p[0], round(p[1] - t0, 1), round(p[2] - t0, 1)) for p in pv])

    # top view and time histories as two figures; missions much wider than tall are placed one above the other in the
    # thesis, the others side by side
    xs = np.concatenate([dr[m, 1], b[mb, 1]]); ys = np.concatenate([dr[m, 2], b[mb, 2]])
    wide = np.ptp(xs) > 2.0 * np.ptp(ys)
    fig, ax = plt.subplots(figsize=(6.3, min(3.2, 0.9 + 5.4 * np.ptp(ys) / np.ptp(xs))) if wide else (3.0, 3.2))
    if name.startswith('inspection'):
        ax.add_patch(Rectangle((-25 - 5.9, -6.7), 11.8, 13.4, fc='#bbbbbb', ec='#555555', lw=0.6, zorder=1))
        ax.annotate('tower', (-25, 0), ha='center', va='center', fontsize=7, zorder=2)
    if len(it):
        mi = (it[:, 0] >= t0) & (it[:, 0] <= t_end)
        ax.plot(it[mi, 1], it[mi, 2], color=TGT, lw=1.0, label='target')
    ax.plot(b[mb, 1], b[mb, 2], color=USV, lw=1.2, label='USV')
    ax.plot(dr[m, 1], dr[m, 2], color=UAV, lw=0.9, label='UAV')
    snaps = np.linspace(t0 + 0.1 * (t_end - t0), t0 + 0.9 * (t_end - t0), 5)
    for k, tk in enumerate(snaps):
        a = 0.35 + 0.65 * k / (len(snaps) - 1)
        pb, pd = interp(b, [tk], (1, 2, 4))[0], interp(dr, [tk], (1, 2))[0]
        boat(ax, pb[0], pb[1], pb[2], alpha=a)
        ax.plot([pb[0], pd[0]], [pb[1], pd[1]], color='#777777', lw=0.6, alpha=a, zorder=4)
        ax.plot(pd[0], pd[1], 'o', ms=3.5, mfc=UAV, mec='k', mew=0.4, alpha=a, zorder=5)
        if len(it):
            pt = interp(it, [tk], (1, 2, 4))[0]; boat(ax, pt[0], pt[1], pt[2], color=TGT, alpha=a)
    ax.set_aspect('equal', adjustable='box'); ax.margins(0.06); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)')
    h = [plt.Line2D([], [], color=UAV, lw=1.0), plt.Line2D([], [], color=USV, lw=1.2)]
    lab = ['UAV', 'USV']
    if len(it):
        h.append(plt.Line2D([], [], color=TGT, lw=1.0)); lab.append('target')
    ax.legend(h, lab, loc='lower right' if name.startswith('inspection') else 'best', fontsize=7)

    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, FIG_NAMES.get(name, name) + '_paths'))

    fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(6.3, 2.6) if wide else (3.3, 3.2), sharex=True,
                                     gridspec_kw={'height_ratios': [0.45, 0.8, 0.8] if wide else [0.8, 1.0, 1.0]})
    for row, (ph, label) in enumerate(((pu, 'UAV'), (pv, 'USV'))):
        y = 1 - row
        for k, (n, ts, te) in enumerate(ph):
            a0.add_patch(Rectangle((ts - t0, y - 0.4), te - ts, 0.8, fc=SHADES[k % len(SHADES)], ec='white', lw=0.8))
            if te - ts > 0.016 * len(n) * (t_end - t0):   # only where the name fits
                a0.text(0.5 * (ts + te) - t0, y, n, ha='center', va='center', fontsize=6.5)
    a0.set_yticks([1, 0]); a0.set_yticklabels(['UAV', 'USV']); a0.set_ylim(-0.6, 1.6); a0.grid(False)
    a1.plot(dr[m, 0] - t0, sep[m], color='#333333', lw=0.9)
    a1.axhline(d_max, color=LIMIT, ls='--', lw=0.9, label='$d_{max}$')
    if usv_margin > 0.0:
        a1.axhline(d_max + usv_margin, color=LIMIT, ls=':', lw=0.9, label=r'$d_{max}+\delta_d$')
    a1.set_ylabel('$d$ (m)'); a1.legend(loc='lower right', fontsize=7); a1.set_ylim(0, 1.12 * d_max)
    a2.plot(dr[m, 0] - t0, dr[m, 3], color=UAV, lw=0.9)
    a2.set_ylabel('UAV $z$ (m)'); a2.set_xlabel('time (s)')
    a0.set_xlim(0, t_end - t0)
    fig.align_ylabels([a0, a1, a2])
    fig.tight_layout(h_pad=0.4)
    save_figure(fig, os.path.join(OUT, FIG_NAMES.get(name, name) + '_timeline'))


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 15.0, float(sys.argv[3]) if len(sys.argv) > 3 else 0.0)
