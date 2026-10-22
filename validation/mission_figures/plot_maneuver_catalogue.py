#!/usr/bin/env python3
"""Catalogue of the manoeuvres (Section 4.4): one tile per manoeuvre, drawn from the mission bags in the world frame,
with the boat and the UAV at simultaneous instants joined by the tether. Data: data/<mission>.npz (extract_mission.py)."""
import os, sys, numpy as np, matplotlib.pyplot as plt
from matplotlib.patches import Polygon
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from thesis_style import set_thesis_style, COLOR_CYCLE
set_thesis_style()
OUT = os.path.join(HERE, '..', '..', 'latex', 'tese', 'Implementation', 'figures', '04_maneuvers_and_missions',
                   'maneuver_catalogue.pdf')
UAV, USV, TGT, TETHER = COLOR_CYCLE[0], COLOR_CYCLE[1], COLOR_CYCLE[3], '#777777'
WINCH_H = 1.3
DRONE_DZ = 0.265   # odometry origin to the point tracked by the controller

def load(n):
    d = dict(np.load(os.path.join(HERE, 'data', f'{n}.npz'))); d['d'][:, 3] += DRONE_DZ; ev = list(zip(d['ev_t'], d['ev']))
    return d['d'], d['b'], d['i'], ev
def tev(ev, s): return next(t for t, e in ev if s in e)
def win(a, t0, t1): return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]
def at(a, t): return np.array([np.interp(t, a[:, 0], a[:, c]) for c in range(1, 5)])

def boat(ax, x, y, yaw, color=USV, L=4.9, W=2.4, alpha=0.9):
    hull = np.array([[-L/2, -W/2], [L/2 - 0.8, -W/2], [L/2, 0], [L/2 - 0.8, W/2], [-L/2, W/2]])
    c, s = np.cos(yaw), np.sin(yaw)
    ax.add_patch(Polygon(hull @ np.array([[c, s], [-s, c]]) + [x, y], closed=True, fc=color, ec='k', lw=0.4,
                         alpha=alpha, zorder=3))

def snapshots(ax, d, b, ts, tgt=None):
    # older instants are drawn more transparent, so that the instants of the vehicles can be matched
    for k, t in enumerate(ts):
        a = 0.3 + 0.7 * k / max(len(ts) - 1, 1)
        pb, pd = at(b, t), at(d, t)
        boat(ax, pb[0], pb[1], pb[3], alpha=a)
        ax.plot([pb[0], pd[0]], [pb[1], pd[1]], color=TETHER, lw=0.6, ls='-', alpha=a, zorder=4)
        ax.plot(pd[0], pd[1], 'o', ms=3.5, mfc=UAV, mec='k', mew=0.4, alpha=a, zorder=5)
        if tgt is not None and len(tgt):
            pt = at(tgt, t); boat(ax, pt[0], pt[1], pt[3], color=TGT, alpha=a)

def faded(ax, a, t0, t1, color):
    """Trajectory faded towards white at the older instants, as the snapshots (blending instead of transparency, so
    that the overlapping segments do not darken)."""
    from matplotlib.collections import LineCollection
    from matplotlib.colors import to_rgb
    k = (0.3 + 0.7 * (a[:-1, 0] - t0) / (t1 - t0))[:, None]
    pts = a[:, 1:3]
    ax.add_collection(LineCollection(np.stack([pts[:-1], pts[1:]], 1), colors=1 - k * (1 - np.array(to_rgb(color))),
                                     lw=1.0, capstyle='round'))
    ax.autoscale_view()

def top(ax, d, b, t0, t1, n=4, tgt=None, title=''):
    faded(ax, win(b, t0, t1), t0, t1, USV)
    faded(ax, win(d, t0, t1), t0, t1, UAV)
    if tgt is not None and len(tgt):
        faded(ax, win(tgt, t0, t1), t0, t1, TGT)
    snapshots(ax, d, b, np.linspace(t0, t1, n), win(tgt, t0 - 1, t1 + 1) if tgt is not None and len(tgt) else None)
    ax.set_aspect('equal', adjustable='datalim'); ax.set_title(title, fontsize=9)
    ax.set_xlabel('$x$ (m)', fontsize=8); ax.set_ylabel('$y$ (m)', fontsize=8)

fig, axs = plt.subplots(2, 3, figsize=(6.3, 4.5))
axs = axs.ravel()

# showcase: takeoff, goto, orbit, land
d, b, _, ev = load('showcase')
t_arm, t_to, t_away, t_orb, t_land, t_cent, t_td = (tev(ev, s) for s in (
    'armed in offboard', 'uav.takeoff: done', 'uav.away: done', 'uav.orbit: done', 'uav.land: started',
    'centred above the pad', 'touchdown'))
PHASE_COLORS = ['#e8eef7', '#f7ece0', '#e6f2e6', '#f3e3e3']
def side(ax, t0, t1, phases, title):
    dd = win(d, t0, t1)
    # heights above the mean height of the winch; the orange line is the heaving winch
    z0 = b[:, 3].mean() + WINCH_H
    for k, (pa, pb, label) in enumerate(phases):
        ax.axvspan(pa - t0, pb - t0, color=PHASE_COLORS[k], lw=0, zorder=0)
        ax.text((pa + pb) / 2 - t0, 1.0, label, transform=ax.get_xaxis_transform(), ha='center', va='bottom',
                fontsize=7)
    ax.plot(dd[:, 0] - t0, dd[:, 3] - z0, color=UAV, lw=1.0)
    bb = win(b, t0, t1); ax.plot(bb[:, 0] - t0, bb[:, 3] + WINCH_H - z0, color=USV, lw=1.0)
    ax.set_xlim(0, t1 - t0)
    ax.set_xlabel('$t$ (s)', fontsize=8); ax.set_ylabel('height above winch (m)', fontsize=8)
    ax.set_title(title, fontsize=9, pad=12)

side(axs[0], t_arm - 1, t_to + 2, [(t_arm, t_arm + 5, '1'), (t_arm + 5, t_to, '2')], '(a) Takeoff')
side(axs[5], t_land, t_td + 4, [(t_land, t_land + 8, '1'), (t_land + 8, t_cent, '2'),
                                 (t_cent, t_cent + 12, '3'), (t_cent + 12, t_td, '4')], '(f) Landing')
# the orbit is drawn in the axes of the boat, centred on the winch, where it is a circle
ax = axs[2]
dd = win(d, t_away, t_orb)
xb, yb, psi = (np.interp(dd[:, 0], b[:, 0], c) for c in (b[:, 1], b[:, 2], np.unwrap(b[:, 4])))
rel = np.column_stack([dd[:, 0], np.cos(psi) * (dd[:, 1] - xb) + np.sin(psi) * (dd[:, 2] - yb),
                       -np.sin(psi) * (dd[:, 1] - xb) + np.cos(psi) * (dd[:, 2] - yb)])
faded(ax, rel, t_away, t_orb, UAV)
boat(ax, 0.0, 0.0, 0.0)
for k, t in enumerate(np.linspace(t_away, t_orb, 5)):
    a = 0.3 + 0.7 * k / 4
    px, py = np.interp(t, rel[:, 0], rel[:, 1]), np.interp(t, rel[:, 0], rel[:, 2])
    ax.plot([0, px], [0, py], color=TETHER, lw=0.6, alpha=a, zorder=4)
    ax.plot(px, py, 'o', ms=3.5, mfc=UAV, mec='k', mew=0.4, alpha=a, zorder=5)
ax.set_aspect('equal', adjustable='datalim'); ax.set_title('(c) Orbit', fontsize=9)
ax.set_xlabel('$x$ in the USV axes (m)', fontsize=8); ax.set_ylabel('$y$ in the USV axes (m)', fontsize=8)

# scout: transfer to a point kept in the axes of the boat, ahead of the bow
d, b, _, ev = load('scout')
top(axs[1], d, b, tev(ev, 'uav.watch: started'), tev(ev, 'uav.watch: done'), n=5, title='(b) Formation')

# patrol: orbit around the boat until the target is detected, chase + pursuit
d, b, tg, ev = load('interdiction')
t_s0, t_det, t_int = tev(ev, 'uav.orbit: started'), tev(ev, 'target detected'), tev(ev, 'target intercepted')
top(axs[3], d, b, t_s0, t_det, n=4, tgt=tg, title='(d) Orbit on patrol')
top(axs[4], d, b, t_det, t_int - 6, n=3, tgt=tg, title='(e) Chase')

for ax in axs:
    ax.tick_params(labelsize=7)
from matplotlib.lines import Line2D
handles = [Line2D([], [], color=UAV, marker='o', mec='k', mew=0.4, ms=3.5), Line2D([], [], color=USV, lw=4),
           Line2D([], [], color=TGT, lw=4), Line2D([], [], color=TETHER, lw=0.8)]
fig.legend(handles, ['UAV', 'USV', 'Moving target', 'Tether'], loc='upper center', ncol=4,
           frameon=False, fontsize=8, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(rect=(0, 0, 1, 0.97))
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT)
print('saved', os.path.normpath(OUT))
