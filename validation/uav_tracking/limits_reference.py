#!/usr/bin/env python3
"""Reference of the actuator-limit test of the UAV (Section 5.1), with the USV holding its position at the origin.

The reference asks more than the UAV can do, one limit after the other, while keeping clear of the limits of the tether
(d_max 35 m, |d_dot| <= 2 m/s), which belong to Section 5.2:
  1. hover, then an arc of 15 m radius at 10.5 m/s, which needs about 37 deg of tilt (the pull of the tether helps
     by ~1.5 m/s^2): the tilt and its rate saturate, and the speed limit is close;
  2. a slow move to 25 m, then an arc at 11 m/s, which the tilt allows but the horizontal speed limit does not;
  3. a slow move out to 30 m and down to 3 m, where a change of height hardly changes the distance to the winch;
  4. a step up of 7 m and a step down to the water: the vertical speed and the thrust rate saturate (the thrust itself
     does not, within the bound on the vertical speed), and the step down ends on the minimum altitude.
The arcs span 240 deg on the +x side, clear of the radio tower of the inspection missions at (-25, 0).
The table is not periodic: after its end the reference holds the last point.

Usage: limits_reference.py [--csv out.csv] [--plot out_prefix]
(the reference used by record_tracking.sh is mission/test_uav_circle/config/limits.csv)
"""
import argparse
import os
import sys

import numpy as np

DT = 0.01
Z_WINCH = 1.3
G = 9.81
TILT_MAX = 0.5        # rad, NMPC
V_XY_MAX = 10.0       # m/s, NMPC
Z_MIN = 2.0           # m, NMPC
D_MAX = 35.0          # m, maximum separation in this test

# Phases: (name, duration, function of the time in the phase -> position)
R_A, R_B, R_C = 15.0, 25.0, 30.0
H, H_LOW, H_HIGH = 8.0, 3.0, 10.0
V_ORBIT_A, V_ORBIT_B = 10.5, 11.0
ARC_DEG = 240.0     # arcs centred on +x, clear of the radio tower at (-25, 0)
V_MOVE = 1.5


def phases():
    """The orbits are arcs of ARC_DEG centred on +x: the radio tower of the inspection missions stands at (-25, 0)."""
    ph = []
    hold = lambda p: (lambda t: np.array(p, float))
    a0 = -np.radians(ARC_DEG) / 2
    on = lambda r, a, z: [r * np.cos(a), r * np.sin(a), z]

    def arc(r, z, v):
        w = v / r
        return lambda t: np.array(on(r, a0 + w * t, z))

    def line(a, b, v):
        a, b = np.array(a, float), np.array(b, float)
        T = np.linalg.norm(b - a) / v
        return T, (lambda t: a + (b - a) * min(t / T, 1.0))

    arc_time = lambda r, v: np.radians(ARC_DEG) * r / v
    ph.append(('hover', 6.0, hold(on(R_A, a0, H))))
    ph.append(('arc A (tilt)', arc_time(R_A, V_ORBIT_A), arc(R_A, H, V_ORBIT_A)))
    ph.append(('hold', 5.0, hold(on(R_A, -a0, H))))
    T, f = line(on(R_A, -a0, H), on(R_B, a0, H), V_MOVE); ph.append(('move', T, f))
    ph.append(('hold', 3.0, hold(on(R_B, a0, H))))
    ph.append(('arc B (speed)', arc_time(R_B, V_ORBIT_B), arc(R_B, H, V_ORBIT_B)))
    ph.append(('hold', 5.0, hold(on(R_B, -a0, H))))
    T, f = line(on(R_B, -a0, H), on(R_C, -a0, H_LOW), V_MOVE); ph.append(('move out and down', T, f))
    ph.append(('hold', 5.0, hold(on(R_C, -a0, H_LOW))))
    ph.append(('step up (v_z, thrust rate)', 8.0, hold(on(R_C, -a0, H_HIGH))))
    ph.append(('step down (v_z, z_min)', 8.0, hold(on(R_C, -a0, 0.0))))
    return ph


def build():
    ts, ps, names, t0 = [], [], [], 0.0
    for name, T, f in phases():
        n = int(round(T / DT))
        for k in range(n):
            ts.append(t0 + k * DT); ps.append(f(k * DT)); names.append(name)
        t0 += n * DT
    t, p = np.array(ts), np.array(ps)
    v = np.gradient(p, DT, axis=0)
    v[np.linalg.norm(v, axis=1) > 50.0] = 0.0          # the steps are jumps of the position, not motions
    return t, p, v, names


def check(t, p, v, names):
    """What each phase asks of the UAV, and the tether quantities of the reference itself."""
    print('duration %.1f s' % t[-1])
    r = p - [0, 0, Z_WINCH]
    d = np.linalg.norm(r, axis=1)
    for name in dict.fromkeys(names):
        m = np.array([n == name for n in names])
        if name.startswith('arc'):
            R = np.hypot(p[m, 0], p[m, 1]).mean(); vv = np.hypot(v[m, 0], v[m, 1]).mean()
            print('  %-30s speed %.1f m/s (limit %.0f), tilt needed %.1f deg (limit %.1f)' % (
                name, vv, V_XY_MAX, np.degrees(np.arctan(vv ** 2 / R / G)), np.degrees(TILT_MAX)))
        if name.startswith('step'):
            print('  %-30s height %.1f m (z_min %.0f), distance %.1f..%.1f m' % (
                name, p[m, 2][0], Z_MIN, d[m].min(), d[m].max()))
    dd = np.sum(r * v, axis=1) / d
    print('  separation %.1f..%.1f m (d_max %.0f), |d_dot| of the reference outside the steps %.2f m/s' % (
        d.min(), d.max(), D_MAX, np.abs(dd).max()))
    # On a step the UAV moves vertically; at 30 m from the winch the rate of the distance is at most ~0.3 of the climb rate
    m = np.array([n.startswith('step') for n in names])
    print('  steps: d_dot / climb rate at most %.2f' % ((p[m, 2] - Z_WINCH) / d[m]).max())


def plot(prefix, t, p, v, names):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from thesis_style import set_thesis_style, COLOR_CYCLE
    import matplotlib.pyplot as plt
    set_thesis_style()
    fig, ax = plt.subplots(figsize=(4.4, 4.0))
    ax.plot(p[:, 1], p[:, 0], color=COLOR_CYCLE[0], lw=1)
    ax.plot(0, 0, 's', color=COLOR_CYCLE[1], ms=6); ax.annotate('USV', (0, 0), xytext=(6, 0), textcoords='offset points')
    ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('$y$ (m)'); ax.set_ylabel('$x$ (m)')
    fig.tight_layout(); fig.savefig(prefix + '_top.png', dpi=150)
    fig, axs = plt.subplots(2, 1, figsize=(7.0, 4.0), sharex=True)
    axs[0].plot(t, np.hypot(v[:, 0], v[:, 1]), color=COLOR_CYCLE[0]); axs[0].axhline(V_XY_MAX, color=COLOR_CYCLE[3], ls='--')
    axs[0].set_ylabel('Horizontal speed (m/s)')
    axs[1].plot(t, p[:, 2], color=COLOR_CYCLE[0]); axs[1].axhline(Z_MIN, color=COLOR_CYCLE[3], ls='--')
    axs[1].set_ylabel('Height (m)'); axs[1].set_xlabel('Time (s)')
    t0 = 0.0
    for name in dict.fromkeys(names):
        m = np.array([n == name for n in names]); a, b = t[m][0], t[m][-1]
        for ax_ in axs:
            ax_.axvline(a, color='k', lw=0.4, alpha=0.4)
        if name != 'hold':
            axs[0].text((a + b) / 2, axs[0].get_ylim()[1] * 0.92, name, ha='center', fontsize=6, rotation=0)
    for ax_ in axs:
        ax_.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(prefix + '_time.png', dpi=150)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()
    t, p, v, names = build()
    check(t, p, v, names)
    if args.csv:
        a = np.zeros_like(v)
        np.savetxt(args.csv, np.column_stack([t, p, v, a]), delimiter=',', fmt='%.5f',
                   header='t,x,y,z,vx,vy,vz,ax,ay,az', comments='')
    if args.plot:
        plot(args.plot, t, p, v, names)


if __name__ == '__main__':
    main()
