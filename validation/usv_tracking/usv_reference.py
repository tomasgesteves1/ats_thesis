#!/usr/bin/env python3
"""References of the USV tests (Section 5.1), with the UAV resting on the deck.

track  - accuracy: two laps of a figure-eight (two circles of 12 m joined by straight legs that cross), slow on the
         circles and faster on the legs, starting and ending at rest. The model of Chapter 3 is inverted along it
         (surge, sway kept at zero, yaw rate) to obtain the thrust and azimuth each point needs, which must stay well
         inside the limits of the thrusters.
limits - actuator limits: a straight leg at 3 m/s (beyond the top speed of about 2.3 m/s), a sharp turn of 90 deg
         at that speed, then a point held still, a step 15 m astern and a step 15 m abeam.

Both tables are played once (the reference then holds the last pose). Columns t,x,y,psi (psi unwrapped).
Usage: usv_reference.py <track|limits> [--csv out.csv] [--plot out_prefix]
(the references used by record_tracking.sh are mission/test_usv_circle/config/usv_{track,limits}.csv)
"""
import argparse
import os
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d

DT = 0.05
# USV model (Table 4.x) and thruster limits (Table 4.x)
M, IZ, X_ARM, Y_ARM = 259.10, 683.22, 2.0855, 1.0271
XU, XUU, YV, YVV, NR, NRR = 100.0, 150.0, 100.0, 100.0, 800.0, 800.0
T_MAX, T_MIN, A_MAX = 510.0, -380.0, np.radians(30.0)
T_RATE, A_RATE = 500.0, np.radians(10.0)
FRACTION = 0.6       # the accuracy reference must stay within this fraction of every limit


def eight(R=12.0, D=24.0, v_leg=1.6, v_arc=1.2, a_lon=0.1, laps=2, hold=10.0, ds=0.02):
    """Figure-eight from rest to rest, as positions and headings sampled every DT."""
    beta = np.arcsin(R / D)
    L = np.sqrt(D ** 2 - R ** 2)
    e1 = np.array([np.cos(beta), np.sin(beta)]); e2 = np.array([-np.cos(beta), np.sin(beta)])
    pts, vel = [], []

    def seg(a, b, v):
        n = int(np.linalg.norm(b - a) / ds)
        for k in range(n):
            pts.append(a + (b - a) * k / n); vel.append(v)

    def arc(c, a0, a1, v):
        n = int(abs(a1 - a0) * R / ds)
        for a in np.linspace(a0, a1, n, endpoint=False):
            pts.append(c + R * np.array([np.cos(a), np.sin(a)])); vel.append(v)

    cr, cl = np.array([D, 0.0]), np.array([-D, 0.0])
    for _ in range(laps):
        seg(np.zeros(2), L * e1, v_leg)                                  # crossing leg, up and right
        arc(cr, np.radians(90 + np.degrees(beta)), np.radians(-90 - np.degrees(beta)), v_arc)   # right circle, clockwise
        seg(np.array([L * np.cos(beta), -L * np.sin(beta)]), np.zeros(2), v_leg)                # back to the crossing
        seg(np.zeros(2), L * e2, v_leg)                                  # crossing leg, up and left
        arc(cl, np.radians(90 - np.degrees(beta)), np.radians(270 + np.degrees(beta)), v_arc)   # left circle, anticlockwise
        seg(np.array([-L * np.cos(beta), -L * np.sin(beta)]), np.zeros(2), v_leg)
    pts.append(np.zeros(2)); vel.append(0.0)
    P, v = np.array(pts), np.array(vel)
    # rotate so that the first leg points along x, the initial heading of the USV
    c, s = np.cos(-beta), np.sin(-beta)
    P = P @ np.array([[c, s], [-s, c]])
    seg_len = np.linalg.norm(np.diff(P, axis=0), axis=1)
    v[0] = 0.0
    for i in range(1, len(v)):                       # accelerate from rest and brake to rest at a_lon
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_lon * seg_len[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_lon * seg_len[i]))
    dt_seg = seg_len / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-3)
    t_nodes = np.concatenate([[0.0], np.cumsum(dt_seg)])
    t = np.arange(0.0, t_nodes[-1], DT)
    xy = np.column_stack([np.interp(t, t_nodes, P[:, k]) for k in range(2)])
    n = int(hold / DT)                                                       # at rest before and after
    xy = np.vstack([np.repeat(xy[:1], n, 0), xy, np.repeat(xy[-1:], n, 0)])
    xy = gaussian_filter1d(xy, sigma=1.5 / DT, axis=0, mode='nearest')       # continuous curvature and acceleration
    t = np.arange(len(xy)) * DT
    vxy = np.gradient(xy, DT, axis=0)
    psi = np.unwrap(np.arctan2(vxy[:, 1], vxy[:, 0]))
    moving = np.hypot(vxy[:, 0], vxy[:, 1]) > 0.05
    idx = np.flatnonzero(moving)
    psi[~moving] = np.interp(np.flatnonzero(~moving), idx, psi[idx])
    return t, xy, psi


def limits():
    """Phases (duration, x, y, psi) as functions of the time in the phase; jumps are steps of the reference."""
    out, tt = [], 0.0
    def add(T, f):
        nonlocal tt
        for k in range(int(round(T / DT))):
            out.append((tt + k * DT, *f(k * DT)))
        tt += round(T / DT) * DT
    V = 3.0
    add(10.0, lambda s: (0.0, 0.0, 0.0))                                    # hold at the start
    add(15.0, lambda s: (V * s, 0.0, 0.0))                                  # sprint along x
    add(10.0, lambda s: (V * 15.0, V * s, np.pi / 2))                       # sharp turn of 90 deg, then along y
    add(15.0, lambda s: (V * 15.0, V * 10.0, np.pi / 2))                    # hold the end of the leg
    add(40.0, lambda s: (V * 15.0, V * 10.0 - 15.0, np.pi / 2))             # step 15 m astern, same heading
    add(40.0, lambda s: (V * 15.0 - 15.0, V * 10.0 - 15.0, np.pi / 2))      # step 15 m abeam (to port)
    a = np.array(out)
    return a[:, 0], a[:, 1:3], a[:, 3]


def check(t, xy, psi):
    """Invert the model along the reference (sway kept at zero) and allocate the thrust with the least effort."""
    u = np.hypot(*np.gradient(xy, DT, axis=0).T)
    r = np.gradient(psi, DT)
    du, dr = np.gradient(u, DT), np.gradient(r, DT)
    X = M * du + (XU + XUU * np.abs(u)) * u
    Y = M * u * r
    N = IZ * dr + (NR + NRR * np.abs(r)) * r
    # X = FxL + FxR, Y = FyL + FyR, N = -x_arm (FyL + FyR) - y_arm (FxL - FxR); least-norm forces of the two thrusters
    A = np.array([[1, 1, 0, 0], [0, 0, 1, 1], [-Y_ARM, Y_ARM, -X_ARM, -X_ARM]], float)
    F = np.linalg.pinv(A) @ np.vstack([X, Y, N])
    T = np.sign(F[:2]) * np.hypot(F[:2], F[2:])
    alpha = np.arctan2(F[2:], np.abs(F[:2]))
    res = dict(speed=u.max(), yaw_rate=np.degrees(np.abs(r).max()),
               thrust=(T.min(), T.max()), azimuth=np.degrees(np.abs(alpha).max()),
               thrust_rate=np.abs(np.gradient(T, DT, axis=1)).max(), azimuth_rate=np.degrees(np.abs(np.gradient(alpha, DT, axis=1)).max()))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('which', choices=['track', 'limits'])
    ap.add_argument('--csv', default='')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()
    t, xy, psi = eight() if args.which == 'track' else limits()
    print('duration %.1f s' % t[-1])
    ok = True
    if args.which == 'track':
        res = check(t, xy, psi)
        lims = dict(thrust=(FRACTION * T_MIN, FRACTION * T_MAX), azimuth=FRACTION * np.degrees(A_MAX),
                    thrust_rate=FRACTION * T_RATE, azimuth_rate=FRACTION * np.degrees(A_RATE))
        print('  speed max %.2f m/s, yaw rate max %.1f deg/s' % (res['speed'], res['yaw_rate']))
        for k, lim in lims.items():
            val = res[k]
            good = (lim[0] <= val[0] and val[1] <= lim[1]) if isinstance(lim, tuple) else val <= lim
            ok &= good
            print('  %-13s %-22s target %-22s %s' % (k, np.round(val, 1), np.round(lim, 1), 'ok' if good else 'FAIL'))
    if args.csv:
        np.savetxt(args.csv, np.column_stack([t, xy, psi]), delimiter=',', fmt='%.5f', header='t,x,y,psi', comments='')
    if args.plot:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from thesis_style import set_thesis_style, COLOR_CYCLE
        import matplotlib.pyplot as plt
        set_thesis_style()
        fig, axs = plt.subplots(1, 2, figsize=(10, 4))
        axs[0].plot(xy[:, 0], xy[:, 1], color=COLOR_CYCLE[1]); axs[0].set_aspect('equal')
        axs[0].set_xlabel('$x$ (m)'); axs[0].set_ylabel('$y$ (m)')
        axs[1].plot(t, np.hypot(*np.gradient(xy, DT, axis=0).T), color=COLOR_CYCLE[1])
        axs[1].set_xlabel('Time (s)'); axs[1].set_ylabel('Speed (m/s)')
        for a in axs:
            a.grid(True, alpha=0.3)
        fig.tight_layout(); fig.savefig(args.plot + '.png', dpi=130)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
