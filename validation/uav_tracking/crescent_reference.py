#!/usr/bin/env python3
"""Crescent reference of the UAV tracking test (Section 5.1), with the USV holding its position at the origin.

One lap: a fast outer arc around the USV (counter-clockwise), a slow U-turn inwards, a fast inner arc (clockwise) and a
slow U-turn outwards. On the arcs the radius and the height are constant, so the distance to the winch does not change
and the speed can be high; the distance only changes in the slow U-turns. The lap is built as a polyline walked with a
speed profile, and then smoothed in time by a periodic Gaussian filter, so that the position and all its derivatives are
continuous (no jump of curvature or acceleration at the junctions). The derivatives are computed spectrally.

Usage: crescent_reference.py [--csv out.csv] [--plot out_prefix]
(the reference used by record_tracking.sh is mission/test_uav_circle/config/crescent.csv)
Writes the table t,x,y,z,vx,vy,vz,ax,ay,az (one lap, periodic, 100 Hz) and checks the reference against the limits of
the NMPC and of the PX4 position controller; exits with an error if any check fails.
"""
import argparse
import os
import sys

import numpy as np

# Geometry of the lap (m), speeds (m/s), smoothing (s)
P = dict(R_out=22.0, R_in=14.0, z_out=14.0, z_in=10.0, span_deg=150.0,
         v_out=6.0, v_in=5.0, v_turn=1.5, a_lon=1.0, sigma=0.7, dt=0.01)

Z_WINCH = 1.3       # height of the winch above the water (m)
MASS, G = 2.06, 9.81
C_D = 0.2065        # tension of the tether per metre of separation (N/m), Section 4.2

# Targets of the reference, well inside the limits of both controllers (Section 4.3 and PX4 defaults)
TARGETS = dict(v_xy=6.5,          # NMPC 10 m/s, PX4 12 m/s
               v_z=0.8,           # NMPC 10 m/s, PX4 1.5 m/s down
               a_xy=2.0,          # PX4 3 m/s^2; NMPC tilt 0.5 rad ~ 5.4 m/s^2
               tilt_deg=12.0,     # NMPC 28.6 deg, PX4 45 deg
               tilt_rate=0.25,    # NMPC 0.8 rad/s
               thrust=30.0,       # NMPC 39.96 N
               thrust_rate=10.0,  # NMPC 30 N/s
               d=30.0,            # d_max 35 m in this test
               d_dot=1.6)         # NMPC and winch 2 m/s


def polyline(p, ds=0.02):
    """Closed path of the lap, sampled every ds, with the nominal speed of each sample."""
    a0, a1 = np.radians(-p['span_deg'] / 2), np.radians(p['span_deg'] / 2)
    rc, rm = (p['R_out'] - p['R_in']) / 2, (p['R_out'] + p['R_in']) / 2
    pts, vel = [], []

    def arc(R, z, a_from, a_to, v):
        n = int(abs(a_to - a_from) * R / ds)
        for a in np.linspace(a_from, a_to, n, endpoint=False):
            pts.append((R * np.cos(a), R * np.sin(a), z)); vel.append(v)

    def uturn(a, z_from, z_to, inwards):
        c = rm * np.array([np.cos(a), np.sin(a)])
        er, et = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
        n = int(np.pi * rc / ds)
        for b in np.linspace(0.0, np.pi, n, endpoint=False):
            q = c + rc * (np.cos(b) * er + np.sin(b) * et) if inwards else c - rc * (np.cos(b) * er + np.sin(b) * et)
            pts.append((q[0], q[1], z_from + (z_to - z_from) * (1 - np.cos(b)) / 2)); vel.append(p['v_turn'])

    arc(p['R_out'], p['z_out'], a0, a1, p['v_out'])
    uturn(a1, p['z_out'], p['z_in'], True)
    arc(p['R_in'], p['z_in'], a1, a0, p['v_in'])
    uturn(a0, p['z_in'], p['z_out'], False)
    return np.array(pts), np.array(vel)


def build(p):
    xyz, v = polyline(p)
    seg = np.linalg.norm(np.roll(xyz, -1, 0) - xyz, axis=1)
    # Speed profile with a bounded longitudinal acceleration (forward and backward passes, periodic)
    for _ in range(3):
        for i in range(len(v)):
            v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * p['a_lon'] * seg[i - 1]))
        for i in range(len(v) - 1, -1, -1):
            j = (i + 1) % len(v)
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2 * p['a_lon'] * seg[i]))
    dt_seg = seg / v
    t_nodes = np.concatenate([[0.0], np.cumsum(dt_seg)])
    period = t_nodes[-1]
    n = int(round(period / p['dt']))
    t = np.arange(n) * period / n
    closed = np.vstack([xyz, xyz[:1]])
    X = np.stack([np.interp(t, t_nodes, closed[:, k]) for k in range(3)])
    # Periodic Gaussian smoothing and spectral derivatives
    f = np.fft.fftfreq(n, d=period / n)
    w = 2j * np.pi * f
    F = np.fft.fft(X, axis=1) * np.exp(-0.5 * (2 * np.pi * f * p['sigma']) ** 2)
    pos, vel, acc, jerk = (np.real(np.fft.ifft(F * w ** k, axis=1)) for k in range(4))
    # The lap starts in the middle of the inward U-turn (fastest descent), where the reference is slow, so that the
    # entry from hover is gentle
    k0 = int(np.argmin(vel[2]))
    pos, vel, acc, jerk = (np.roll(a, -k0, axis=1) for a in (pos, vel, acc, jerk))
    return t, period, pos, vel, acc, jerk


def check(t, pos, vel, acc, jerk):
    r = pos - np.array([[0.0], [0.0], [Z_WINCH]])
    d = np.linalg.norm(r, axis=0)
    d_dot = np.sum(r * vel, axis=0) / d
    tension = C_D * d
    # Thrust vector: m (a + g) minus the tether force (pulls towards the winch), its rate, and the tilt it needs
    f_t = -tension * r / d
    thrust_vec = MASS * (acc + np.array([[0.0], [0.0], [G]])) - f_t
    thrust = np.linalg.norm(thrust_vec, axis=0)
    tilt = np.arccos(np.clip(thrust_vec[2] / thrust, -1, 1))
    dt = t[1] - t[0]
    m = dict(v_xy=np.hypot(vel[0], vel[1]).max(), v_z=np.abs(vel[2]).max(), a_xy=np.hypot(acc[0], acc[1]).max(),
             tilt_deg=np.degrees(tilt.max()), tilt_rate=np.abs(np.gradient(tilt, dt)).max(), thrust=thrust.max(),
             thrust_rate=np.abs(np.gradient(thrust, dt)).max(), d=d.max(), d_dot=np.abs(d_dot).max())
    extra = dict(v_xy_min=np.hypot(vel[0], vel[1]).min(), d_min=d.min(), tension=(tension.min(), tension.max()),
                 jerk_xy=np.hypot(jerk[0], jerk[1]).max())
    return m, extra, dict(d=d, d_dot=d_dot, tilt=tilt, thrust=thrust)


def plot(prefix, t, period, pos, vel, sig):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from thesis_style import set_thesis_style, COLOR_CYCLE
    import matplotlib.pyplot as plt
    set_thesis_style()
    sp = np.hypot(vel[0], vel[1])
    # Top view, coloured by speed
    fig, ax = plt.subplots(figsize=(4.2, 3.6))
    sc = ax.scatter(pos[0], pos[1], c=sp, s=2, cmap='viridis')
    ax.plot(0, 0, 's', color=COLOR_CYCLE[1], ms=7, label='USV')
    for k in range(0, len(t), int(len(t) / 12)):
        ax.annotate('', xy=(pos[0, (k + 5) % len(t)], pos[1, (k + 5) % len(t)]), xytext=(pos[0, k], pos[1, k]),
                    arrowprops=dict(arrowstyle='->', color='k', lw=0.8))
    fig.colorbar(sc, ax=ax, label='Horizontal speed (m/s)')
    ax.set_aspect('equal'); ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)'); ax.legend(loc='lower left')
    fig.tight_layout(); fig.savefig(prefix + '_top.png', dpi=200); fig.savefig(prefix + '_top.pdf')
    # Time histories over one lap, with the limits of the NMPC
    fig, axs = plt.subplots(4, 1, figsize=(6.0, 7.0), sharex=True)
    axs[0].plot(t, sp, label='horizontal'); axs[0].plot(t, vel[2], label='vertical')
    axs[0].set_ylabel('Speed (m/s)'); axs[0].legend(loc='upper right')
    axs[1].plot(t, sig['d']); axs[1].set_ylabel('$d$ (m)')
    axs[2].plot(t, sig['d_dot']); axs[2].axhline(2, color=COLOR_CYCLE[3], ls='--'); axs[2].axhline(-2, color=COLOR_CYCLE[3], ls='--')
    axs[2].set_ylabel(r'$\dot d$ (m/s)')
    axs[3].plot(t, np.degrees(sig['tilt'])); axs[3].axhline(np.degrees(0.5), color=COLOR_CYCLE[3], ls='--')
    axs[3].set_ylabel('Tilt (deg)'); axs[3].set_xlabel('Time (s)')
    for a in axs:
        a.grid(True, alpha=0.3)
    axs[0].set_xlim(0, period)
    fig.tight_layout(); fig.savefig(prefix + '_time.png', dpi=200); fig.savefig(prefix + '_time.pdf')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()
    t, period, pos, vel, acc, jerk = build(P)
    m, extra, sig = check(t, pos, vel, acc, jerk)
    print(f'period {period:.1f} s, speed {extra["v_xy_min"]:.2f}-{m["v_xy"]:.2f} m/s, '
          f'd {extra["d_min"]:.1f}-{m["d"]:.1f} m, tension {extra["tension"][0]:.2f}-{extra["tension"][1]:.2f} N, '
          f'jerk {extra["jerk_xy"]:.2f} m/s^3')
    ok = True
    for k, lim in TARGETS.items():
        flag = 'ok' if m[k] <= lim else 'FAIL'
        ok &= m[k] <= lim
        print(f'  {k:12s} {m[k]:7.2f}  (target {lim})  {flag}')
    if args.csv:
        np.savetxt(args.csv, np.column_stack([t, pos.T, vel.T, acc.T]), delimiter=',', fmt='%.5f',
                   header='t,x,y,z,vx,vy,vz,ax,ay,az', comments='')
    if args.plot:
        plot(args.plot, t, period, pos, vel, sig)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
