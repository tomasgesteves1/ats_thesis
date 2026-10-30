#!/usr/bin/env python3
"""
Redraw the USV model-validation figures of Section 4.2.1 in the current thesis style, from the curves stored in the
vector PDFs that were generated on 2026-10-12 (commit 3d5b241), since the bag recorded after the CG fix of the
simulator, from which they were computed, is no longer available.

Each curve is read from the PDF as the polyline that matplotlib wrote, and mapped to data units through the tick
marks of its axes (linear in both axes). The recovered data is checked against Table 4.6 before plotting (the errors over the 2 s horizon exactly; the RMSE
over the whole run only approximately, since matplotlib simplified the long time series when writing the PDFs).

Usage: replot_from_pdf.py <dir with the original PDFs>
"""
import os
import re
import sys

import numpy as np
import matplotlib.pyplot as plt
import pymupdf

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))
from thesis_style import set_thesis_style, save_figure, fig_size, legend_above, SIMULATION_COLOR, COLOR_CYCLE  # noqa: E402

LIMIT_COLOR = '#d62728'   # limits, as in Chapter 5
OUT = os.path.abspath(os.path.join(CURRENT_DIR, '..', '..', 'latex', 'tese', 'Implementation', 'figures',
                                   '02_model_validation'))


def _number(text):
    t = text.strip().replace('−', '-')
    try:
        return float(t)
    except ValueError:
        return None


def read_axes(path):
    """Axes box, data mapping and the polylines (with colour, width and dash) of a single-axes matplotlib PDF."""
    page = pymupdf.open(path)[0]
    labels = []
    for b in page.get_text('dict')['blocks']:
        for line in b.get('lines', []):
            for sp in line['spans']:
                v = _number(sp['text'])
                if v is not None:
                    x0, y0, x1, y1 = sp['bbox']
                    labels.append((v, 0.5 * (x0 + x1), 0.5 * (y0 + y1)))
    drawings = page.get_drawings()
    box = max((d['rect'] for d in drawings if d['items'][0][0] == 're' and d['rect'].width < page.rect.width - 1),
              key=lambda r: r.width * r.height)
    xt, yt = [], []
    for d in drawings:
        if len(d['items']) != 1 or d['items'][0][0] != 'l' or d.get('color') != (0.0, 0.0, 0.0):
            continue
        p, q = d['items'][0][1], d['items'][0][2]
        if abs(p.x - q.x) < 1e-3 and abs(p.y - q.y) < 5 and min(p.y, q.y) >= box.y1 - 0.5:
            xt.append(p.x)                                 # tick mark below the axes
        elif abs(p.y - q.y) < 1e-3 and abs(p.x - q.x) < 5 and max(p.x, q.x) <= box.x0 + 0.5:
            yt.append(p.y)                                 # tick mark left of the axes

    def fit(ticks, along):
        pairs = []
        for t in ticks:
            cand = [lab for lab in labels if (lab[2] > box.y1 if along == 'x' else lab[1] < box.x0)]
            lab = min(cand, key=lambda l_: abs((l_[1] if along == 'x' else l_[2]) - t))
            pairs.append((t, lab[0]))
        a = np.array(pairs)
        k, c = np.polyfit(a[:, 0], a[:, 1], 1)
        res = np.abs(np.polyval([k, c], a[:, 0]) - a[:, 1]).max()
        assert res < 1e-6 * max(1.0, np.abs(a[:, 1]).max()) + 1e-3 * abs(k), (path, along, pairs)
        return k, c

    kx, cx = fit(xt, 'x')
    ky, cy = fit(yt, 'y')
    curves = []
    for d in drawings:
        items = d['items']
        if d.get('color') is None or not all(i[0] == 'l' for i in items):
            continue
        r = d['rect']
        if r.x0 < box.x0 - 0.5 or r.x1 > box.x1 + 0.5 or r.y0 < box.y0 - 0.5 or r.y1 > box.y1 + 0.5:
            continue
        pts = [items[0][1]] + [i[2] for i in items]
        xy = np.array([[kx * p.x + cx, ky * p.y + cy] for p in pts])
        col = '#%02x%02x%02x' % tuple(int(round(255 * c)) for c in d['color'][:3])
        curves.append(dict(xy=xy, color=col, width=d.get('width') or 0, dashed=bool(d.get('dashes')
                                                                                and d['dashes'] != '[] 0'),
                           n=len(items), rect=r))
    return curves


def series(curves, color, min_n=3):
    """Polylines of the given colour (legend samples, grid lines and axis lines have fewer points)."""
    return [c['xy'] for c in curves if c['color'] == color and c['n'] >= min_n]


def check(name, value, table, tol):
    ok = abs(value - table) <= tol
    print(f'  {name:42s} recovered {value:9.4f} | Table 4.6 {table:9.4f} | {"OK" if ok else "DIFFERENT"}')
    return ok


def main(src):
    blue, grey, orange = COLOR_CYCLE[0], '#666666', COLOR_CYCLE[1]
    data = {}
    # Velocities over the whole run: simulation (grey) and model (blue)
    for var in 'uvr':
        c = read_axes(os.path.join(src, f'usv_velocity_{var}_validation.pdf'))
        data[var] = (series(c, grey)[0], series(c, blue)[0])
    # Thruster inputs: port (blue) and starboard (orange)
    for name in ('thrust', 'azimuth'):
        c = read_axes(os.path.join(src, f'usv_actuator_inputs_{name}.pdf'))
        data[name] = (series(c, blue)[0], series(c, orange)[0])
    # Prediction errors over the horizon
    for name in ('pos', 'psi', 'vel', 'r'):
        c = read_axes(os.path.join(src, f'usv_prediction_error_{name}.pdf'))
        data['err_' + name] = series(c, blue)[0]
    # Trajectory and its 2 s predictions
    c = read_axes(os.path.join(src, 'usv_prediction_trajectory.pdf'))
    data['traj'] = max(series(c, grey), key=len)
    data['pred'] = [s for s in series(c, blue, min_n=2) if len(s) > 2]

    # --- Check against Table 4.6 ---------------------------------------------------------------------------------
    print('Recovered data against Table 4.6:')
    ok = True
    for var, table in (('u', 0.0129), ('v', 0.0092), ('r', 0.1579)):
        sim, mod = data[var]
        t = np.linspace(max(sim[0, 0], mod[0, 0]), min(sim[-1, 0], mod[-1, 0]), 5000)
        rmse = np.sqrt(np.mean((np.interp(t, *sim.T) - np.interp(t, *mod.T)) ** 2))
        # matplotlib dropped the points that did not change the drawing (path simplification), so the RMSE over the
        # run can only be approximated from these curves: informative, not a pass/fail check
        check(f'{var} RMSE over the run (approx., simplified)', rmse, table, 0.5 * table)
    for name, table in (('pos', 0.0143), ('psi', 0.1599), ('vel', 0.0147), ('r', 0.1594)):
        e = data['err_' + name]
        ok &= check(f'{name} RMSE at the end of the 2 s horizon', e[-1, 1], table, 0.02 * table)
    print('  run duration %.2f s (thesis 69.96 s)' % (data['u'][0][-1, 0] - data['u'][0][0, 0]))
    if not ok:
        print('The recovered data does not match Table 4.6: figures not written.')
        sys.exit(1)

    # --- Figures in the thesis style -----------------------------------------------------------------------------
    set_thesis_style()
    for var, unit in (('u', 'm/s'), ('v', 'm/s'), ('r', 'deg/s')):
        sim, mod = data[var]
        fig, ax = plt.subplots(figsize=fig_size(f'usv_velocity_{var}_validation'))
        ax.plot(*sim.T, color=SIMULATION_COLOR, label='Simulation', linewidth=1.25)
        ax.plot(*mod.T, color=COLOR_CYCLE[0], label='Prediction model', linewidth=1.0)
        lo, hi = min(sim[:, 1].min(), mod[:, 1].min()), max(sim[:, 1].max(), mod[:, 1].max())
        span = hi - lo
        ax.set_ylim(lo - 0.08 * span, hi + 0.38 * span)     # room for the legend, as in the original
        ax.set_xlabel('Time (s)'); ax.set_ylabel(f'${var}$ ({unit})')
        legend_above(ax)    # narrow panel: the legend above the axes, as in the UAV panels
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, f'usv_velocity_{var}_validation'))
        plt.close(fig)

    for name, ylabel, ylim, lims, labels, limit_label in (
            ('thrust', 'Thrust (N)', (-450.0, 1000.0), (510.0, -380.0), ('Port ($T_L$)', 'Starboard ($T_R$)'),
             r'Limits ($510/-380\,\mathrm{N}$)'),
            ('azimuth', 'Azimuth (deg)', (-36.0, 72.0), (30.0, -30.0), (r'Port ($\alpha_L$)', r'Starboard ($\alpha_R$)'),
             r'Limits ($\pm 30^\circ$)')):
        port, stbd = data[name]
        fig, ax = plt.subplots(figsize=fig_size(f'usv_actuator_inputs_{name}'))
        ax.plot(*port.T, color=COLOR_CYCLE[0], label=labels[0], linewidth=1.0)
        ax.plot(*stbd.T, color=COLOR_CYCLE[1], label=labels[1], linewidth=1.0)
        for i, y in enumerate(lims):
            ax.axhline(y, color=LIMIT_COLOR, linestyle='--', linewidth=0.9, label=limit_label if i == 0 else None)
        ax.set_ylim(*ylim); ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel)
        ax.legend(loc='upper right')
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, f'usv_actuator_inputs_{name}'))
        plt.close(fig)

    for name, ylabel in (('pos', 'Position error RMSE (m)'), ('psi', 'Heading error RMSE (deg)'),
                         ('vel', 'Velocity error RMSE (m/s)'), ('r', 'Yaw rate error RMSE (deg/s)')):
        e = data['err_' + name]
        fig, ax = plt.subplots(figsize=fig_size(f'usv_prediction_error_{name}'))
        ax.plot(*e.T, color=COLOR_CYCLE[0], linewidth=1.0, label='Prediction model')
        ax.set_xlabel('Prediction horizon (s)'); ax.set_ylabel(ylabel)
        ax.set_xlim(0, e[-1, 0]); ax.set_ylim(bottom=0)
        ax.legend(loc='upper left')
        fig.tight_layout()
        save_figure(fig, os.path.join(OUT, f'usv_prediction_error_{name}'))
        plt.close(fig)

    traj, preds = data['traj'], data['pred']
    fig, ax = plt.subplots(figsize=fig_size('usv_prediction_trajectory'))
    ax.plot(*traj.T, color=SIMULATION_COLOR, linewidth=1.2, label='Simulation')
    for j, s in enumerate(preds):
        ax.plot(*s.T, color=COLOR_CYCLE[0], linewidth=1.2, label='2 s predictions' if j == 0 else None)
        ax.plot(*s[0], 'o', color=COLOR_CYCLE[0], markersize=2.5)
    ax.plot(*traj[0], 's', color='#333333', markersize=4, label='Start')
    ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)')
    ax.margins(0.12); ax.axis('equal')
    ax.legend(loc='lower left')
    fig.tight_layout()
    save_figure(fig, os.path.join(OUT, 'usv_prediction_trajectory'))
    plt.close(fig)


if __name__ == '__main__':
    main(sys.argv[1])
