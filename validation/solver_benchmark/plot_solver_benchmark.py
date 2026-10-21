#!/usr/bin/env python3
"""Figures of the offline solver benchmark (called by solver_benchmark.py plot)."""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from thesis_style import set_thesis_style, COLOR_CYCLE

LIMIT_COLOR = "#d62728"
SIZE = (3.3, 2.6)  # one subfloat of 0.48\textwidth
# Fixed axes area (no tight cropping), so that the panels of a subfloat pair have the same size
MARGINS = dict(left=0.19, right=0.97, bottom=0.17, top=0.80)
Y_LIMITS = {"usv": (0.05, 300.0), "uav": (0.8, 150.0)}


def _save(fig, fig_dir, name):
    os.makedirs(fig_dir, exist_ok=True)
    fig.subplots_adjust(**MARGINS)
    path = os.path.join(fig_dir, f"{name}.pdf")
    fig.savefig(path, format="pdf")
    print("saved", path)

# Configurations compared in the iteration-scheme figures, with their legend labels (online first)
SCHEMES = {
    "usv": [("online", r"SQP, tol. $10^{-4}$, 25 it."), ("rti", "Real-time iteration"),
            ("default", r"SQP, tol. $10^{-6}$, 100 it.")],
    "uav": [("online", "Real-time iteration"), ("sqp", r"SQP, tol. $10^{-4}$, 25 it.")],
}


def _legend_above(ax, handles, labels):
    leg = ax.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=2, fontsize=7,
                    frameon=False, columnspacing=1.0, handlelength=1.6)
    for h in getattr(leg, "legend_handles", None) or leg.legendHandles:
        h.set_linewidth(1.5)


def _load(results, vehicle, name):
    r = np.load(os.path.join(results, f"{vehicle}_{name}.npz"))
    return {k: r[k] for k in r.files}


def _time_axis(r):
    return r["t"] - r["t"][0]


def plot_scheme(results, fig_dir, vehicle, Ts):
    runs = [(_load(results, vehicle, n), label) for n, label in SCHEMES[vehicle]]
    # Computation time per sampling instant
    fig, ax = plt.subplots(figsize=SIZE)
    for i, (r, label) in reversed(list(enumerate(runs))):
        ax.plot(_time_axis(r), r["time_ms"], color=COLOR_CYCLE[i], lw=0.7, label=label)
    ax.axhline(1e3 * Ts, color=LIMIT_COLOR, ls="--", lw=1.0, label=rf"$T_s = {1e3 * Ts:.0f}$ ms")
    ax.set_yscale("log")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Computation time (ms)")
    h, l = ax.get_legend_handles_labels()
    order = [len(runs) - 1 - i for i in range(len(runs))] + [len(runs)]
    _legend_above(ax, [h[i] for i in order], [l[i] for i in order])
    ax.set_ylim(*Y_LIMITS[vehicle])
    _save(fig, fig_dir, f"{vehicle}_solver_time")
    plt.close(fig)
    # Optimality residual of the applied solution
    fig, ax = plt.subplots(figsize=SIZE)
    for i, (r, label) in reversed(list(enumerate(runs))):
        ax.plot(_time_axis(r), r["kkt"], color=COLOR_CYCLE[i], lw=0.7, label=label)
    ax.set_yscale("log")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Optimality residual")
    h, l = ax.get_legend_handles_labels()
    _legend_above(ax, h[::-1], l[::-1])
    _save(fig, fig_dir, f"{vehicle}_solver_residual")
    plt.close(fig)


def plot_horizon(results, fig_dir, vehicle, names, Ts, N_used):
    Ns, mean, worst = [], [], []
    for n in names:
        r = _load(results, vehicle, n)
        Ns.append(int(r["N"]))
        mean.append(r["time_ms"].mean())
        worst.append(r["time_ms"].max())
    fig, ax = plt.subplots(figsize=SIZE)
    ax.plot(Ns, worst, color=COLOR_CYCLE[0], ls="--", marker="o", mfc="white", ms=5, label="Maximum")
    ax.plot(Ns, mean, color=COLOR_CYCLE[0], marker="o", ms=5, label="Mean")
    ax.axhline(1e3 * Ts, color=LIMIT_COLOR, ls="--", lw=1.0, label=rf"$T_s = {1e3 * Ts:.0f}$ ms")
    ax.axvline(N_used, color="#888888", lw=0.8, ls=":")
    ax.text(N_used, 0.97, f" $N = {N_used}$ (used)", transform=ax.get_xaxis_transform(), va="top", fontsize=7,
            color="#555555")
    ax.set_xticks(Ns)
    ax.set_ylim(0, 1e3 * Ts * 1.12)
    ax.set_xlabel("Prediction horizon $N$")
    ax.set_ylabel("Computation time (ms)")
    h, l = ax.get_legend_handles_labels()
    _legend_above(ax, h, l)
    _save(fig, fig_dir, f"{vehicle}_horizon_time")
    plt.close(fig)


def plot_all(results, fig_dir, configs, sweep, TS):
    set_thesis_style()
    for vehicle in ("usv", "uav"):
        plot_scheme(results, fig_dir, vehicle, TS[vehicle])
        plot_horizon(results, fig_dir, vehicle, sweep[vehicle], TS[vehicle], configs[vehicle]["online"][0])
