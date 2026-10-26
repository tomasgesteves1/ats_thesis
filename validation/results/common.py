"""Shared helpers of the Chapter 5 figures: loading of the extracted npz, time windows and metrics."""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from thesis_style import set_thesis_style, save_figure, COLOR_CYCLE  # noqa: E402,F401

WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
BAGS = os.path.join(WS, 'bags')
DATA = os.path.join(HERE, 'data')
FIGS = os.path.join(WS, 'src', 'latex', 'tese', 'Results', 'figures')
EA_FIGS = os.path.join(WS, 'src', 'latex', 'extended_abstract', 'figures')   # column-width variants
UAV, USV, PX4, TGT, LIMIT, REF = COLOR_CYCLE[0], COLOR_CYCLE[1], COLOR_CYCLE[2], COLOR_CYCLE[4], COLOR_CYCLE[3], '#666666'

# Layout of /mpc_states_and_inputs (uav_mpc_node)
TEL = {"x": 0, "y": 1, "z": 2, "roll_cmd": 6, "pitch_cmd": 7, "thrust": 8, "roll": 9, "pitch": 10, "yaw": 11,
       "ref_x": 12, "ref_y": 13, "ref_z": 14, "vx": 15, "vy": 16, "vz": 17, "yaw_cmd": 21}

# Limits of the controllers (Tables of Section 4.3)
UAV_LIM = dict(phi_max=0.5, theta_max=0.5, rate_max=0.8, psi_rate_max=1.0, T_min=2.02, T_max=39.96, T_rate_max=100.0,
               d_dot_max=2.0, v_max=10.0)
USV_LIM = dict(T_max=510.0, T_min=-380.0, alpha_max=np.radians(30.0), alpha_rate_max=np.radians(10.0), T_rate_max=500.0)


def load(name):
    d = dict(np.load(os.path.join(DATA, name + '.npz'), allow_pickle=True))
    if len(d['tel']):
        t = d['tel'][:, 0]; d['tel'] = {'t': t, **{k: d['tel'][:, 1 + i] for k, i in TEL.items()}}
    return d


def interp(a, t, cols):
    return np.column_stack([np.interp(t, a[:, 0], a[:, c]) for c in cols])


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def events(d):
    return list(zip(d['ev_t'], d['ev']))


def first_event(d, text):
    return next((t for t, e in events(d) if text in e), None)
