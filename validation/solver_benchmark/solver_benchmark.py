#!/usr/bin/env python3
"""
Offline benchmark of the USV and UAV NMPC solvers (Section 4.3 of the thesis).

The problems that each controller solved during a tracking run are rebuilt from a bag recorded with
validation/record_tracking.sh, following usv_mpc_pipeline.cpp and uav_mpc_pipeline.cpp, and solved again with
different solver configurations (iteration scheme, tolerance, iteration limit, prediction horizon N). For every
sampling instant the computation time, the number of iterations, the status and the optimality (KKT) residual are
stored.

Differences from the online controllers (signals that were not recorded in the bag):
  - the predicted trajectory of the other vehicle (UAV horizon for the USV tether constraint, USV horizon for the
    UAV tether model) is replaced by the trajectory that the vehicle actually followed;
  - the tether attachment point is the USV position raised by the deck height z_t;
  - the disturbance estimate of the UAV is rebuilt by running the observer on the replayed predictions.
The measured state and the commands applied at the previous instant (part of the initial state) are the recorded
ones, so every configuration solves the same sequence of problems as the online controller.

Usage:
  python3 solver_benchmark.py run  --vehicle usv|uav --config NAME [--bag PATH]
  python3 solver_benchmark.py all  [--bag PATH]       (runs every configuration, each in its own process)
  python3 solver_benchmark.py plot                    (figures into the thesis folder)
"""
import os
import sys
import argparse
import importlib.util
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
WS_SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
BUILD_DIR = os.path.join(HERE, "build")
RESULTS_DIR = os.path.join(HERE, "results")
FIG_DIR = os.path.join(WS_SRC, "latex", "tese", "Implementation", "figures", "03_controller_implementation")
DEFAULT_BAG = os.path.join(os.path.dirname(WS_SRC), "bags", "tracking", "tracking_aggressive_20261014_155559")

# name: (N, nlp_solver_type, tol, max_iter). The first entry of each vehicle is the configuration used online.
CONFIGS = {
    "usv": {
        "online": (20, "SQP", 1e-4, 25),
        "rti": (20, "SQP_RTI", None, None),
        "default": (20, "SQP", 1e-6, 100),
        "N10": (10, "SQP", 1e-4, 25),
        "N30": (30, "SQP", 1e-4, 25),
        "N40": (40, "SQP", 1e-4, 25),
    },
    "uav": {
        "online": (50, "SQP_RTI", None, None),
        "sqp": (50, "SQP", 1e-4, 25),
        "N25": (25, "SQP_RTI", None, None),
        "N75": (75, "SQP_RTI", None, None),
        "N100": (100, "SQP_RTI", None, None),
    },
}
HORIZON_SWEEP = {"usv": ["N10", "online", "N30", "N40"], "uav": ["N25", "online", "N75", "N100"]}
TS = {"usv": 0.1, "uav": 0.02}


# ---------------------------------------------------------------------------------------------------------------
# Solvers
# ---------------------------------------------------------------------------------------------------------------
def _load_generator(vehicle):
    path = os.path.join(WS_SRC, "control", f"{vehicle}_mpc", "scripts", f"generate_acados_{vehicle}.py")
    spec = importlib.util.spec_from_file_location(f"gen_{vehicle}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, os.path.dirname(path))
    spec.loader.exec_module(mod)
    return mod


def build_solver(vehicle, name):
    from acados_template import AcadosOcpSolver
    N, nlp_type, tol, max_iter = CONFIGS[vehicle][name]
    ocp = _load_generator(vehicle).generate_acados_ocp()
    ocp.dims.N = N
    ocp.solver_options.tf = TS[vehicle] * N
    ocp.solver_options.nlp_solver_type = nlp_type
    if tol is not None:
        ocp.solver_options.tol = tol
        ocp.solver_options.nlp_solver_max_iter = max_iter
    out = os.path.join(BUILD_DIR, f"{vehicle}_{name}")
    os.makedirs(out, exist_ok=True)
    ocp.code_export_directory = os.path.join(out, "c_generated_code")
    cwd = os.getcwd()
    os.chdir(out)  # acados builds relative to the working directory
    try:
        solver = AcadosOcpSolver(ocp, json_file=os.path.join(out, "acados_ocp.json"), verbose=False)
    finally:
        os.chdir(cwd)
    return solver, N


def solve_and_measure(solver):
    status = solver.solve()
    try:
        it = int(solver.get_stats("nlp_iter"))
    except Exception:
        it = int(solver.get_stats("sqp_iter"))
    t = float(solver.get_stats("time_tot"))
    res = np.asarray(solver.get_residuals(recompute=True), dtype=float)
    return status, it, t, float(np.max(res))


# ---------------------------------------------------------------------------------------------------------------
# Bag
# ---------------------------------------------------------------------------------------------------------------
def load_bag(bag):
    from tracking_common import read_bag, odometry_arrays
    topics = ["/drone/ground_truth/odometry", "/boat/ground_truth/odometry", "/uav/reference_path",
              "/usv/reference_path", "/uav_solve_time_ms", "/usv_solve_time_ms", "/mpc_states_and_inputs",
              "/boat/thrusters/left/thrust", "/boat/thrusters/right/thrust", "/boat/thrusters/left/pos",
              "/boat/thrusters/right/pos"]
    raw = read_bag(bag, topics)
    d = {}
    for key, topic in (("drone", "/drone/ground_truth/odometry"), ("boat", "/boat/ground_truth/odometry")):
        tb, msgs = raw[topic]
        ts, p, q, v = odometry_arrays(tb, msgs)
        w = np.array([[m.twist.twist.angular.z] for m in msgs])[:, 0]
        d[key] = dict(t_bag=tb, t=ts, p=p, q=q, v=v, r=w)
    to_sim = lambda t: np.interp(t, d["drone"]["t_bag"], d["drone"]["t"])
    for key, topic in (("uav_path", "/uav/reference_path"), ("usv_path", "/usv/reference_path")):
        tb, msgs = raw[topic]
        pts = []
        for m in msgs:
            pos = np.array([[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in m.poses])
            quat = np.array([[ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                              ps.pose.orientation.w] for ps in m.poses])
            pts.append((pos, R.from_quat(quat).as_euler("ZYX")[:, 0]))
        d[key] = dict(t=to_sim(tb), paths=pts)
    for key, topic in (("uav_solve", "/uav_solve_time_ms"), ("usv_solve", "/usv_solve_time_ms")):
        tb, msgs = raw[topic]
        d[key] = dict(t=to_sim(tb), ms=np.array([m.data for m in msgs]))
    # Commands applied online, which are part of the initial state: thrusters [T_L, T_R, alpha_L, alpha_R] and
    # UAV [phi_cmd, theta_cmd, psi_cmd, T] (columns 6, 7, 21, 8 of /mpc_states_and_inputs)
    t_cmd, cols = None, []
    for topic in ("/boat/thrusters/left/thrust", "/boat/thrusters/right/thrust", "/boat/thrusters/left/pos",
                  "/boat/thrusters/right/pos"):
        tb, msgs = raw[topic]
        n = len(msgs) if t_cmd is None else min(len(msgs), len(t_cmd))
        t_cmd = to_sim(tb)[:n] if t_cmd is None else t_cmd[:n]
        cols.append(np.array([m.data for m in msgs])[:n])
    d["usv_cmd"] = dict(t=t_cmd, cmd=np.column_stack([c[:len(t_cmd)] for c in cols]))
    tb, msgs = raw["/mpc_states_and_inputs"]
    tel = np.array([m.data[:22] for m in msgs])
    d["uav_cmd"] = dict(t=to_sim(tb), cmd=tel[:, [6, 7, 21, 8]])
    return d


def previous_command(rec, t, Ts):
    """Command applied at the previous sampling instant (the last one published before t)."""
    return rec["cmd"][latest(rec["t"], t - 0.5 * Ts)].copy()


def latest(times, t):
    return max(int(np.searchsorted(times, t, side="right")) - 1, 0)


def interp_pos(od, t):
    return np.array([np.interp(t, od["t"], od["p"][:, i]) for i in range(3)])


def extend_circle(pos, yaw, n_points, center=(0.0, 0.0)):
    """Extends a circular reference path to n_points by repeating the rotation of its last step about the centre."""
    pos, yaw = list(pos), list(yaw)
    c = np.array(center)
    a1 = np.arctan2(*(pos[-1][:2] - c)[::-1])
    a0 = np.arctan2(*(pos[-2][:2] - c)[::-1])
    step = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
    while len(pos) < n_points:
        last = pos[-1]
        rel = last[:2] - c
        cs, sn = np.cos(step), np.sin(step)
        nxt = np.array([c[0] + cs * rel[0] - sn * rel[1], c[1] + sn * rel[0] + cs * rel[1], last[2]])
        pos.append(nxt)
        yaw.append(yaw[-1] + step)
    return np.array(pos), np.array(yaw)


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ---------------------------------------------------------------------------------------------------------------
# USV replay (usv_mpc_pipeline.cpp)
# ---------------------------------------------------------------------------------------------------------------
def replay_usv(solver, N, d):
    Ts, d_max, z_t, n_uav_nodes = 0.1, 15.0, 1.25, 11
    t_uav0, t_end = d["uav_solve"]["t"][0], d["drone"]["t"][-1] - 1.0
    instants = [t for t in d["usv_solve"]["t"] if t_uav0 <= t <= t_end]
    first = True
    out = []
    for t in instants:
        b = d["boat"]
        k = latest(b["t"], t)
        yaw = R.from_quat(b["q"][k]).as_euler("ZYX")[0]
        last_cmd = previous_command(d["usv_cmd"], t, Ts)
        x0 = np.concatenate([[b["p"][k, 0], b["p"][k, 1], yaw, b["v"][k, 0], b["v"][k, 1], b["r"][k]], last_cmd])
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        if first:
            for i in range(N + 1):
                solver.set(i, "x", x0)
            first = False
        pos, psi = d["usv_path"]["paths"][latest(d["usv_path"]["t"], t)]
        if len(pos) < N + 1:
            pos, psi = extend_circle(pos, psi, N + 1)
        n = len(pos)
        for i in range(N + 1):
            j = min(i, n - 1)
            if j < n - 1:
                u_ref = ((pos[j + 1, 0] - pos[j, 0]) * np.cos(psi[j]) + (pos[j + 1, 1] - pos[j, 1]) * np.sin(psi[j])) / Ts
                r_ref = wrap(psi[j + 1] - psi[j]) / Ts
            else:
                u_ref = ((pos[j, 0] - pos[j - 1, 0]) * np.cos(psi[j - 1]) + (pos[j, 1] - pos[j - 1, 1]) * np.sin(psi[j - 1])) / Ts
                r_ref = wrap(psi[j] - psi[j - 1]) / Ts
            yref = np.zeros(14 if i < N else 10)
            yref[0], yref[1], yref[3], yref[5] = pos[j, 0], pos[j, 1], u_ref, r_ref
            solver.set(i, "yref", yref)
            active = i < n_uav_nodes
            pa = interp_pos(d["drone"], t + i * Ts) if active else np.array([0.0, 0.0, z_t])
            solver.set(i, "p", np.array([psi[j], pa[0], pa[1], pa[2], z_t]))
            if i >= 1:
                solver.constraints_set(i, "uh", np.array([d_max ** 2 if active else 1.0e6]))
        status, it, tm, res = solve_and_measure(solver)
        if status not in (0, 2):  # failure handling of the pipeline: reset and solve once more
            solver.reset()
            for i in range(N + 1):
                solver.set(i, "x", x0)
                if i < N:
                    solver.set(i, "u", np.zeros(4))
            status, it2, tm2, res = solve_and_measure(solver)
            it, tm = it + it2, tm + tm2
        cmd = last_cmd + solver.get(0, "u") * Ts if status in (0, 2) else last_cmd
        out.append((t, status, it, tm, res, *cmd))
    return np.array(out)


# ---------------------------------------------------------------------------------------------------------------
# UAV replay (uav_mpc_pipeline.cpp)
# ---------------------------------------------------------------------------------------------------------------
def replay_uav(solver, N, d):
    Ts, m, g, L, z_t = 0.02, 2.06, 9.81, 15.0, 1.25
    beta, d_bar = 0.03, 8.0
    t_end = d["drone"]["t"][-1] - 2.1
    instants = [t for t in d["uav_solve"]["t"] if t <= t_end]
    first = True
    yaw_cont = 0.0
    cmd = None
    v_pred = None
    dist = np.zeros(3)
    out = []
    for t in instants:
        od = d["drone"]
        k = latest(od["t"], t)
        rot = R.from_quat(od["q"][k])
        yaw, pitch, roll = rot.as_euler("ZYX")
        v_world = rot.apply(od["v"][k])
        cmd = previous_command(d["uav_cmd"], t, Ts)
        if first:
            yaw_cont = yaw
        else:
            yaw_cont += wrap(yaw - yaw_cont)
        cmd[2] = yaw_cont + wrap(cmd[2] - yaw_cont)  # continuous yaw command, as in the pipeline
        n_x, n_y, n_z = np.sin(pitch) * np.cos(roll), -np.sin(roll), np.cos(pitch) * np.cos(roll)
        dpsi = yaw - cmd[2]
        lx = np.cos(dpsi) * n_x - np.sin(dpsi) * n_y
        ly = np.sin(dpsi) * n_x + np.cos(dpsi) * n_y
        x0 = np.concatenate([od["p"][k], v_world, [np.arcsin(-ly), np.arctan2(lx, n_z), yaw_cont], cmd])
        if v_pred is not None:  # disturbance observer
            dist = np.clip(dist + beta * m * (v_world - v_pred) / Ts, -d_bar, d_bar)
        if first:
            for i in range(N + 1):
                solver.set(i, "x", x0)
                if i < N:
                    solver.set(i, "u", np.zeros(4))
            first = False
        else:  # shift the previous solution by one node
            us = [solver.get(i, "u") for i in range(N)]
            xs = [solver.get(i, "x") for i in range(N + 1)]
            for i in range(N - 1):
                solver.set(i, "u", us[i + 1])
            solver.set(N - 1, "u", us[N - 2])
            for i in range(N):
                solver.set(i, "x", xs[i + 1])
            solver.set(N, "x", xs[N - 1])
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        # Boat horizon (sampled every 0.1 s) and attachment point
        boat_h = np.array([interp_pos(d["boat"], t + j * 0.1)[:2] for j in range(21)])
        pos, psi = d["uav_path"]["paths"][latest(d["uav_path"]["t"], t)]
        if len(pos) < N + 1:
            pos, psi = extend_circle(pos, psi, N + 1)
        vel = np.vstack([np.diff(pos, axis=0) / Ts, (pos[-1] - pos[-2]) / Ts])
        for i in range(N + 1):
            idx = i * Ts / 0.1
            j = int(np.floor(idx))
            if j < len(boat_h) - 1:
                a = idx - j
                bp = (1 - a) * boat_h[j] + a * boat_h[j + 1]
                bv = (boat_h[j + 1] - boat_h[j]) / 0.1
            else:
                bp, bv = boat_h[-1], (boat_h[-1] - boat_h[-2]) / 0.1
            r = pos[i] - np.array([bp[0], bp[1], 0.0])
            dn = np.linalg.norm(r)
            ref = pos[i] if dn <= L else np.array([bp[0], bp[1], 0.0]) + r * L / dn
            yref = np.zeros(15 if i < N else 11)
            yref[0:3], yref[3:6] = ref, vel[i]
            solver.set(i, "yref", yref)
            solver.set(i, "p", np.array([bp[0], bp[1], z_t, bv[0], bv[1], 0.0, bp[0], bp[1], 1.0, *dist]))
            if i >= 1:
                solver.constraints_set(i, "lh", np.array([0.0, -20.0, 0.0]))
                solver.constraints_set(i, "uh", np.array([L ** 2, 2.0, 10.0]))
        status, it, tm, res = solve_and_measure(solver)
        if status == 0:
            x1 = solver.get(1, "x")
            cmd = x1[9:13].copy()
            v_pred = x1[3:6].copy()
        else:
            v_pred = None
        out.append((t, status, it, tm, res, *cmd))
    return np.array(out)


# ---------------------------------------------------------------------------------------------------------------
def run(vehicle, name, bag):
    solver, N = build_solver(vehicle, name)
    data = load_bag(bag)
    res = (replay_usv if vehicle == "usv" else replay_uav)(solver, N, data)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    np.savez(os.path.join(RESULTS_DIR, f"{vehicle}_{name}.npz"), t=res[:, 0], status=res[:, 1], iters=res[:, 2],
             time_ms=1e3 * res[:, 3], kkt=res[:, 4], cmd=res[:, 5:], N=N, recorded_ms=data[f"{vehicle}_solve"]["ms"])
    tm = 1e3 * res[:, 3]
    print(f"{vehicle}/{name}: {len(res)} instants | time mean {tm.mean():.2f} p99 {np.percentile(tm, 99):.2f} "
          f"max {tm.max():.2f} ms | iters mean {res[:, 2].mean():.1f} max {res[:, 2].max():.0f} | "
          f"KKT median {np.median(res[:, 4]):.1e} | status {dict(zip(*np.unique(res[:, 1], return_counts=True)))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "all", "plot"])
    ap.add_argument("--vehicle", choices=["usv", "uav"])
    ap.add_argument("--config")
    ap.add_argument("--bag", default=DEFAULT_BAG)
    a = ap.parse_args()
    if a.cmd == "run":
        run(a.vehicle, a.config, a.bag)
    elif a.cmd == "all":
        for vehicle, cfgs in CONFIGS.items():
            for name in cfgs:
                subprocess.run([sys.executable, __file__, "run", "--vehicle", vehicle, "--config", name, "--bag", a.bag],
                               check=True)
    else:
        from plot_solver_benchmark import plot_all
        plot_all(RESULTS_DIR, FIG_DIR, CONFIGS, HORIZON_SWEEP, TS)


if __name__ == "__main__":
    main()
