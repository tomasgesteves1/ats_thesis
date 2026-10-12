import os
import sys
import numpy as np
import time

def test_uav_mpc_solver():
    print("=== TEST 1: UAV NMPC Solver with Winch Rate Constraint ===")
    from acados_template import AcadosOcpSolver
    # Import the generated solver json from uav_mpc
    json_path = os.path.join(os.path.dirname(__file__), 'acados_ocp.json')
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        return False

    solver = AcadosOcpSolver(None, json_file=json_path, generate=False, build=False)
    print("Acados UAV OCP Solver successfully initialized.")

    N = 50
    # State: [px, py, pz, vx, vy, vz, phi, theta, phi_cmd, theta_cmd]
    # Parameters: [psi, xb, yb, zb, vxb, vyb, vzb]
    
    # Test Scenario A: Drone hovering at (0, 0, 5), boat at (0, 0, 1.5), stationary
    x0 = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)

    # Set parameters for all stages
    p_stat = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    for i in range(N + 1):
        solver.set(i, "p", p_stat)
        solver.set(i, "x", x0)
    for i in range(N):
        solver.set(i, "u", np.array([0.0, 0.0, 9.81]))

    # Tether constraint: max distance 15m (15^2 = 225), max radial sep speed 2.0 m/s
    lh = np.array([0.0, -20.0])
    uh = np.array([225.0, 2.0])
    for i in range(1, N):
        solver.constraints_set(i, "lh", lh)
        solver.constraints_set(i, "uh", uh)
    solver.constraints_set(N, "lh", lh)
    solver.constraints_set(N, "uh", uh)

    # Reference: hold at (0, 0, 5)
    yref = np.zeros(13)
    yref[0] = 0.0
    yref[1] = 0.0
    yref[2] = 5.0
    yref[12] = 9.81
    for i in range(N):
        solver.cost_set(i, "yref", yref)
    yref_e = np.zeros(10)
    yref_e[0] = 0.0
    yref_e[1] = 0.0
    yref_e[2] = 5.0
    solver.cost_set(N, "yref", yref_e)

    # Solve
    t0 = time.time()
    status = solver.solve()
    dt = (time.time() - t0) * 1000.0
    print(f"Scenario A (Hover): status = {status}, time = {dt:.2f} ms")
    assert status == 0, f"Solver failed in hover test: {status}"

    u0 = solver.get(0, "u")
    print(f"Optimal control u0 = {u0} (Thrust = {u0[2]:.2f} m/s^2, expected ~ 9.81)")
    assert abs(u0[2] - 9.81) < 0.5, "Thrust command deviates significantly from gravity compensation"

    # Test Scenario B: Drone ordered to move far away (X=30m), which would violate tether length and winch speed
    # Reference at (30, 0, 5)
    yref[0] = 30.0
    for i in range(N):
        solver.cost_set(i, "yref", yref)
    yref_e[0] = 30.0
    solver.cost_set(N, "yref", yref_e)

    t0 = time.time()
    status = solver.solve()
    dt = (time.time() - t0) * 1000.0
    print(f"Scenario B (High step away): status = {status}, time = {dt:.2f} ms")
    assert status == 0, f"Solver failed in constrained step test: {status}"

    # Verify predicted trajectory respects v_sep <= 2.0 m/s and d <= 15m
    max_pred_dist = 0.0
    max_v_sep = -100.0
    for i in range(N + 1):
        xi = solver.get(i, "x")
        r = xi[0:3] - p_stat[1:4]
        v_rel = xi[3:6] - p_stat[4:7]
        dist = np.linalg.norm(r)
        v_sep = np.dot(r, v_rel) / np.sqrt(dist**2 + 0.01)
        if dist > max_pred_dist:
            max_pred_dist = dist
        if v_sep > max_v_sep:
            max_v_sep = v_sep

    print(f"Predicted max distance: {max_pred_dist:.2f} m (limit: 15.0 m)")
    print(f"Predicted max separation speed: {max_v_sep:.2f} m/s (winch limit: 2.0 m/s)")
    assert max_pred_dist <= 15.1, f"Tether distance violated: {max_pred_dist}"
    assert max_v_sep <= 2.1, f"Winch separation speed violated: {max_v_sep}"

    # Test Scenario C: Drone moving tangentially around the boat (boat at origin, drone at (10, 0, 5), commanded to (10, 10, 5))
    # In tangential motion, v_sep is near 0, so drone velocity can be high (> 2.0 m/s)
    x0_tang = np.array([10.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    solver.set(0, "lbx", x0_tang)
    solver.set(0, "ubx", x0_tang)
    yref_tang = np.array([10.0, 10.0, 5.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81])
    for i in range(N):
        solver.cost_set(i, "yref", yref_tang)
    yref_e_tang = np.array([10.0, 10.0, 5.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    solver.cost_set(N, "yref", yref_e_tang)

    status = solver.solve()
    assert status == 0, f"Solver failed in tangential motion: {status}"
    
    # Check tangential speed reached in predicted horizon
    max_tang_speed = 0.0
    for i in range(N + 1):
        xi = solver.get(i, "x")
        vy = xi[4]
        if vy > max_tang_speed:
            max_tang_speed = vy
    print(f"Scenario C (Tangential flight): max Vy achieved = {max_tang_speed:.2f} m/s (exceeds winch 2.0m/s as allowed)")
    assert max_tang_speed > 2.0, "UAV was unnecessarily restricted in tangential direction!"

    print(">>> UAV MPC Solver PASSED all validation tests! <<<\n")
    return True

def test_usv_mpc_solver():
    print("=== TEST 2: USV NMPC Solver with Validated Model ===")
    from acados_template import AcadosOcpSolver
    json_path = os.path.join(os.path.dirname(__file__), '../../usv_mpc/scripts/acados_ocp_usv.json')
    if not os.path.exists(json_path):
        print(f"Warning: {json_path} not found")
        return False

    solver = AcadosOcpSolver(None, json_file=json_path, generate=False, build=False)
    print("Acados USV OCP Solver successfully initialized.")

    N = 20
    # State: [x, y, psi, u, v, r]
    # Controls: [Fx, Fy, Mz]
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)

    # Command forward motion u = 2.0 m/s
    yref = np.zeros(9)
    yref[0] = 5.0
    yref[3] = 2.0
    for i in range(N):
        solver.cost_set(i, "yref", yref)
    yref_e = np.zeros(6)
    yref_e[0] = 5.0
    yref_e[3] = 2.0
    solver.cost_set(N, "yref", yref_e)

    t0 = time.time()
    status = solver.solve()
    dt = (time.time() - t0) * 1000.0
    print(f"USV Surge Test: status = {status}, time = {dt:.2f} ms")
    assert status == 0, f"USV Solver failed: {status}"

    u0 = solver.get(0, "u")
    print(f"Optimal control u0 = Fx: {u0[0]:.2f} N, Fy: {u0[1]:.2f} N, Mz: {u0[2]:.2f} Nm")
    assert u0[0] > 0.0, "USV should command positive forward thrust"

    print(">>> USV MPC Solver PASSED all validation tests! <<<\n")
    return True

if __name__ == '__main__':
    ok_uav = test_uav_mpc_solver()
    ok_usv = test_usv_mpc_solver()
    if ok_uav and ok_usv:
        print("ALL CONTROLLER SOLVER TESTS PASSED SUCCESSFULLY!")
        sys.exit(0)
    else:
        sys.exit(1)
