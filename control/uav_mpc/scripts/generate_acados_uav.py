import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import json
import os

def create_uav_model() -> AcadosModel:
    model_name = 'uav_tethered'

    # Gravitational acceleration
    g_vec = ca.vertcat(0.0, 0.0, -9.81)
    
    # States (x): [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, phi_cmd, theta_cmd]
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    phi = ca.SX.sym('phi', 1)
    theta = ca.SX.sym('theta', 1)
    phi_cmd = ca.SX.sym('phi_cmd', 1)
    theta_cmd = ca.SX.sym('theta_cmd', 1)
    x = ca.vertcat(p, v, phi, theta, phi_cmd, theta_cmd)

    # Controls (u): [phi_dot_cmd, theta_dot_cmd, a_T]
    phi_dot_cmd = ca.SX.sym('phi_dot_cmd', 1)
    theta_dot_cmd = ca.SX.sym('theta_dot_cmd', 1)
    a_T = ca.SX.sym('a_T', 1)
    u = ca.vertcat(phi_dot_cmd, theta_dot_cmd, a_T)

    # Parameters (p): [psi] (yaw angle of the drone)
    psi = ca.SX.sym('psi', 1)

    # Thrust acceleration in world frame (ENU) using actual model states phi and theta
    ax_thrust = a_T * (ca.cos(psi) * ca.sin(theta) * ca.cos(phi) + ca.sin(psi) * ca.sin(phi))
    ay_thrust = a_T * (ca.sin(psi) * ca.sin(theta) * ca.cos(phi) - ca.cos(psi) * ca.sin(phi))
    az_thrust = a_T * (ca.cos(theta) * ca.cos(phi))
    a_thrust = ca.vertcat(ax_thrust, ay_thrust, az_thrust)

    # State derivatives (dynamics with tau = 0.15s and state augmentation)
    tau = 0.15
    xdot_expr = ca.vertcat(
        v,
        a_thrust + g_vec,
        (phi_cmd - phi) / tau,
        (theta_cmd - theta) / tau,
        phi_dot_cmd,
        theta_dot_cmd
    )

    # ACADOS model formulation
    model = AcadosModel()
    model.f_expl_expr = xdot_expr
    model.x = x
    model.u = u
    model.p = psi
    model.name = model_name

    return model

def generate_acados_ocp():
    # Load config file relative to the script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '../config/acados_generator_config.json')
    with open(config_path, 'r') as f:
        config = json.load(f)

    # Load model
    model = create_uav_model()
    
    # Configure Optimal Control Problem (OCP)
    ocp = AcadosOcp()
    ocp.model = model
    
    # Horizon dimensions from config
    ocp.dims.N = config["prediction_horizon"]["N"]
    Ts = config["prediction_horizon"]["Ts"]
    ocp.solver_options.tf = Ts * ocp.dims.N
    
    # Configure Costs (LINEAR_LS)
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Weight matrices from config
    Qp = config["weights"]["Qp"]
    Qv = config["weights"]["Qv"]
    R = config["weights"]["R"]
    Q_tilt = config["weights"].get("Qtilt", [0.0, 0.0])  # Penalization on attitude states (phi, theta)
    Q_tilt_cmd = [0.0, 0.0]  # Penalization on attitude commands (phi_cmd, theta_cmd)
    ocp.cost.W = np.diag(Qp + Qv + Q_tilt + Q_tilt_cmd + R)
    ocp.cost.W_e = np.diag(Qp + Qv + Q_tilt + Q_tilt_cmd)
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    # Default references
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref[12] = 9.81  # Default a_T cancels gravity (index 12 for thrust input in 13-dimensional yref)
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Default parameters: [psi]
    ocp.parameter_values = np.array([0.0])
    
    # Control input bounds [phi_dot_cmd, theta_dot_cmd, a_T] from config
    phi_dot_max = config["limits"]["phi_dot_max"]
    theta_dot_max = config["limits"]["theta_dot_max"]
    a_T_min = config["limits"]["a_T_min_scale"] * 9.81
    a_T_max = config["limits"]["a_T_max_scale"] * 9.81
    
    ocp.constraints.lbu = np.array([-phi_dot_max, -theta_dot_max, a_T_min])
    ocp.constraints.ubu = np.array([phi_dot_max, theta_dot_max, a_T_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # State bounds [v_x, v_y, v_z, phi, theta, phi_cmd, theta_cmd] from config
    v_max = config["limits"]["v_max"]
    phi_max = config["limits"]["phi_max"]
    theta_max = config["limits"]["theta_max"]
    ocp.constraints.lbx = np.array([-v_max, -v_max, -v_max, -phi_max, -theta_max, -phi_max, -theta_max])
    ocp.constraints.ubx = np.array([v_max, v_max, v_max, phi_max, theta_max, phi_max, theta_max])
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8, 9])
    
    # Soft constraints for states: soften all 7 box constraints
    ocp.constraints.idxsbx = np.array([0, 1, 2, 3, 4, 5, 6])
    
    # Slack variables penalty costs
    ns = 7
    slack_weights = config.get("slack_weights", {"zl": 100.0, "zu": 100.0, "Zl": 1000.0, "Zu": 1000.0})
    zl_val = slack_weights.get("zl", 100.0)
    zu_val = slack_weights.get("zu", 100.0)
    Zl_val = slack_weights.get("Zl", 1000.0)
    Zu_val = slack_weights.get("Zu", 1000.0)
    
    ocp.cost.zl = zl_val * np.ones((ns,))
    ocp.cost.zu = zu_val * np.ones((ns,))
    ocp.cost.Zl = Zl_val * np.ones((ns,))
    ocp.cost.Zu = Zu_val * np.ones((ns,))
    
    # Default initial state
    ocp.constraints.x0 = np.zeros(nx)
    
    # ACADOS Solver Options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'  # Runge-Kutta
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Real-Time Iteration
    
    # Export directory
    ocp.code_export_directory = '../acados_solver'
    
    return ocp

if __name__ == '__main__':
    ocp = generate_acados_ocp()
    # Generate native C code
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("Success! C code generated in 'acados_solver' folder.")
