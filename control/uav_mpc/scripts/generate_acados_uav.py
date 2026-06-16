import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def create_uav_model() -> AcadosModel:
    model_name = 'uav_tethered'

    # Gravitational acceleration
    g_vec = ca.vertcat(0.0, 0.0, -9.81)
    
    # States (x): [p_x, p_y, p_z, v_x, v_y, v_z]
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    x = ca.vertcat(p, v)

    # Controls (u): [phi_cmd, theta_cmd, a_T]
    phi_cmd = ca.SX.sym('phi_cmd', 1)
    theta_cmd = ca.SX.sym('theta_cmd', 1)
    a_T = ca.SX.sym('a_T', 1)
    u = ca.vertcat(phi_cmd, theta_cmd, a_T)

    # Parameters (p): [psi] (yaw angle of the drone)
    psi = ca.SX.sym('psi', 1)

    # Thrust acceleration in world frame (ENU) assuming instant attitude tracking
    ax_thrust = a_T * (ca.cos(psi) * ca.sin(theta_cmd) * ca.cos(phi_cmd) + ca.sin(psi) * ca.sin(phi_cmd))
    ay_thrust = a_T * (ca.sin(psi) * ca.sin(theta_cmd) * ca.cos(phi_cmd) - ca.cos(psi) * ca.sin(phi_cmd))
    az_thrust = a_T * (ca.cos(theta_cmd) * ca.cos(phi_cmd))
    a_thrust = ca.vertcat(ax_thrust, ay_thrust, az_thrust)

    # State derivatives
    xdot_expr = ca.vertcat(
        v,
        a_thrust + g_vec
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
    # Load model
    model = create_uav_model()
    
    # Configure Optimal Control Problem (OCP)
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = 20  # Prediction Horizon
    Ts = 0.05
    ocp.solver_options.tf = Ts * ocp.dims.N
    
    # Configure Costs (LINEAR_LS)
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Weight matrices
    ocp.cost.W = np.diag([15.0, 15.0, 20.0, 2.0, 2.0, 2.0, 0.05, 0.05, 0.01])
    ocp.cost.W_e = np.diag([15.0, 15.0, 20.0, 2.0, 2.0, 2.0])
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    # Default references
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref[8] = 9.81  # Default a_T cancels gravity
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Default parameters: [psi]
    ocp.parameter_values = np.array([0.0])
    
    # Control input bounds [phi_cmd, theta_cmd, a_T]
    phi_max = 0.2       # ~11.5 deg maximum tilt (less aggressive)
    theta_max = 0.2
    a_T_min = 0.1 * 9.81
    a_T_max = 15.0      # Less aggressive thrust limit
    
    ocp.constraints.lbu = np.array([-phi_max, -theta_max, a_T_min])
    ocp.constraints.ubu = np.array([phi_max, theta_max, a_T_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # State bounds [v_x, v_y, v_z]
    v_max = 2.0         # Less aggressive velocity limit
    ocp.constraints.lbx = np.array([-v_max, -v_max, -v_max])
    ocp.constraints.ubx = np.array([v_max, v_max, v_max])
    ocp.constraints.idxbx = np.array([3, 4, 5])
    
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
