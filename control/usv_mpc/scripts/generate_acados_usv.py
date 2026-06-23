import os
import sys
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# Add current directory to path to import wamv_model
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import wamv_model

def create_usv_dynamic_model() -> AcadosModel:
    states, states_dot, controls, f_expl, f_impl = wamv_model.get_wamv_casadi_model()
    
    model = AcadosModel()
    model.name = 'usv_dynamic'
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = states_dot
    model.u = controls
    
    return model

def generate_acados_ocp():
    model = create_usv_dynamic_model()
    
    ocp = AcadosOcp()
    ocp.model = model
    
    # prediction horizon (N steps, control period Ts)
    ocp.dims.N = 20
    Ts = 0.1
    ocp.solver_options.tf = Ts * ocp.dims.N
    
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    # Cost: Minimize tracking error (LINEAR_LS)
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Weights (Q for states, R for controls)
    Qx = 150.0    # Heavily increased to eliminate position lag
    Qy = 150.0    # Heavily increased to eliminate position lag
    Qpsi = 150.0  # Keep heading aligned tangent to circle
    Qu = 2.0
    Qv = 40.0     # Suppress sway sliding
    Qr = 1.0
    
    # Control input penalties (regularization)
    Rx = 0.0001   # Greatly reduced to allow necessary thrust to overcome drag
    Ry = 0.001    # Reduced to allow necessary sway correction
    Rn = 0.0005   # Reduced to allow quick steering response
    
    ocp.cost.W = np.diag([Qx, Qy, Qpsi, Qu, Qv, Qr, Rx, Ry, Rn])
    ocp.cost.W_e = np.diag([Qx, Qy, Qpsi, Qu, Qv, Qr])
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Virtual Force/Moment Constraints (Fossen model control inputs)
    # Estimated maximum thrust commands mapped to CG
    x_max = 4700.0   # N (forward max thrust: 2 * 2350 N)
    x_min = -3506.0  # N (backward max thrust with efficiency: 2 * -2350 * 0.746 N)
    y_max = 2000.0   # N (sway max force, conservative limit for roll stability)
    n_max = 3000.0   # N*m (yaw max moment, conservative limit)
    
    ocp.constraints.lbu = np.array([x_min, -y_max, -n_max])
    ocp.constraints.ubu = np.array([x_max, y_max, n_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    ocp.constraints.x0 = np.zeros(nx)
    
    # Solver Options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    
    ocp.code_export_directory = '../acados_solver'
    
    return ocp

if __name__ == '__main__':
    ocp = generate_acados_ocp()
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp_usv.json')
    print("Sucesso! Código C dinâmico gerado na pasta 'acados_solver'.")
