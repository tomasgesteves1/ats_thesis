import os
import sys
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def create_usv_kinematic_model() -> AcadosModel:
    model_name = 'usv_kinematic'

    # Estados: posição (x, y) e orientação (psi)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    psi = ca.SX.sym('psi')
    state = ca.vertcat(x, y, psi)

    # Controlos: velocidade linear (v) e velocidade angular (w)
    v = ca.SX.sym('v')
    w = ca.SX.sym('w')
    control = ca.vertcat(v, w)

    # Dinâmica Cinemática do Uniciclo (Diferencial / Skid-Steer)
    xdot = ca.SX.sym('xdot')
    ydot = ca.SX.sym('ydot')
    psidot = ca.SX.sym('psidot')
    state_dot = ca.vertcat(xdot, ydot, psidot)

    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        w
    )
    f_impl = state_dot - f_expl

    model = AcadosModel()
    model.name = model_name
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = state
    model.xdot = state_dot
    model.u = control

    return model

def generate_acados_ocp():
    model = create_usv_kinematic_model()
    
    ocp = AcadosOcp()
    ocp.model = model
    
    # Horizonte de predição (ex: olhar 2 segundos para a frente com dt=0.1)
    ocp.dims.N = 20
    Ts = 0.1
    ocp.solver_options.tf = Ts * ocp.dims.N
    
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    # Custo: Minimizar erro para a trajetória (LINEAR_LS)
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Pesos (Q para estados, R para comandos)
    Qx = 10.0
    Qy = 10.0
    Qpsi = 0.5  # O barco não precisa de estar perfeitamente virado se não estiver a andar
    Rv = 1.0    # Penalizar mudanças bruscas de velocidade
    Rw = 1.0
    
    ocp.cost.W = np.diag([Qx, Qy, Qpsi, Rv, Rw])
    ocp.cost.W_e = np.diag([Qx, Qy, Qpsi])
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Limites (Constraints) do WAM-V
    v_max = 3.0   # m/s
    w_max = 1.0   # rad/s
    
    ocp.constraints.lbu = np.array([-v_max, -w_max])
    ocp.constraints.ubu = np.array([v_max, w_max])
    ocp.constraints.idxbu = np.array([0, 1])
    
    ocp.constraints.x0 = np.zeros(nx)
    
    # Opções do Solver
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    
    ocp.code_export_directory = '../acados_solver'
    
    return ocp

if __name__ == '__main__':
    ocp = generate_acados_ocp()
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp_usv.json')
    print("Sucesso! Código C cinemático gerado na pasta 'acados_solver'.")
