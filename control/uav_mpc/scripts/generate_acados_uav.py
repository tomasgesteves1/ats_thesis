import os
import sys
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def create_uav_model() -> AcadosModel:
    model_name = 'uav_tethered'

    # Constantes físicas
    m = 2.06
    g_vec = ca.vertcat(0.0, 0.0, -9.81)
    
    # Estados (x): [p_x, p_y, p_z, v_x, v_y, v_z, phi (roll real), theta (pitch real)]
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    phi = ca.SX.sym('phi', 1)
    theta = ca.SX.sym('theta', 1)
    x = ca.vertcat(p, v, phi, theta)

    # Controlos (u): [phi_cmd, theta_cmd, a_T (thrust acceleration)]
    phi_cmd = ca.SX.sym('phi_cmd', 1)
    theta_cmd = ca.SX.sym('theta_cmd', 1)
    a_T = ca.SX.sym('a_T', 1)
    u = ca.vertcat(phi_cmd, theta_cmd, a_T)

    # Parâmetros (p) - Variáveis que podem ser alteradas em tempo de execução
    p_anchor = ca.SX.sym('p_anchor', 3)
    T0 = ca.SX.sym('T0', 1)
    wc = ca.SX.sym('wc', 1)
    eps = ca.SX.sym('eps', 1)
    psi = ca.SX.sym('psi', 1) # yaw angle of the drone
    tau = ca.SX.sym('tau', 1) # attitude response time constant
    
    p_params = ca.vertcat(p_anchor, T0, wc, eps, psi, tau)

    # Thrust acceleration in world frame (ENU) using Z-Y-X rotation sequence on actual state angles
    ax_thrust = a_T * (ca.cos(psi) * ca.sin(theta) * ca.cos(phi) + ca.sin(psi) * ca.sin(phi))
    ay_thrust = a_T * (ca.sin(psi) * ca.sin(theta) * ca.cos(phi) - ca.cos(psi) * ca.sin(phi))
    az_thrust = a_T * (ca.cos(theta) * ca.cos(phi))
    a_thrust = ca.vertcat(ax_thrust, ay_thrust, az_thrust)

    # Dinâmica baseada na força do cabo a apontar verticalmente para baixo
    s = p_anchor - p
    s_norm_eps = ca.sqrt(ca.dot(s, s) + eps**2)
    tether_mag = (T0 + wc * s_norm_eps) / m
    a_tether = ca.vertcat(0.0, 0.0, -tether_mag)

    # Arrasto aerodinâmico (cd estimado em 0.15 para compensar mismatch)
    cd = 0.15
    a_drag = - cd * v
    
    # Dinâmica de atitude de primeira ordem
    dphi = (phi_cmd - phi) / ca.fmax(tau, 0.01)
    dtheta = (theta_cmd - theta) / ca.fmax(tau, 0.01)

    xdot_expr = ca.vertcat(
        v,
        a_thrust + g_vec + a_tether + a_drag,
        dphi,
        dtheta
    )

    # Formulação do modelo ACADOS
    f_impl = ca.SX.sym('xdot', 8) - xdot_expr

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = xdot_expr
    model.x = x
    model.xdot = ca.SX.sym('xdot', 8)
    model.u = u
    model.p = p_params
    model.name = model_name

    # Restrições Não Lineares (Tamanho do cabo ||p - p_anchor||^2)
    dp = p - p_anchor
    dist_sq = ca.dot(dp, dp)
    model.con_h_expr = ca.vertcat(dist_sq)

    return model

def generate_acados_ocp():
    # Carregar Modelo
    model = create_uav_model()
    
    # Configurar o Problema de Controlo Ótimo (OCP)
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = 20  # Horizonte
    Ts = 0.05
    ocp.solver_options.tf = Ts * ocp.dims.N
    
    # Configurar Custos (LINEAR_LS)
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Matrizes de peso - Sintonizadas fisicamente de acordo com as escalas
    Qp = np.diag([15.0, 15.0, 20.0])   # Penalização de posição (Z mais apertado para manter altitude)
    Qv = np.diag([2.0, 2.0, 2.0])      # Penalização de velocidade para amortecimento (damping)
    Qa = np.diag([0.1, 0.1])           # Penalização de atitude real para evitar inclinações extremas desnecessárias
    R = np.diag([0.05, 0.05, 0.01])    # Penalização de comandos: thrust em m/s^2 é menos penalizado devido à escala (9.81 default)
    
    W = np.zeros((ny, ny))
    W[0:3, 0:3] = Qp
    W[3:6, 3:6] = Qv
    W[6:8, 6:8] = Qa
    W[8:11, 8:11] = R
    ocp.cost.W = W
    
    W_e = np.zeros((ny_e, ny_e))
    W_e[0:3, 0:3] = Qp
    W_e[3:6, 3:6] = Qv
    W_e[6:8, 6:8] = Qa
    ocp.cost.W_e = W_e
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    # Referências default (serão atualizadas pelo C++ a cada loop)
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref[10] = 9.81  # Referência default para a_T é cancelar a gravidade
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Valores default para os parâmetros [p_anchor_x, y, z, T0, wc, eps, psi, tau]
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 3.0, 0.2, 0.02, 0.0, 0.15])
    
    # Limites das entradas [phi_cmd, theta_cmd, a_T]
    phi_max = 0.4       # ~23 graus de inclinação máxima
    theta_max = 0.4
    a_T_min = 0.1 * 9.81
    a_T_max = 2.0 * 9.81
    v_max = 10.0
    L_max = 30.0
    
    ocp.constraints.lbu = np.array([-phi_max, -theta_max, a_T_min])
    ocp.constraints.ubu = np.array([phi_max, theta_max, a_T_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # Limites dos estados [v_x, v_y, v_z, phi, theta]
    ocp.constraints.lbx = np.array([-v_max, -v_max, -v_max, -phi_max, -theta_max])
    ocp.constraints.ubx = np.array([v_max, v_max, v_max, phi_max, theta_max])
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7])
    
    # Limites da restrição não linear do cabo (lh <= h <= uh)
    ocp.constraints.lh = np.array([-10.0])
    ocp.constraints.uh = np.array([L_max**2])
    
    # Estado inicial default
    ocp.constraints.x0 = np.zeros(nx)
    
    # A Magia do ACADOS: O Solver!
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'  # Runge-Kutta
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Real-Time Iteration (Mega rápido)
    
    # Onde gravar os ficheiros C
    ocp.code_export_directory = '../acados_solver'
    
    return ocp

if __name__ == '__main__':
    ocp = generate_acados_ocp()
    # Gera o código C nativo
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("Sucesso! Código C gerado na pasta 'acados_solver'. Podes fechar o Python agora!")
