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
    
    # Estados (x)
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    x = ca.vertcat(p, v)

    # Controlos (u)
    u = ca.SX.sym('u', 3)

    # Parâmetros (p) - Variáveis que podem ser alteradas em tempo de execução
    p_anchor = ca.SX.sym('p_anchor', 3)
    T0 = ca.SX.sym('T0', 1)
    wc = ca.SX.sym('wc', 1)
    eps = ca.SX.sym('eps', 1)
    
    p_params = ca.vertcat(p_anchor, T0, wc, eps)

    # Dinâmica baseada na força do cabo a apontar verticalmente para baixo
    s = p_anchor - p
    s_norm_eps = ca.sqrt(ca.dot(s, s) + eps**2)
    tether_mag = (T0 + wc * s_norm_eps) / m
    a_tether = ca.vertcat(0.0, 0.0, -tether_mag)
    
    xdot_expr = ca.vertcat(
        v,
        u + g_vec + a_tether
    )

    # Formulação do modelo ACADOS
    f_impl = ca.SX.sym('xdot', 6) - xdot_expr

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = xdot_expr
    model.x = x
    model.xdot = ca.SX.sym('xdot', 6)
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
    
    # Configurar Custos (LINEAR_LS equivale ao teu cost.py)
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # Matrizes de peso (do parameters.py) - Suavizadas para evitar agressividade extrema
    Qp = 5.0 * np.eye(3)
    Qv = 2.0 * np.eye(3)
    R = 0.5 * np.eye(3)
    
    ocp.cost.W = np.block([
        [Qp, np.zeros((3,6))],
        [np.zeros((3,3)), Qv, np.zeros((3,3))],
        [np.zeros((3,6)), R]
    ])
    ocp.cost.W_e = np.block([
        [Qp, np.zeros((3,3))],
        [np.zeros((3,3)), Qv]
    ])
    
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    
    # Referências default (serão atualizadas pelo C++ a cada loop)
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref[8] = 9.81  # Referência default para u_z é cancelar a gravidade
    ocp.cost.yref_e = np.zeros((ny_e,))
    
    # Valores default para os parâmetros [p_anchor_x, y, z, T0, wc, eps]
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 3.0, 0.2, 0.02])
    
    # Limites
    u_max = 2 * 9.81
    v_max = 1.5
    L_max = 30.0
    
    # O drone NÃO PODE puxar-se para baixo! u_z >= 0
    ocp.constraints.lbu = np.array([-u_max, -u_max, 0.0])
    ocp.constraints.ubu = np.array([u_max, u_max, u_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    ocp.constraints.lbx = np.array([-v_max, -v_max, -v_max])
    ocp.constraints.ubx = np.array([v_max, v_max, v_max])
    ocp.constraints.idxbx = np.array([3, 4, 5])
    
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
