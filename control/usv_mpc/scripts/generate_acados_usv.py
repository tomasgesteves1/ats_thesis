import os
import sys
import json
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# Add current directory to path to import wamv_model
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import wamv_model


def create_usv_dynamic_model() -> AcadosModel:
    states, states_dot, controls, params, f_expl, f_impl = wamv_model.get_wamv_casadi_model()

    model = AcadosModel()
    model.name = 'usv_dynamic'
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.p = params

    # Residuals of the least-squares cost (Eq. usv_mpc_stage_cost):
    # [x, y, sin((psi-psi_ref)/2), u, v, r, T_L, T_R, alpha_L, alpha_R, dT_L, dT_R, dalpha_L, dalpha_R]
    # q_psi*(1-cos(d)) = 2*q_psi*sin^2(d/2), so the heading term is a squared residual with weight 2*q_psi.
    psi_ref = params[0]
    heading_res = ca.sin((states[2] - psi_ref) / 2.0)
    model.cost_y_expr = ca.vertcat(states[0:2], heading_res, states[3:10], controls)
    model.cost_y_expr_e = ca.vertcat(states[0:2], heading_res, states[3:10])

    # Tether: squared distance between the attachment point (x, y, z_t) and the predicted UAV position
    x_a, y_a, z_a, z_t = params[1], params[2], params[3], params[4]
    d_sq = (states[0] - x_a) ** 2 + (states[1] - y_a) ** 2 + (z_a - z_t) ** 2
    model.con_h_expr = ca.vertcat(d_sq)
    model.con_h_expr_e = ca.vertcat(d_sq)

    return model


def generate_acados_ocp():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(script_dir, '../config/acados_generator_config.json'), 'r') as f:
        config = json.load(f)

    model = create_usv_dynamic_model()

    ocp = AcadosOcp()
    ocp.model = model

    # Prediction horizon (N steps, sampling time Ts)
    ocp.dims.N = config["prediction_horizon"]["N"]
    Ts = config["prediction_horizon"]["Ts"]
    ocp.solver_options.tf = Ts * ocp.dims.N

    # Cost weights (Section 3.3): Qp, q_psi, Qnu, Qtau, R
    w = config["weights"]
    W_state = list(w["Qp"]) + [2.0 * w["q_psi"]] + list(w["Qnu"]) + list(w["Qtau"])
    W_input = list(w["R"])
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag(W_state + W_input)
    ocp.cost.W_e = np.diag(W_state)
    ocp.cost.yref = np.zeros((len(W_state) + len(W_input),))
    ocp.cost.yref_e = np.zeros((len(W_state),))

    # Parameters per node: [psi_ref, x_a, y_a, z_a, z_t]
    ocp.parameter_values = np.zeros(wamv_model.NP)

    # Thruster range limits (state) and rate limits (input)
    lim = config["limits"]
    a_max = np.deg2rad(lim["alpha_max_deg"])
    # Surge speed bound u >= u_min (soft): the boat may back off slowly to hold or brake, but not sail astern fast
    # (without it, a sudden change of the reference makes it turn the stern and back towards the target)
    ocp.constraints.lbx = np.array([lim["u_min"], lim["T_min"], lim["T_min"], -a_max, -a_max])
    ocp.constraints.ubx = np.array([lim["u_max"], lim["T_max"], lim["T_max"], a_max, a_max])
    ocp.constraints.idxbx = np.array([3, 6, 7, 8, 9])
    ocp.constraints.idxsbx = np.array([0])

    ad_max = np.deg2rad(lim["alpha_dot_max_deg"])
    ocp.constraints.lbu = np.array([-lim["T_dot_max"], -lim["T_dot_max"], -ad_max, -ad_max])
    ocp.constraints.ubu = np.array([lim["T_dot_max"], lim["T_dot_max"], ad_max, ad_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.x0 = np.zeros(wamv_model.NX)

    # Soft tether constraint d^2 <= d_max^2 (bounds updated at run time)
    d_max = config["tether"]["d_max_default"]
    ocp.constraints.lh = np.array([0.0])
    ocp.constraints.uh = np.array([d_max ** 2])
    ocp.constraints.lh_e = np.array([0.0])
    ocp.constraints.uh_e = np.array([d_max ** 2])
    ocp.constraints.idxsh = np.array([0])
    ocp.constraints.idxsh_e = np.array([0])

    # Slack penalties rho(sigma) = mu1*sigma + mu2*sigma^2 (acados: z*s + 0.5*Z*s^2, so Z = 2*mu2). The slacks of
    # the stages are ordered [surge bound, tether]; the terminal stage has only the tether
    mu1 = config["slack_weights"]["mu1"]
    mu2 = config["slack_weights"]["mu2"]
    mu1_u = config["slack_weights_surge"]["mu1"]
    mu2_u = config["slack_weights_surge"]["mu2"]
    ocp.cost.zl = np.array([mu1_u, mu1])
    ocp.cost.zu = np.array([mu1_u, mu1])
    ocp.cost.Zl = 2.0 * np.array([mu2_u, mu2])
    ocp.cost.Zu = 2.0 * np.array([mu2_u, mu2])
    ocp.cost.zl_e = mu1 * np.ones(1)
    ocp.cost.zu_e = mu1 * np.ones(1)
    ocp.cost.Zl_e = 2.0 * mu2 * np.ones(1)
    ocp.cost.Zu_e = 2.0 * mu2 * np.ones(1)

    # Solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 25
    ocp.solver_options.tol = 1e-4

    ocp.code_export_directory = '../acados_solver'

    return ocp


if __name__ == '__main__':
    ocp = generate_acados_ocp()
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp_usv.json')
    print("Success! USV solver C code generated in 'acados_solver'.")
