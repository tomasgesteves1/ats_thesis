import os
import numpy as np
import yaml
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

NX = 13   # [x, y, z, vx, vy, vz, phi, theta, psi, phi_cmd, theta_cmd, psi_cmd, T]
NU = 4    # [phi_dot_cmd, theta_dot_cmd, psi_dot_cmd, T_dot]
NP = 16   # [x_v, y_v, z_v, vx_v, vy_v, vz_v, bx, by, tether_on, d_x, d_y, d_z, tether_model, p_ref (3)]


def load_parameters():
    """Single source of the controller parameters: config/uav_mpc.yaml"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(script_dir, '../config/uav_mpc.yaml'), 'r') as f:
        return yaml.safe_load(f)['uav_mpc_node']['ros__parameters']


def tether_force(p, anchor, tether_on, prm):
    """Closed-form tether force at the UAV (Eqs. uav_tether_geometry -- uav_tether_force)."""
    ez = ca.vertcat(0.0, 0.0, 1.0)
    r = anchor - p
    d = ca.sqrt(ca.dot(r, r) + 1e-6)
    d0 = r / d
    h = d0 - d0[2] * ez
    nh = ca.sqrt(ca.dot(h, h) + prm['tether_delta'] ** 2)
    eh = h / nh
    alpha = ca.atan2(-d0[2], nh)                       # depression angle of the chord
    gamma = np.deg2rad(prm['tether_deflection_deg']) * ca.cos(alpha)
    direction = ca.cos(alpha + gamma) * eh - ca.sin(alpha + gamma) * ez
    c_d = prm['tether_density'] * prm['gravity'] / (1.0 - prm['tether_slack_ratio'])
    return tether_on * c_d * d * direction


def create_uav_model(prm) -> AcadosModel:
    m, g, d_r = prm['mass'], prm['gravity'], prm['rotor_drag']
    tau, tau_psi = prm['tau_roll_pitch'], prm['tau_yaw']

    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    phi, theta, psi = ca.SX.sym('phi'), ca.SX.sym('theta'), ca.SX.sym('psi')
    phi_c, theta_c, psi_c = ca.SX.sym('phi_cmd'), ca.SX.sym('theta_cmd'), ca.SX.sym('psi_cmd')
    T = ca.SX.sym('T')
    x = ca.vertcat(p, v, phi, theta, psi, phi_c, theta_c, psi_c, T)

    dphi_c, dtheta_c, dpsi_c, dT = ca.SX.sym('dphi_cmd'), ca.SX.sym('dtheta_cmd'), ca.SX.sym('dpsi_cmd'), ca.SX.sym('T_dot')
    u = ca.vertcat(dphi_c, dtheta_c, dpsi_c, dT)

    # Parameters per node: tether attachment point (position, velocity), yaw reference point, tether switch, and the
    # estimated external force (disturbance), constant along the horizon, and the switch of the tether force in the model
    # (off only to compare against a controller unaware of the tether)
    anchor = ca.SX.sym('anchor', 3)
    anchor_v = ca.SX.sym('anchor_v', 3)
    b_ref = ca.SX.sym('b_ref', 2)
    tether_on = ca.SX.sym('tether_on')
    dist = ca.SX.sym('dist', 3)
    tether_model = ca.SX.sym('tether_model')
    p_ref = ca.SX.sym('p_ref', 3)
    params = ca.vertcat(anchor, anchor_v, b_ref, tether_on, dist, tether_model, p_ref)

    # Translational dynamics (Eqs. uav_mpc_xddot -- uav_mpc_zddot)
    # The tilt (phi, theta) is the thrust axis expressed in the frame rotated by the commanded yaw psi_c, which is the
    # frame in which the attitude setpoint is built: the thrust axis is rotated by psi_c about the vertical
    tilt = lambda a, b: ca.vertcat(ca.sin(b) * ca.cos(a), -ca.sin(a), ca.cos(b) * ca.cos(a))
    n = tilt(phi, theta)
    thrust_dir = ca.vertcat(ca.cos(psi_c) * n[0] - ca.sin(psi_c) * n[1],
                            ca.sin(psi_c) * n[0] + ca.cos(psi_c) * n[1], n[2])
    drag = ca.vertcat(d_r * v[0], d_r * v[1], 0.0)
    acc = T / m * thrust_dir - drag / m + ca.vertcat(0.0, 0.0, -g) \
          + tether_force(p, anchor, tether_on * tether_model, prm) / m + dist / m

    # The thrust axis of the platform converges to the commanded axis with time constant tau in the inertial frame;
    # expressed in the rotating frame (rate dpsi_c) its derivative has the extra term -dpsi_c * e_z x n
    # The tilt command reaches the platform after the transport delay T_t: x_c(t - T_t) ~ x_c - T_t * dx_c, and the
    # frame in which it was built has rotated by T_t * dpsi_c since then
    T_t = prm['tilt_transport_delay']
    n_cd = tilt(phi_c - T_t * dphi_c, theta_c - T_t * dtheta_c)
    rot = -T_t * dpsi_c
    n_c = ca.vertcat(ca.cos(rot) * n_cd[0] - ca.sin(rot) * n_cd[1], ca.sin(rot) * n_cd[0] + ca.cos(rot) * n_cd[1], n_cd[2])
    n_dot = (n_c - n) / tau - dpsi_c * ca.vertcat(-n[1], n[0], 0.0)
    J = ca.jacobian(n, ca.vertcat(phi, theta))
    phi_dot = ca.dot(J[:, 0], n_dot)
    theta_dot = ca.dot(J[:, 1], n_dot) / ca.cos(phi) ** 2
    # The yaw command reaches the platform after the transport delay: psi_c(t - T_d) ~ psi_c - T_d * dpsi_c
    psi_cmd_delayed = psi_c - prm['yaw_transport_delay'] * dpsi_c

    xdot = ca.vertcat(v, acc, phi_dot, theta_dot, (psi_cmd_delayed - psi) / tau_psi,
                      dphi_c, dtheta_c, dpsi_c, dT)

    model = AcadosModel()
    model.name = 'uav_tethered'
    model.f_expl_expr = xdot
    model.x, model.u, model.p = x, u, params

    # Cost residuals (Eq. uav_mpc_stage_cost). The yaw term q_psi*(1 - e_psi.b/|b|_eps), with e_psi=[cos psi, sin psi],
    # equals (q_psi/2)*||e_psi - b/|b|_eps||^2 up to a constant independent of psi, so it enters as a squared residual.
    b = b_ref - p[0:2]
    b_unit = b / ca.sqrt(ca.dot(b, b) + prm['yaw_epsilon'])
    # The yaw term is active only while the tether (hence the USV) is present: the UAV then looks at the attachment point
    yaw_res = tether_on * ca.vertcat(ca.cos(psi) - b_unit[0], ca.sin(psi) - b_unit[1])
    thrust_res = T - prm['mass'] * prm['gravity']          # deviation from the hover thrust
    # Position error through a pseudo-Huber shaping: the residual e (1 + |e|^2/delta^2)^(-1/4) gives a cost equal to the
    # quadratic one near the reference and growing linearly beyond delta, so that a distant or unreachable reference pulls
    # with a bounded force and never outweighs the penalty of the soft constraints
    e_p = p - p_ref
    pos_res = e_p * (1.0 + ca.dot(e_p, e_p) / prm['huber_delta'] ** 2) ** (-0.25)
    model.cost_y_expr = ca.vertcat(pos_res, v, phi, theta, yaw_res, thrust_res, u)
    model.cost_y_expr_e = ca.vertcat(pos_res, v, phi, theta, yaw_res, thrust_res)

    # Nonlinear constraints: h = [d^2, d_dot, |v_xy|, cos(tilt), cos(tilt_cmd)], where cos(tilt) = cos(phi) cos(theta) is
    # the vertical component of the thrust direction n, so the tilt is limited as a whole, whatever its direction
    r = p - anchor
    v_rel = v - anchor_v
    d_sq = ca.dot(r, r)
    d_dot = ca.dot(r, v_rel) / ca.sqrt(d_sq + 0.01)
    v_xy = ca.sqrt(v[0] ** 2 + v[1] ** 2 + 1e-4)
    cos_tilt = ca.cos(phi) * ca.cos(theta)
    cos_tilt_cmd = ca.cos(phi_c) * ca.cos(theta_c)
    model.con_h_expr = ca.vertcat(d_sq, d_dot, v_xy, cos_tilt, cos_tilt_cmd)
    model.con_h_expr_e = ca.vertcat(d_sq, d_dot, v_xy, cos_tilt, cos_tilt_cmd)

    return model


def generate_acados_ocp():
    prm = load_parameters()
    model = create_uav_model(prm)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = int(prm['horizon_steps'])
    ocp.solver_options.tf = prm['sampling_time'] * ocp.dims.N

    # Cost
    Qp, Qv, Qtilt = prm['weight_Qp'], prm['weight_Qv'], prm['weight_Qtilt']
    q_psi = prm['weight_q_psi']
    W_state = Qp + Qv + Qtilt + [q_psi / 2.0, q_psi / 2.0] + [prm['weight_Q_thrust']]
    W_input = prm['weight_R']
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag(W_state + W_input)
    ocp.cost.W_e = np.diag(W_state)
    ocp.cost.yref = np.zeros((len(W_state) + len(W_input),))
    ocp.cost.yref_e = np.zeros((len(W_state),))

    ocp.parameter_values = np.zeros(NP)

    # Hard input constraints
    ocp.constraints.lbu = np.array([-prm['phi_dot_max'], -prm['theta_dot_max'], -prm['psi_dot_max'], -prm['thrust_rate_max']])
    ocp.constraints.ubu = np.array([prm['phi_dot_max'], prm['theta_dot_max'], prm['psi_dot_max'], prm['thrust_rate_max']])
    ocp.constraints.idxbu = np.arange(4)

    # State bounds: soft z >= z_min and |vz| <= v_z_max, hard T in [T_min, T_max]
    lbx = np.array([prm['z_min'], -prm['v_z_max'], prm['thrust_min']])
    ubx = np.array([1.0e3, prm['v_z_max'], prm['thrust_max']])
    idxbx = np.array([2, 5, 12])
    ocp.constraints.lbx, ocp.constraints.ubx, ocp.constraints.idxbx = lbx, ubx, idxbx
    ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx
    ocp.constraints.idxsbx = np.arange(2)
    ocp.constraints.idxsbx_e = np.arange(2)

    # Soft nonlinear constraints: d^2 <= d_max^2, |d_dot| <= d_dot_max, |v_xy| <= v_xy_max, and the total tilt of the
    # predicted and of the commanded attitude below tilt_max, so that the command sent to the autopilot respects it too
    # (bounds updated at run time)
    c = np.cos(prm['tilt_max'])
    lh = np.array([0.0, prm['d_dot_min'], 0.0, c, c])
    uh = np.array([prm['tether_max_length'] ** 2, prm['winch_speed_limit'], prm['v_xy_max'], 1.0, 1.0])
    ocp.constraints.lh, ocp.constraints.uh = lh, uh
    ocp.constraints.lh_e, ocp.constraints.uh_e = lh, uh
    ocp.constraints.idxsh = np.arange(5)
    ocp.constraints.idxsh_e = np.arange(5)

    # Slack penalties rho(sigma) = mu1*sigma + mu2*sigma^2 (acados: z*s + 0.5*Z*s^2, so Z = 2*mu2).
    # Slack order: bounds (z, vz), then h (d^2, d_dot, |v_xy|, tilt, tilt_cmd)
    mu1 = np.array([prm['slack_mu1_bounds']] * 2 + [prm['slack_mu1_tether']] * 2 + [prm['slack_mu1_bounds']] * 3)
    mu2 = np.array([prm['slack_mu2_bounds']] * 2 + [prm['slack_mu2_tether']] * 2 + [prm['slack_mu2_bounds']] * 3)
    for suffix in ('', '_e'):
        setattr(ocp.cost, 'zl' + suffix, mu1.copy())
        setattr(ocp.cost, 'zu' + suffix, mu1.copy())
        setattr(ocp.cost, 'Zl' + suffix, 2.0 * mu2)
        setattr(ocp.cost, 'Zu' + suffix, 2.0 * mu2)

    ocp.constraints.x0 = np.zeros(NX)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    ocp.code_export_directory = '../acados_solver'
    return ocp


if __name__ == '__main__':
    ocp = generate_acados_ocp()
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("Success! UAV solver C code generated in 'acados_solver'.")
