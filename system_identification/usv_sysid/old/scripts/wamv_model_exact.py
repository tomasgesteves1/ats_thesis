import casadi as ca
import numpy as np

# ==============================================================================
# Exact WAM-V Parameters Computed from model.sdf (Rigid Body Assembly)
# ==============================================================================
# Composite Mass: Base (227) + GPS (1.0) + IMU (0.1) + Left Engine (15.0) + Right Engine (15.0) + Propellers (2 * 0.5)
MASS = 259.1          

# Composite Center of Gravity relative to base_link:
# X_cg = -0.28979 m, Y_cg = 0.0 m, Z_cg = 0.15048 m
#
# Composite Yaw Inertia about the Center of Gravity (using Parallel Axis Theorem):
# Base IZZ (495.037) + Base offset term (227 * 0.28979^2) + Engine IZZs (2 * 0.078)
# + Engine offset terms (2 * 15.0 * ((-2.373776 - X_cg)^2 + 1.027135^2))
# + Propeller offset terms (2 * 0.5 * ((-2.651932 - X_cg)^2 + 1.027135^2)) + GPS offset term
IZZ = 683.15

# Hydrodynamic damping coefficients from vrx::SimpleHydrodynamics plugin
X_U = 100.0            # Linear surge damping
X_UU = 150.0           # Quadratic surge damping (forward)
X_UU_BWD = 150.0       # Quadratic surge damping (backward)
Y_V = 100.0            # Linear sway damping
Y_VV = 100.0           # Quadratic sway damping
N_R = 800.0            # Linear yaw damping
N_RR = 800.0           # Quadratic yaw damping

# Thruster coordinates relative to the composite Center of Gravity (CG):
# x_t_base = -2.373776, y_t_base = 1.027135
# x_t_cg = x_t_base - X_cg = -2.373776 - (-0.28979) = -2.083986 m
THRUSTER_X_ARM = -2.083986  # Longitudinal position of thrusters (m)
THRUSTER_Y_ARM = 1.027135   # Lateral distance from centerline to thrusters (m)

def wamv_dynamics_numerical(x, u_virtual):
    """
    Numerical evaluation of WAM-V dynamics (Fossen 3-DOF model) for simulation.
    """
    psi = x[2]
    u = x[3]
    v = x[4]
    r = x[5]

    X_force = u_virtual[0]
    Y_force = u_virtual[1]
    N_moment = u_virtual[2]

    # Kinematics
    x_dot = u * np.cos(psi) - v * np.sin(psi)
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    psi_dot = r

    # Dynamics (Fossen 3-DOF model equations using composite mass/inertia)
    x_uu_eff = X_UU_BWD if u < 0.0 else X_UU
    
    u_dot = (X_force - (X_U + x_uu_eff * np.abs(u)) * u) / MASS + v * r
    v_dot = (Y_force - (Y_V + Y_VV * np.abs(v)) * v) / MASS - u * r
    r_dot = (N_moment - (N_R + N_RR * np.abs(r)) * r) / IZZ

    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])

def get_wamv_casadi_model():
    """
    Generates the CasADi symbolic variables and dynamics expressions.
    """
    x_pos = ca.SX.sym('x_pos')
    y_pos = ca.SX.sym('y_pos')
    psi = ca.SX.sym('psi')
    u = ca.SX.sym('u')
    v = ca.SX.sym('v')
    r = ca.SX.sym('r')
    states = ca.vertcat(x_pos, y_pos, psi, u, v, r)

    x_pos_dot = ca.SX.sym('x_pos_dot')
    y_pos_dot = ca.SX.sym('y_pos_dot')
    psi_dot = ca.SX.sym('psi_dot')
    u_dot = ca.SX.sym('u_dot')
    v_dot = ca.SX.sym('v_dot')
    r_dot = ca.SX.sym('r_dot')
    states_dot = ca.vertcat(x_pos_dot, y_pos_dot, psi_dot, u_dot, v_dot, r_dot)

    X_force = ca.SX.sym('X_force')
    Y_force = ca.SX.sym('Y_force')
    N_moment = ca.SX.sym('N_moment')
    controls = ca.vertcat(X_force, Y_force, N_moment)

    x_uu_eff = ca.if_else(u < 0.0, X_UU_BWD, X_UU)

    f_expl = ca.vertcat(
        u * ca.cos(psi) - v * ca.sin(psi),
        u * ca.sin(psi) + v * ca.cos(psi),
        r,
        (X_force - (X_U + x_uu_eff * ca.fabs(u)) * u) / MASS + v * r,
        (Y_force - (Y_V + Y_VV * ca.fabs(v)) * v) / MASS - u * r,
        (N_moment - (N_R + N_RR * ca.fabs(r)) * r) / IZZ
    )

    f_impl = states_dot - f_expl

    return states, states_dot, controls, f_expl, f_impl

def thrust_allocation_map(thrust_left, thrust_right, angle_left, angle_right):
    """
    Maps physical thruster forces and angles to virtual forces on the CG.
    """
    tl_eff = thrust_left
    tr_eff = thrust_right
    
    f_x_L = tl_eff * np.cos(angle_left)
    f_y_L = tl_eff * np.sin(angle_left)
    
    f_x_R = tr_eff * np.cos(angle_right)
    f_y_R = tr_eff * np.sin(angle_right)
    
    X_force = f_x_L + f_x_R
    Y_force = f_y_L + f_y_R
    
    N_moment = THRUSTER_X_ARM * (f_y_L + f_y_R) - THRUSTER_Y_ARM * (f_x_L - f_x_R)
    
    return np.array([X_force, Y_force, N_moment])
