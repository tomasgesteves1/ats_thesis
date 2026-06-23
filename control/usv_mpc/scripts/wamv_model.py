import casadi as ca
import numpy as np

# ==============================================================================
# Ground Truth Physical Parameters from model.sdf (VRX WAM-V)
# ==============================================================================
MASS = 289.83          # Optimized rigid body mass (kg) (Nominal: 227)
IZZ = 600.00           # Optimized moment of inertia about Z axis (kg*m^2) (Nominal: 495)

# Hydrodynamic damping coefficients
X_U = 165.97           # Linear surge damping (N/(m/s))
X_UU = 140.00          # Quadratic surge damping (forward) (N/(m/s)^2)
X_UU_BWD = 157.68      # Quadratic surge damping (backward) (N/(m/s)^2)
Y_V = 92.85            # Linear sway damping (N/(m/s))
Y_VV = 102.42          # Quadratic sway damping (N/(m/s)^2)
N_R = 400.76           # Linear yaw damping (N*m/(rad/s))
N_RR = 1069.15         # Quadratic yaw damping (N*m/(rad/s)^2)

# Thruster geometric configuration (coordinates relative to CG)
THRUSTER_X_ARM = -2.373776  # Longitudinal position of thrusters (m)
THRUSTER_Y_ARM = 1.027135   # Lateral distance from centerline to thrusters (m)

def wamv_dynamics_numerical(x, u_virtual):
    """
    Numerical evaluation of WAM-V dynamics (Fossen 3-DOF model) for simulation.
    x: State vector numpy array [x, y, psi, u, v, r]
    u_virtual: Control input vector numpy array [X, Y, N] (Surge force, Sway force, Yaw moment)
    Returns: State derivative dx/dt [x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot]
    """
    psi = x[2]
    u = x[3]
    v = x[4]
    r = x[5]

    X_force = u_virtual[0]
    Y_force = u_virtual[1]
    N_moment = u_virtual[2]

    # Kinematics (Body-to-Global translation)
    x_dot = u * np.cos(psi) - v * np.sin(psi)
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    psi_dot = r

    # Dynamics (Fossen 3-DOF model equations)
    # Added mass is zero in SDF: X_dot_U = 0, Y_dot_V = 0, N_dot_R = 0
    # Direction-dependent drag in surge (bow vs stern asymmetry)
    x_uu_eff = X_UU_BWD if u < 0.0 else X_UU
    
    u_dot = (X_force - (X_U + x_uu_eff * np.abs(u)) * u) / MASS + v * r
    v_dot = (Y_force - (Y_V + Y_VV * np.abs(v)) * v) / MASS - u * r
    r_dot = (N_moment - (N_R + N_RR * np.abs(r)) * r) / IZZ

    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])

def get_wamv_casadi_model():
    """
    Generates the CasADi symbolic variables and dynamics expressions.
    This function is designed to be imported by the Acados generator script.
    """
    # Define CasADi states
    x_pos = ca.SX.sym('x_pos')
    y_pos = ca.SX.sym('y_pos')
    psi = ca.SX.sym('psi')
    u = ca.SX.sym('u')
    v = ca.SX.sym('v')
    r = ca.SX.sym('r')
    states = ca.vertcat(x_pos, y_pos, psi, u, v, r)

    # Define CasADi derivatives
    x_pos_dot = ca.SX.sym('x_pos_dot')
    y_pos_dot = ca.SX.sym('y_pos_dot')
    psi_dot = ca.SX.sym('psi_dot')
    u_dot = ca.SX.sym('u_dot')
    v_dot = ca.SX.sym('v_dot')
    r_dot = ca.SX.sym('r_dot')
    states_dot = ca.vertcat(x_pos_dot, y_pos_dot, psi_dot, u_dot, v_dot, r_dot)

    # Define CasADi control inputs (Virtual force/moment level: [X, Y, N])
    X_force = ca.SX.sym('X_force')
    Y_force = ca.SX.sym('Y_force')
    N_moment = ca.SX.sym('N_moment')
    controls = ca.vertcat(X_force, Y_force, N_moment)

    # Direction-dependent drag in surge for CasADi (bow vs stern asymmetry)
    x_uu_eff = ca.if_else(u < 0.0, X_UU_BWD, X_UU)

    # Explicit dynamics equations (f_expl)
    f_expl = ca.vertcat(
        u * ca.cos(psi) - v * ca.sin(psi),
        u * ca.sin(psi) + v * ca.cos(psi),
        r,
        (X_force - (X_U + x_uu_eff * ca.fabs(u)) * u) / MASS + v * r,
        (Y_force - (Y_V + Y_VV * ca.fabs(v)) * v) / MASS - u * r,
        (N_moment - (N_R + N_RR * ca.fabs(r)) * r) / IZZ
    )

    # Implicit dynamics equations (f_impl = states_dot - f_expl)
    f_impl = states_dot - f_expl

    return states, states_dot, controls, f_expl, f_impl

def thrust_allocation_map(thrust_left, thrust_right, angle_left, angle_right):
    """
    Maps physical thruster forces and angles to virtual forces on the CG (Thrust Allocation).
    T_L, T_R, alpha_L, alpha_R -> [X, Y, N]^T
    Accounts for reverse thrust propeller inefficiency (0.746 multiplier).
    """
    # Apply reverse thrust efficiency factor if thrust command is negative
    tl_eff = thrust_left * 0.746 if thrust_left < 0.0 else thrust_left
    tr_eff = thrust_right * 0.746 if thrust_right < 0.0 else thrust_right
    
    # Force components per motor in body frame
    f_x_L = tl_eff * np.cos(angle_left)
    f_y_L = tl_eff * np.sin(angle_left)
    
    f_x_R = tr_eff * np.cos(angle_right)
    f_y_R = tr_eff * np.sin(angle_right)
    
    # Sum of forces in body frame
    X_force = f_x_L + f_x_R
    Y_force = f_y_L + f_y_R
    
    # Resulting moment: N = x * F_y - y * F_x
    N_moment = THRUSTER_X_ARM * (f_y_L + f_y_R) - THRUSTER_Y_ARM * (f_x_L - f_x_R)
    
    return np.array([X_force, Y_force, N_moment])
