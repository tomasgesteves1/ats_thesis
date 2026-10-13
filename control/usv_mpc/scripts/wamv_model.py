import casadi as ca

# ==============================================================================
# Nominal parameters of the WAM-V model (Section 4.2.1, Table 4.1)
# ==============================================================================
MASS = 259.10          # Rigid body mass (kg)
IZZ = 683.22           # Yaw moment of inertia about the CG = base_link origin (kg*m^2)

# Hydrodynamic damping (VRX SimpleHydrodynamics)
X_U = 100.00           # Linear surge damping (N/(m/s))
X_UU = 150.00          # Quadratic surge damping (N/(m/s)^2)
Y_V = 100.00           # Linear sway damping (N/(m/s))
Y_VV = 100.00          # Quadratic sway damping (N/(m/s)^2)
N_R = 800.00           # Linear yaw damping (N*m/(rad/s))
N_RR = 800.00          # Quadratic yaw damping (N*m/(rad/s)^2)

# Thruster geometry relative to the CG (m), as in Eq. usv_mpc_alloc_moment
X_ARM = 2.085527       # Longitudinal distance (thrusters are aft of the CG)
Y_ARM = 1.027135       # Lateral distance from the centreline

NX = 10                # [x, y, psi, u, v, r, T_L, T_R, alpha_L, alpha_R]
NU = 4                 # [dT_L, dT_R, dalpha_L, dalpha_R]
NP = 5                 # [psi_ref, x_a, y_a, z_a, z_t]


def get_wamv_casadi_model():
    """
    CasADi model of the USV used by the NMPC (Section 3.3): the thruster thrusts
    and azimuths are states, and their rates of change are the control inputs.
    Returns x, xdot, u, p, f_expl, f_impl.
    """
    x_pos = ca.SX.sym('x_pos')
    y_pos = ca.SX.sym('y_pos')
    psi = ca.SX.sym('psi')
    u = ca.SX.sym('u')
    v = ca.SX.sym('v')
    r = ca.SX.sym('r')
    T_L = ca.SX.sym('T_L')
    T_R = ca.SX.sym('T_R')
    a_L = ca.SX.sym('alpha_L')
    a_R = ca.SX.sym('alpha_R')
    states = ca.vertcat(x_pos, y_pos, psi, u, v, r, T_L, T_R, a_L, a_R)
    states_dot = ca.SX.sym('xdot', NX)

    dT_L = ca.SX.sym('dT_L')
    dT_R = ca.SX.sym('dT_R')
    da_L = ca.SX.sym('dalpha_L')
    da_R = ca.SX.sym('dalpha_R')
    controls = ca.vertcat(dT_L, dT_R, da_L, da_R)

    # Parameters per node: heading reference and UAV predicted position / attachment height
    psi_ref = ca.SX.sym('psi_ref')
    x_a = ca.SX.sym('x_a')
    y_a = ca.SX.sym('y_a')
    z_a = ca.SX.sym('z_a')
    z_t = ca.SX.sym('z_t')
    params = ca.vertcat(psi_ref, x_a, y_a, z_a, z_t)

    # Thrust allocation (Eqs. usv_mpc_alloc_force / usv_mpc_alloc_moment)
    X_force = T_L * ca.cos(a_L) + T_R * ca.cos(a_R)
    Y_force = T_L * ca.sin(a_L) + T_R * ca.sin(a_R)
    N_moment = -X_ARM * (T_L * ca.sin(a_L) + T_R * ca.sin(a_R)) \
               - Y_ARM * (T_L * ca.cos(a_L) - T_R * ca.cos(a_R))

    f_expl = ca.vertcat(
        u * ca.cos(psi) - v * ca.sin(psi),
        u * ca.sin(psi) + v * ca.cos(psi),
        r,
        (X_force - (X_U + X_UU * ca.fabs(u)) * u) / MASS + v * r,
        (Y_force - (Y_V + Y_VV * ca.fabs(v)) * v) / MASS - u * r,
        (N_moment - (N_R + N_RR * ca.fabs(r)) * r) / IZZ,
        dT_L, dT_R, da_L, da_R
    )
    f_impl = states_dot - f_expl

    return states, states_dot, controls, params, f_expl, f_impl
