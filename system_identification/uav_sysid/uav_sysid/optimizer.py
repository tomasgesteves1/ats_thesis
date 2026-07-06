import numpy as np
from scipy.optimize import minimize

def simulate_1st_order(tau, u_cmd, dt, y0):
    """
    Simulates a first-order response: tau * y_dot + y = u_cmd.
    """
    y_sim = np.zeros_like(u_cmd)
    y_sim[0] = y0
    for i in range(1, len(u_cmd)):
        y_sim[i] = y_sim[i-1] + (dt / tau) * (u_cmd[i-1] - y_sim[i-1])
    return y_sim

def loss_1st_order(tau, u_cmd, y_meas, dt):
    if tau <= 0.01:
        return 1e6
    y_sim = simulate_1st_order(tau, u_cmd, dt, y_meas[0])
    return np.mean((y_meas - y_sim) ** 2)

def identify_attitude_axis(t, cmd, meas, name="Roll"):
    """
    Identifies the time constant tau for a single attitude axis.
    """
    dt = np.mean(np.diff(t))
    
    # Initial guess for tau: 0.15 seconds
    res = minimize(loss_1st_order, [0.15], args=(cmd, meas, dt), method='Nelder-Mead')
    tau_est = res.x[0]
    
    # Calculate fit metric (R2)
    y_sim = simulate_1st_order(tau_est, cmd, dt, meas[0])
    y_mean = np.mean(meas)
    ss_res = np.sum((meas - y_sim) ** 2)
    ss_tot = np.sum((meas - y_mean) ** 2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
    
    return tau_est, r2, y_sim

def identify_vertical_dynamics(t, thrust_cmd, z, vz, g=9.81):
    """
    Identifies vertical thrust gain (c1) and drag (c2):
    az_meas = c1 * thrust_cmd - g - c2 * vz
    Uses numerical differentiation for acceleration: az = d(vz)/dt
    """
    dt = np.mean(np.diff(t))
    
    # Compute acceleration using central differences
    az = np.gradient(vz, dt)
    
    # Formulate linear regression: az + g = c1 * thrust_cmd - c2 * vz
    # Y = A * X
    # Y = az + g
    # A = [thrust_cmd, -vz]
    # X = [c1, c2]^T
    Y = az + g
    A = np.column_stack((thrust_cmd, -vz))
    
    # Solve least squares: A * X = Y
    sol, residuals, rank, s = np.linalg.lstsq(A, Y, rcond=None)
    c1_thrust_gain = sol[0]
    c2_vertical_drag = sol[1]
    
    # Compute simulated acceleration
    az_sim = c1_thrust_gain * thrust_cmd - g - c2_vertical_drag * vz
    
    # Compute R2 metric for acceleration
    ss_res = np.sum((az - az_sim) ** 2)
    ss_tot = np.sum((az - np.mean(az)) ** 2)
    r2_accel = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
    
    return c1_thrust_gain, c2_vertical_drag, r2_accel, az, az_sim
