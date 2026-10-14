from scipy.spatial.transform import Rotation as R

# Constant rotation matrix for ENU (world) <-> NED
# x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
R_enu_to_ned = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

# Constant rotation matrix for FLU (body-ROS) <-> FRD (body-PX4)
# x_frd = x_flu, y_frd = -y_flu, z_frd = -z_flu
R_flu_to_frd = R.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

def enu_to_ned_quaternion(q_enu):
    """
    Convert ENU/FLU quaternion [x, y, z, w] to NED/FRD quaternion [x, y, z, w].
    """
    r_enu = R.from_quat(q_enu)
    r_ned = R_enu_to_ned * r_enu * R_flu_to_frd
    return r_ned.as_quat()

def ned_to_enu_quaternion(q_ned):
    """
    Convert NED/FRD quaternion [x, y, z, w] to ENU/FLU quaternion [x, y, z, w].
    """
    r_ned = R.from_quat(q_ned)
    r_enu = R_enu_to_ned * r_ned * R_flu_to_frd
    return r_enu.as_quat()

def quaternion_to_euler_enu(q_quat, hamiltonian=False):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw) in radians in the ENU frame.
    If hamiltonian is True, the input is [w, x, y, z] (standard for PX4).
    Otherwise, it is [x, y, z, w] (standard for ROS 2 / SciPy).
    """
    if hamiltonian:
        q_scipy = [q_quat[1], q_quat[2], q_quat[3], q_quat[0]]
    else:
        q_scipy = q_quat
        
    r = R.from_quat(q_scipy)
    euler = r.as_euler('xyz') # roll, pitch, yaw
    return euler[0], euler[1], euler[2]

def quaternion_ned_to_euler_enu(q_d_ned):
    """
    Convert desired quaternion from NED/FRD [w, x, y, z] to Euler angles (roll, pitch, yaw) in ENU frame.
    """
    q_scipy_ned = [q_d_ned[1], q_d_ned[2], q_d_ned[3], q_d_ned[0]]
    q_scipy_enu = ned_to_enu_quaternion(q_scipy_ned)
    return quaternion_to_euler_enu(q_scipy_enu, hamiltonian=False)
