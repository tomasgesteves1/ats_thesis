#pragma once

namespace uav_mpc {
namespace kinematics {

/**
 * Convert a quaternion (w, x, y, z) into Euler angles (roll, pitch, yaw) in ENU frame.
 */
void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw);

/**
 * Convert a 3x3 rotation matrix to a quaternion (Hamiltonian [w, x, y, z] order).
 */
void rotationMatrixToQuaternion(const double R[3][3], double q[4]);

/**
 * Rotate a 3D vector by a quaternion (v_out = q * v_in * q*).
 */
void rotateVectorByQuaternion(double qx, double qy, double qz, double qw, const double v_in[3], double v_out[3]);

/**
 * Compute the desired attitude quaternion (q_d) in NED/FRD frame given body tilt angles in ENU and yaw.
 */
void computeDesiredQuaternion(double phi_cmd, double theta_cmd, double yaw, double q_d[4]);

} // namespace kinematics
} // namespace uav_mpc
