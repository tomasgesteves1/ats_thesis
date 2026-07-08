#include "uav_mpc/uav_mpc_kinematics.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace uav_mpc {
namespace kinematics {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Matrix3d R = q.toRotationMatrix();
    
    roll = std::atan2(R(2, 1), R(2, 2));
    double sinp = -R(2, 0);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }
    yaw = std::atan2(R(1, 0), R(0, 0));
}

void rotationMatrixToQuaternion(const double R[3][3], double q[4]) {
    Eigen::Matrix3d R_eigen;
    R_eigen << R[0][0], R[0][1], R[0][2],
               R[1][0], R[1][1], R[1][2],
               R[2][0], R[2][1], R[2][2];
    Eigen::Quaterniond q_eigen(R_eigen);
    q[0] = q_eigen.w();
    q[1] = q_eigen.x();
    q[2] = q_eigen.y();
    q[3] = q_eigen.z();
}

void rotateVectorByQuaternion(double qx, double qy, double qz, double qw, const double v_in[3], double v_out[3]) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d v(v_in[0], v_in[1], v_in[2]);
    Eigen::Vector3d v_rot = q * v;
    
    v_out[0] = v_rot.x();
    v_out[1] = v_rot.y();
    v_out[2] = v_rot.z();
}

void computeDesiredQuaternion(double phi_cmd, double theta_cmd, double yaw, double q_d[4]) {
    // Desired rotation matrix in ENU frame: R_enu = R_z(yaw) * R_y(theta_cmd) * R_x(phi_cmd)
    Eigen::Matrix3d R_enu = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(theta_cmd, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(phi_cmd, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Convert R_enu to R_d (NED/FRD) frame using rotation: R_d = M_world * R_enu * M_body
    Eigen::Matrix3d R_d;
    R_d(0, 0) = R_enu(1, 0);
    R_d(0, 1) = -R_enu(1, 1);
    R_d(0, 2) = -R_enu(1, 2);

    R_d(1, 0) = R_enu(0, 0);
    R_d(1, 1) = -R_enu(0, 1);
    R_d(1, 2) = -R_enu(0, 2);

    R_d(2, 0) = -R_enu(2, 0);
    R_d(2, 1) = R_enu(2, 1);
    R_d(2, 2) = R_enu(2, 2);

    // Convert R_d to desired quaternion q_d (Hamiltonian [w, x, y, z] order for PX4)
    Eigen::Quaterniond q_eigen(R_d);
    q_d[0] = q_eigen.w();
    q_d[1] = q_eigen.x();
    q_d[2] = q_eigen.y();
    q_d[3] = q_eigen.z();
}

} // namespace kinematics
} // namespace uav_mpc
