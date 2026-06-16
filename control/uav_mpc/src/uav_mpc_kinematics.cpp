#include "uav_mpc/uav_mpc_kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace uav_mpc {
namespace kinematics {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) {
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void rotationMatrixToQuaternion(const double R[3][3], double q[4]) {
    double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        double s = 2.0 * std::sqrt(tr + 1.0);
        q[0] = 0.25 * s;                // w
        q[1] = (R[2][1] - R[1][2]) / s; // x
        q[2] = (R[0][2] - R[2][0]) / s; // y
        q[3] = (R[1][0] - R[0][1]) / s; // z
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q[0] = (R[2][1] - R[1][2]) / s; // w
        q[1] = 0.25 * s;                // x
        q[2] = (R[0][1] + R[1][0]) / s; // y
        q[3] = (R[0][2] + R[2][0]) / s; // z
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q[0] = (R[0][2] - R[2][0]) / s; // w
        q[1] = (R[0][1] + R[1][0]) / s; // x
        q[2] = 0.25 * s;                // y
        q[3] = (R[1][2] + R[2][1]) / s; // z
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q[0] = (R[1][0] - R[0][1]) / s; // w
        q[1] = (R[0][2] + R[2][0]) / s; // x
        q[2] = (R[1][2] + R[2][1]) / s; // y
        q[3] = 0.25 * s;                // z
    }
}

void rotateVectorByQuaternion(double qx, double qy, double qz, double qw, const double v_in[3], double v_out[3]) {
    // v_out = v_in + 2 * q_xyz x (q_xyz x v_in + w * v_in)
    double tx = 2.0 * (qy * v_in[2] - qz * v_in[1]);
    double ty = 2.0 * (qz * v_in[0] - qx * v_in[2]);
    double tz = 2.0 * (qx * v_in[1] - qy * v_in[0]);

    v_out[0] = v_in[0] + qw * tx + qy * tz - qz * ty;
    v_out[1] = v_in[1] + qw * ty + qz * tx - qx * tz;
    v_out[2] = v_in[2] + qw * tz + qx * ty - qy * tx;
}

void computeDesiredQuaternion(double phi_cmd, double theta_cmd, double yaw, double q_d[4]) {
    double c_psi = std::cos(yaw);
    double s_psi = std::sin(yaw);
    double c_theta = std::cos(theta_cmd);
    double s_theta = std::sin(theta_cmd);
    double c_phi = std::cos(phi_cmd);
    double s_phi = std::sin(phi_cmd);

    // Desired rotation matrix in ENU frame: R_enu = R_z(yaw) * R_y(theta_cmd) * R_x(phi_cmd)
    double R_enu[3][3];
    R_enu[0][0] = c_psi * c_theta;
    R_enu[0][1] = c_psi * s_theta * s_phi - s_psi * c_phi;
    R_enu[0][2] = c_psi * s_theta * c_phi + s_psi * s_phi;
    R_enu[1][0] = s_psi * c_theta;
    R_enu[1][1] = s_psi * s_theta * s_phi + c_psi * c_phi;
    R_enu[1][2] = s_psi * s_theta * c_phi - c_psi * s_phi;
    R_enu[2][0] = -s_theta;
    R_enu[2][1] = c_theta * s_phi;
    R_enu[2][2] = c_theta * c_phi;

    // Convert R_enu to R_d (NED/FRD) frame using rotation M: R_d = M * R_enu * M
    double R_d[3][3];
    R_d[0][0] = R_enu[1][1];
    R_d[0][1] = R_enu[1][0];
    R_d[0][2] = -R_enu[1][2];

    R_d[1][0] = R_enu[0][1];
    R_d[1][1] = R_enu[0][0];
    R_d[1][2] = -R_enu[0][2];

    R_d[2][0] = -R_enu[2][1];
    R_d[2][1] = -R_enu[2][0];
    R_d[2][2] = R_enu[2][2];

    // Convert R_d to desired quaternion q_d (Hamiltonian [w, x, y, z] order for PX4)
    rotationMatrixToQuaternion(R_d, q_d);
}

} // namespace kinematics
} // namespace uav_mpc
