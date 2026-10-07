#include "mocap_bridge/mocap_bridge_kinematics.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace mocap_bridge {
namespace kinematics {

void convertEnuToNed(const EnuOdometry& in, NedOdometry& out) {
    // 1. Position: ENU -> NED
    // X_ned (North) = Y_enu
    // Y_ned (East)  = X_enu
    // Z_ned (Down)  = -Z_enu
    out.position[0] = static_cast<float>(in.position[1]);
    out.position[1] = static_cast<float>(in.position[0]);
    out.position[2] = static_cast<float>(-in.position[2]);

    // 2. Orientation: Body FLU -> ENU to Body FRD -> NED
    // Rotation static compositions matching PX4 official gz_bridge:
    // q_FLU_to_FRD: 180 deg rotation around X (w=0, x=1, y=0, z=0)
    // q_ENU_to_NED: 180 deg around X followed by 90 deg around Z (w=0, x=sqrt(0.5), y=sqrt(0.5), z=0)
    // q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.inverse()
    const Eigen::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
    const double s = std::sqrt(0.5);
    const Eigen::Quaterniond q_enu_to_ned(0.0, s, s, 0.0);

    // Input quaternion in ROS order [x, y, z, w]
    const Eigen::Quaterniond q_flu_to_enu(in.orientation[3], in.orientation[0], in.orientation[1], in.orientation[2]);

    Eigen::Quaterniond q_frd_to_ned = q_enu_to_ned * q_flu_to_enu * q_flu_to_frd.inverse();
    q_frd_to_ned.normalize();

    // PX4 expects Hamiltonian order [w, x, y, z]
    out.orientation[0] = static_cast<float>(q_frd_to_ned.w());
    out.orientation[1] = static_cast<float>(q_frd_to_ned.x());
    out.orientation[2] = static_cast<float>(q_frd_to_ned.y());
    out.orientation[3] = static_cast<float>(q_frd_to_ned.z());

    // 3. Linear Velocity: Body FLU -> Body FRD
    // vx_frd = vx_flu
    // vy_frd = -vy_flu
    // vz_frd = -vz_flu
    out.linear_vel[0] = static_cast<float>(in.linear_vel[0]);
    out.linear_vel[1] = static_cast<float>(-in.linear_vel[1]);
    out.linear_vel[2] = static_cast<float>(-in.linear_vel[2]);

    // 4. Angular Velocity: Body FLU -> Body FRD
    out.angular_vel[0] = static_cast<float>(in.angular_vel[0]);
    out.angular_vel[1] = static_cast<float>(-in.angular_vel[1]);
    out.angular_vel[2] = static_cast<float>(-in.angular_vel[2]);
}

void convertNedToEnu(const NedOdometry& in, EnuOdometry& out) {
    // 1. Position: NED -> ENU
    // X_enu (East)  = Y_ned
    // Y_enu (North) = X_ned
    // Z_enu (Up)    = -Z_ned
    out.position[0] = static_cast<double>(in.position[1]);
    out.position[1] = static_cast<double>(in.position[0]);
    out.position[2] = static_cast<double>(-in.position[2]);

    // 2. Orientation: Body FRD -> NED to Body FLU -> ENU
    // q_FLU_to_ENU = q_ENU_to_NED.inverse() * q_FRD_to_NED * q_FLU_to_FRD
    const Eigen::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
    const double s = std::sqrt(0.5);
    const Eigen::Quaterniond q_enu_to_ned(0.0, s, s, 0.0);

    // Input Hamiltonian quaternion [w, x, y, z]
    const Eigen::Quaterniond q_frd_to_ned(in.orientation[0], in.orientation[1], in.orientation[2], in.orientation[3]);

    Eigen::Quaterniond q_flu_to_enu = q_enu_to_ned.inverse() * q_frd_to_ned * q_flu_to_frd;
    q_flu_to_enu.normalize();

    // Output ROS order [x, y, z, w]
    out.orientation[0] = q_flu_to_enu.x();
    out.orientation[1] = q_flu_to_enu.y();
    out.orientation[2] = q_flu_to_enu.z();
    out.orientation[3] = q_flu_to_enu.w();

    // 3. Linear Velocity
    if (in.velocity_frame == 1) { // VELOCITY_FRAME_NED
        const Eigen::Vector3d v_world_enu(in.linear_vel[1], in.linear_vel[0], -in.linear_vel[2]);
        const Eigen::Vector3d v_body_flu = q_flu_to_enu.inverse() * v_world_enu;
        out.linear_vel[0] = v_body_flu.x();
        out.linear_vel[1] = v_body_flu.y();
        out.linear_vel[2] = v_body_flu.z();
    } else { // VELOCITY_FRAME_BODY_FRD
        out.linear_vel[0] = static_cast<double>(in.linear_vel[0]);
        out.linear_vel[1] = static_cast<double>(-in.linear_vel[1]);
        out.linear_vel[2] = static_cast<double>(-in.linear_vel[2]);
    }

    // 4. Angular Velocity: Body FRD -> Body FLU
    out.angular_vel[0] = static_cast<double>(in.angular_vel[0]);
    out.angular_vel[1] = static_cast<double>(-in.angular_vel[1]);
    out.angular_vel[2] = static_cast<double>(-in.angular_vel[2]);
}

EulerAngles quaternionToEuler(const std::array<double, 4>& q_xyzw) {
    const double qx = q_xyzw[0];
    const double qy = q_xyzw[1];
    const double qz = q_xyzw[2];
    const double qw = q_xyzw[3];

    EulerAngles angles;

    // Roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
        angles.pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        angles.pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

OdometryError computeError(const EnuOdometry& gt, const EnuOdometry& est) {
    OdometryError err;
    for (size_t i = 0; i < 3; ++i) {
        err.position_error[i] = est.position[i] - gt.position[i];
        err.linear_vel_error[i] = est.linear_vel[i] - gt.linear_vel[i];
        err.angular_vel_error[i] = est.angular_vel[i] - gt.angular_vel[i];
    }

    const EulerAngles gt_euler = quaternionToEuler(gt.orientation);
    const EulerAngles est_euler = quaternionToEuler(est.orientation);

    auto wrap_angle = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    err.attitude_error.roll = wrap_angle(est_euler.roll - gt_euler.roll);
    err.attitude_error.pitch = wrap_angle(est_euler.pitch - gt_euler.pitch);
    err.attitude_error.yaw = wrap_angle(est_euler.yaw - gt_euler.yaw);

    return err;
}

} // namespace kinematics
} // namespace mocap_bridge
