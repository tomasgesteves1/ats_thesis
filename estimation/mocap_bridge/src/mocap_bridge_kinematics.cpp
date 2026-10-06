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

} // namespace kinematics
} // namespace mocap_bridge
