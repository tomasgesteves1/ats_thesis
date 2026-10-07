#pragma once

#include <array>
#include <cstdint>

namespace mocap_bridge {
namespace kinematics {

struct EnuOdometry {
    std::array<double, 3> position{0.0, 0.0, 0.0};       // [x, y, z] in ENU (Gazebo world)
    std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0}; // [x, y, z, w] in ENU/FLU
    std::array<double, 3> linear_vel{0.0, 0.0, 0.0};     // [vx, vy, vz] in FLU (body)
    std::array<double, 3> angular_vel{0.0, 0.0, 0.0};    // [wx, wy, wz] in FLU (body)
};

struct NedOdometry {
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};     // [x, y, z] in NED
    std::array<float, 4> orientation{1.0f, 0.0f, 0.0f, 0.0f};  // [w, x, y, z] in NED/FRD (Hamiltonian)
    std::array<float, 3> linear_vel{0.0f, 0.0f, 0.0f};   // [vx, vy, vz]
    std::array<float, 3> angular_vel{0.0f, 0.0f, 0.0f};  // [wx, wy, wz] in FRD (body)
    uint8_t velocity_frame{1};                         // 1: NED, 3: BODY_FRD
};

struct EulerAngles {
    double roll{0.0};   // radians
    double pitch{0.0};  // radians
    double yaw{0.0};    // radians
};

struct OdometryError {
    std::array<double, 3> position_error{0.0, 0.0, 0.0};
    EulerAngles attitude_error{0.0, 0.0, 0.0};
    std::array<double, 3> linear_vel_error{0.0, 0.0, 0.0};
    std::array<double, 3> angular_vel_error{0.0, 0.0, 0.0};
};

void convertEnuToNed(const EnuOdometry& in, NedOdometry& out);
void convertNedToEnu(const NedOdometry& in, EnuOdometry& out);
EulerAngles quaternionToEuler(const std::array<double, 4>& q_xyzw);
OdometryError computeError(const EnuOdometry& gt, const EnuOdometry& est);

} // namespace kinematics
} // namespace mocap_bridge

