#pragma once

#include <array>

namespace mocap_bridge {
namespace kinematics {

struct EnuOdometry {
    std::array<double, 3> position;       // [x, y, z] in ENU (Gazebo world)
    std::array<double, 4> orientation;    // [x, y, z, w] in ENU/FLU
    std::array<double, 3> linear_vel;     // [vx, vy, vz] in FLU (body)
    std::array<double, 3> angular_vel;    // [wx, wy, wz] in FLU (body)
};

struct NedOdometry {
    std::array<float, 3> position;        // [x, y, z] in NED
    std::array<float, 4> orientation;     // [w, x, y, z] in NED/FRD (Hamiltonian)
    std::array<float, 3> linear_vel;      // [vx, vy, vz] in FRD (body)
    std::array<float, 3> angular_vel;     // [wx, wy, wz] in FRD (body)
};

void convertEnuToNed(const EnuOdometry& in, NedOdometry& out);

} // namespace kinematics
} // namespace mocap_bridge
