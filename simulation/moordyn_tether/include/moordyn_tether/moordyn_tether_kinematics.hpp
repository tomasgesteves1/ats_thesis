#ifndef MOORDYN_TETHER__MOORDYN_TETHER_KINEMATICS_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_KINEMATICS_HPP_

#include <array>

namespace moordyn_tether
{

/// @brief Full kinematic state of one tether attachment body.
///
/// All positions are in the world frame.
/// Velocities and orientation are in the body frame (as reported by odometry).
struct BodyState
{
    double anchor_pos[3]{};           // attachment point position  (world frame, from TF)
    double body_center_pos[3]{};      // body CoM / base_link pos   (world frame, from TF)
    double quat[4]{0.0, 0.0, 0.0, 1.0}; // orientation [qx, qy, qz, qw]
    double lin_vel_body[3]{};         // linear  velocity            (body frame, from odometry)
    double ang_vel_body[3]{};         // angular velocity            (body frame, from odometry)
};

/// @brief Pure C++ kinematics utilities for the MoorDyn tether.
class Kinematics
{
public:
    /// Euclidean distance between two 3-D points.
    static double distance3(const double a[3], const double b[3]);

    /// Build a flat position vector [boat_anchor xyz, drone_anchor xyz] for MoorDyn.
    static void buildPositionVector(const std::array<BodyState, 2> & states, double pos[6]);

    /// Build a flat velocity vector [boat_anchor xyz, drone_anchor xyz] for MoorDyn.
    static void buildVelocityVector(const std::array<BodyState, 2> & states, double vel[6]);

private:
    /// Rotate vector v from body to world frame using quaternion q = [qx,qy,qz,qw].
    static void rotateByQuat(const double q[4], const double v[3], double out[3]);

    /// Cross product: out = a × b.
    static void cross3(const double a[3], const double b[3], double out[3]);

    /// Compute the world-frame velocity of the anchor point from a full BodyState.
    static void computeAnchorVelocity(const BodyState & state, double vel_world[3]);
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_KINEMATICS_HPP_
