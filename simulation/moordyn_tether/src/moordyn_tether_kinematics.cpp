#include "moordyn_tether/moordyn_tether_kinematics.hpp"

#include <cmath>

namespace moordyn_tether
{

double Kinematics::distance3(const double a[3], const double b[3])
{
    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double dz = b[2] - a[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Kinematics::buildPositionVector(
    const std::array<BodyState, 2> & states, double pos[6])
{
    pos[0] = states[0].anchor_pos[0];
    pos[1] = states[0].anchor_pos[1];
    pos[2] = states[0].anchor_pos[2] - 100.0; // Z offset to trick MoorDyn into being underwater
    pos[3] = states[1].anchor_pos[0];
    pos[4] = states[1].anchor_pos[1];
    pos[5] = states[1].anchor_pos[2] - 100.0; // Z offset to trick MoorDyn into being underwater
}

void Kinematics::buildVelocityVector(
    const std::array<BodyState, 2> & states, double vel[6])
{
    double v_boat[3];
    computeAnchorVelocity(states[0], v_boat);
    vel[0] = v_boat[0];
    vel[1] = v_boat[1];
    vel[2] = v_boat[2];

    double v_drone[3];
    computeAnchorVelocity(states[1], v_drone);
    vel[3] = v_drone[0];
    vel[4] = v_drone[1];
    vel[5] = v_drone[2];
}

void Kinematics::rotateByQuat(const double q[4],
                                         const double v[3],
                                         double out[3])
{
    // Active rotation of v by unit quaternion q = [qx, qy, qz, qw].
    // Uses the Rodrigues formula via cross products.
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];

    const double tx = 2.0 * (qy * v[2] - qz * v[1]);
    const double ty = 2.0 * (qz * v[0] - qx * v[2]);
    const double tz = 2.0 * (qx * v[1] - qy * v[0]);

    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

void Kinematics::cross3(const double a[3],
                                   const double b[3],
                                   double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void Kinematics::computeAnchorVelocity(const BodyState & state,
                                                   double vel_world[3])
{
    // Step 1: Rotate body-frame linear velocity to world frame.
    double v_lin_world[3];
    rotateByQuat(state.quat, state.lin_vel_body, v_lin_world);

    // Step 2: Rotate body-frame angular velocity to world frame.
    double omega_world[3];
    rotateByQuat(state.quat, state.ang_vel_body, omega_world);

    // Step 3: Compute lever arm from body center to anchor point (world frame).
    const double r[3] = {
        state.anchor_pos[0] - state.body_center_pos[0],
        state.anchor_pos[1] - state.body_center_pos[1],
        state.anchor_pos[2] - state.body_center_pos[2]
    };

    // Step 4: Angular velocity contribution: ω_world × r
    double ang_contrib[3];
    cross3(omega_world, r, ang_contrib);

    // Step 5: Total anchor velocity in world frame.
    vel_world[0] = v_lin_world[0] + ang_contrib[0];
    vel_world[1] = v_lin_world[1] + ang_contrib[1];
    vel_world[2] = v_lin_world[2] + ang_contrib[2];
}

}  // namespace moordyn_tether
