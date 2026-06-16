#pragma once

namespace uav_mpc {
namespace tether {

/**
 * Calculate the winch tension (T0) based on tether length and density.
 */
double calculateWinchTension(double L_tether, double base_tension = 0.1, double linear_density = 0.020);

/**
 * Estimate the tether force magnitude assumed by the MPC model.
 */
double estimateTetherForce(double px, double py, double pz,
                           double anchor_x, double anchor_y, double anchor_z,
                           double T0_val, double eps = 0.02);

} // namespace tether
} // namespace uav_mpc
