#pragma once

#include <vector>
#include <utility>

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

/**
 * Get interpolated boat position at a given stage time.
 */
std::pair<double, double> getInterpolatedBoatPos(
    int stage_idx, double Ts, double anchor_x, double anchor_y,
    bool has_boat_horizon, const std::vector<std::vector<double>>& boat_horizon);

/**
 * Project reference point to be within the safe sphere of tether length from the boat position.
 */
void projectReferenceToTetherLimit(
    int stage_idx, double Ts, double rx, double ry, double rz,
    double anchor_x, double anchor_y, bool has_boat_horizon,
    const std::vector<std::vector<double>>& boat_horizon,
    double L_tether, bool use_tether,
    double& proj_x, double& proj_y, double& proj_z);

} // namespace tether
} // namespace uav_mpc
