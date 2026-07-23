#pragma once

#include <vector>
#include <utility>

namespace uav_mpc {
namespace tether {

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
