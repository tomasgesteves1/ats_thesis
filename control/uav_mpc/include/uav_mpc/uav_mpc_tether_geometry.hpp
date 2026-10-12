#pragma once

#include <vector>
#include <utility>

namespace uav_mpc {
namespace tether {

struct BoatState {
    double px{0.0};
    double py{0.0};
    double pz{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
};

/**
 * Get interpolated boat state (position and velocity) at a given stage time.
 */
BoatState getInterpolatedBoatState(
    int stage_idx, double Ts, double anchor_x, double anchor_y, double anchor_z,
    bool has_boat_horizon, const std::vector<std::vector<double>>& boat_horizon);

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
