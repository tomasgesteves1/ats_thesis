#include "uav_mpc/uav_mpc_tether_geometry.hpp"
#include <cmath>

namespace uav_mpc {
namespace tether {

BoatState getInterpolatedBoatState(
    int stage_idx, double Ts, double anchor_x, double anchor_y, double anchor_z,
    bool has_boat_horizon, const std::vector<std::vector<double>>& boat_horizon) 
{
    BoatState state;
    state.px = anchor_x;
    state.py = anchor_y;
    state.pz = anchor_z;
    state.vx = 0.0;
    state.vy = 0.0;
    state.vz = 0.0;

    if (has_boat_horizon && boat_horizon.size() >= 2) {
        // Lever arm / mounting offset between boat COM in horizon and anchor TF pose
        double offset_x = anchor_x - boat_horizon[0][0];
        double offset_y = anchor_y - boat_horizon[0][1];

        double t = stage_idx * Ts;
        double boat_Ts = 0.1; // USV MPC control period
        double idx_f = t / boat_Ts;
        size_t j = static_cast<size_t>(std::floor(idx_f));
        if (j < boat_horizon.size() - 1) {
            double alpha = idx_f - j;
            state.px = (1.0 - alpha) * boat_horizon[j][0] + alpha * boat_horizon[j+1][0] + offset_x;
            state.py = (1.0 - alpha) * boat_horizon[j][1] + alpha * boat_horizon[j+1][1] + offset_y;
            state.vx = (boat_horizon[j+1][0] - boat_horizon[j][0]) / boat_Ts;
            state.vy = (boat_horizon[j+1][1] - boat_horizon[j][1]) / boat_Ts;
        } else {
            state.px = boat_horizon.back()[0] + offset_x;
            state.py = boat_horizon.back()[1] + offset_y;
            size_t last = boat_horizon.size() - 1;
            state.vx = (boat_horizon[last][0] - boat_horizon[last-1][0]) / boat_Ts;
            state.vy = (boat_horizon[last][1] - boat_horizon[last-1][1]) / boat_Ts;
        }
    }
    return state;
}

std::pair<double, double> getInterpolatedBoatPos(
    int stage_idx, double Ts, double anchor_x, double anchor_y,
    bool has_boat_horizon, const std::vector<std::vector<double>>& boat_horizon) 
{
    auto state = getInterpolatedBoatState(stage_idx, Ts, anchor_x, anchor_y, 0.0, has_boat_horizon, boat_horizon);
    return {state.px, state.py};
}

void projectReferenceToTetherLimit(
    int stage_idx, double Ts, double rx, double ry, double rz,
    double anchor_x, double anchor_y, bool has_boat_horizon,
    const std::vector<std::vector<double>>& boat_horizon,
    double L_tether, bool use_tether,
    double& proj_x, double& proj_y, double& proj_z)
{
    if (!use_tether) {
        proj_x = rx;
        proj_y = ry;
        proj_z = rz;
        return;
    }
    auto b_pos = getInterpolatedBoatPos(stage_idx, Ts, anchor_x, anchor_y, has_boat_horizon, boat_horizon);
    double dx = rx - b_pos.first;
    double dy = ry - b_pos.second;
    double dz = rz; // Boat Z is assumed 0 in XY projection constraint
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    double L_safe = 1.0 * L_tether;
    if (d > L_safe) {
        double scale = L_safe / d;
        proj_x = b_pos.first + dx * scale;
        proj_y = b_pos.second + dy * scale;
        proj_z = dz * scale;
    } else {
        proj_x = rx;
        proj_y = ry;
        proj_z = rz;
    }
}

} // namespace tether
} // namespace uav_mpc
