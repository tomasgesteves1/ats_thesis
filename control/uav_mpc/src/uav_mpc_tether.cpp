#include "uav_mpc/uav_mpc_tether.hpp"
#include <cmath>

namespace uav_mpc {
namespace tether {

double calculateWinchTension(double L_tether, double base_tension, double linear_density) {
    const double g = 9.81;
    return base_tension + linear_density * L_tether * g;
}

double estimateTetherForce(double px, double py, double pz,
                           double anchor_x, double anchor_y, double anchor_z,
                           double T0_val, double eps) {
    double dx = anchor_x - px;
    double dy = anchor_y - py;
    double dz = anchor_z - pz;
    double s_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double s_norm_eps = std::sqrt(s_dist*s_dist + eps*eps);
    return T0_val * (s_dist / s_norm_eps);
}

std::pair<double, double> getInterpolatedBoatPos(
    int stage_idx, double Ts, double anchor_x, double anchor_y,
    bool has_boat_horizon, const std::vector<std::vector<double>>& boat_horizon) 
{
    double x_b = anchor_x;
    double y_b = anchor_y;
    if (has_boat_horizon && !boat_horizon.empty()) {
        double t = stage_idx * Ts;
        double boat_Ts = 0.1; // USV MPC control period
        double idx_f = t / boat_Ts;
        size_t j = static_cast<size_t>(std::floor(idx_f));
        if (j < boat_horizon.size() - 1) {
            double alpha = idx_f - j;
            x_b = (1.0 - alpha) * boat_horizon[j][0] + alpha * boat_horizon[j+1][0];
            y_b = (1.0 - alpha) * boat_horizon[j][1] + alpha * boat_horizon[j+1][1];
        } else {
            x_b = boat_horizon.back()[0];
            y_b = boat_horizon.back()[1];
        }
    }
    return {x_b, y_b};
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
