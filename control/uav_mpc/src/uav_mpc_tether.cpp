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

} // namespace tether
} // namespace uav_mpc
