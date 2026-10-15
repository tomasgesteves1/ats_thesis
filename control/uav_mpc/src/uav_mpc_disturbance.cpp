#include "uav_mpc/uav_mpc_disturbance.hpp"

#include <algorithm>

namespace uav_mpc
{

void DisturbanceObserver::configure(double gain, double max_force, double mass, double sampling_time)
{
    gain_ = gain;
    max_force_ = max_force;
    mass_ = mass;
    ts_ = sampling_time;
}

void DisturbanceObserver::update(const std::array<double, 3> & v_measured, const std::array<double, 3> & v_predicted,
                                 bool freeze_vertical)
{
    for (int i = 0; i < (freeze_vertical ? 2 : 3); ++i) {
        // force that the model did not explain over the last step
        const double residual = mass_ * (v_measured[i] - v_predicted[i]) / ts_;
        d_[i] = std::clamp(d_[i] + gain_ * residual, -max_force_, max_force_);
    }
}

}  // namespace uav_mpc
