#ifndef UAV_MPC__UAV_MPC_DISTURBANCE_HPP_
#define UAV_MPC__UAV_MPC_DISTURBANCE_HPP_

#include <array>

namespace uav_mpc
{

/// First-order observer of the external force that the predictive model does not explain (wind, model errors), for
/// offset-free tracking: at every step the velocity predicted by the previous solution for the current instant is
/// compared with the measured one, and the force estimate is corrected by
///     d_k = d_{k-1} + beta * m * (v_meas - v_pred) / T_s,
/// with its components bounded. The vertical component can be frozen (kept) while the horizontal ones keep being
/// updated, and the estimate can be reset.
class DisturbanceObserver
{
public:
    void configure(double gain, double max_force, double mass, double sampling_time);
    void update(const std::array<double, 3> & v_measured, const std::array<double, 3> & v_predicted,
                bool freeze_vertical = false);
    void reset() { d_ = {0.0, 0.0, 0.0}; }
    const std::array<double, 3> & estimate() const { return d_; }

private:
    double gain_{0.0};
    double max_force_{0.0};
    double mass_{1.0};
    double ts_{0.02};
    std::array<double, 3> d_{0.0, 0.0, 0.0};
};

}  // namespace uav_mpc

#endif  // UAV_MPC__UAV_MPC_DISTURBANCE_HPP_
