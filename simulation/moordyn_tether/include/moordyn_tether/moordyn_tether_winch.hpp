#ifndef MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_

#include <string>
#include <array>
#include <vector>
#include "moordyn/MoorDyn2.h"
#include "moordyn_tether/moordyn_tether_kinematics.hpp"

namespace moordyn_tether
{

/// @brief Modes of operation for the virtual winch
enum class WinchMode {
    GEOMETRIC,  ///< Length is based on straight-line distance + slack
    TENSION     ///< Length is dynamically adjusted to maintain a target tension magnitude
};

/// @brief Configuration for the virtual winch.
struct WinchConfig
{
    bool      enabled{false};
    WinchMode mode{WinchMode::GEOMETRIC};

    // --- Geometric Mode Parameters ---
    double slack_factor{1.15};   ///< Multiplier on straight-line distance (>1.0).

    // --- Tension Mode Parameters ---
    double target_tension{1.0};   ///< Target tension to maintain [N].
    double kp_tension{1.0};       ///< Proportional gain for tension control [m/s per N].
    double kd_tension{0.1};       ///< Derivative gain to damp oscillations [m/s per (N/s)].
    double winch_speed_limit{10.0}; ///< Max spooling speed [m/s].

    // --- Constraints ---
    double min_length{3.0};      ///< Minimum unstretched length [m].
    double max_length{100.0};    ///< Maximum unstretched length [m].
};

/// @brief Virtual Winch logic for adjusting the tether length.
class VirtualWinch
{
public:
    VirtualWinch() = default;

    /// Update the winch configuration.
    void setConfig(const WinchConfig & config);

    /// Get current configuration.
    const WinchConfig & getConfig() const { return cfg_; }

    /// @brief Computes the desired unstretched length and applies it to MoorDyn.
    void update(MoorDynLine line, const std::array<BodyState, 2> & body_states, double dt, const std::vector<double>& forces);

private:
    WinchConfig cfg_;
    double last_error_{0.0};
    bool   has_last_error_{false};
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_
