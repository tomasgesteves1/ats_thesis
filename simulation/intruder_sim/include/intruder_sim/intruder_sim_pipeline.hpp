#ifndef INTRUDER_SIM__INTRUDER_SIM_PIPELINE_HPP_
#define INTRUDER_SIM__INTRUDER_SIM_PIPELINE_HPP_

#include <array>
#include <string>
#include <vector>

namespace intruder_sim
{

/// Behaviour of the intruder boat.
struct IntruderConfig
{
    std::vector<std::array<double, 2>> patrol;   ///< patrol loop (m); the boat starts at its first point
    double patrol_speed{0.6};      ///< m/s
    double flee_speed{1.0};        ///< m/s
    double acceleration{0.3};      ///< m/s^2
    double turn_rate{0.3};         ///< rad/s
    double waypoint_radius{2.0};   ///< m, a patrol point is reached within this distance
    double alert_radius{10.0};     ///< m, the intruder flees when the UAV or the USV gets closer (horizontal)
    double capture_radius{8.0};    ///< m, the intruder stops when the USV stays closer ...
    double capture_time{3.0};      ///< s, ... for this time
    std::array<double, 4> zone{0.0, 100.0, -50.0, 50.0};   ///< x_min, x_max, y_min, y_max (m) it does not leave
    double zone_margin{8.0};       ///< m, near the edge of the zone the flight direction is turned inwards
};

enum class Mode { kPatrol, kFlee, kCaptured };

/// Kinematic state (world frame).
struct IntruderState
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
    Mode mode{Mode::kPatrol};
};

/// Pure C++ behaviour of the intruder: unicycle with limited acceleration and turn rate that patrols a loop, flees
/// away from the USV once the UAV or the USV comes within the alert radius (staying inside its zone), and stops
/// when the USV has stayed within the capture radius.
class IntruderSimPipeline
{
public:
    explicit IntruderSimPipeline(const IntruderConfig & config);

    /// Advances dt seconds given the horizontal positions of the pursuers; returns an event to log (or empty).
    std::string step(double dt, const std::array<double, 2> & uav, const std::array<double, 2> & usv);

    const IntruderState & state() const { return state_; }

private:
    double fleeHeading(const std::array<double, 2> & usv) const;

    IntruderConfig cfg_;
    IntruderState state_;
    size_t waypoint_{1};
    double capture_timer_{0.0};
};

}  // namespace intruder_sim

#endif  // INTRUDER_SIM__INTRUDER_SIM_PIPELINE_HPP_
