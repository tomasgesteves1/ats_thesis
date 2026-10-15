#ifndef MISSION_MANEUVERS__MISSION_MANEUVERS_USV_HPP_
#define MISSION_MANEUVERS__MISSION_MANEUVERS_USV_HPP_

#include <vector>

#include "mission_maneuvers/mission_maneuvers_geometry.hpp"
#include "mission_maneuvers/mission_maneuvers_maneuver.hpp"

namespace mission_maneuvers
{

/// Keeps the position and heading the boat had when the maneuver started.
/// Params: duration [s] (0 = finishes at once, the boat keeps holding while it is the last maneuver).
class UsvHold : public UsvManeuver
{
public:
    using UsvManeuver::UsvManeuver;
    std::string type() const override { return "hold"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UsvCommand & cmd) override;

private:
    double t0_{0.0};
    UsvPose pose_;
};

/// Follows the polyline from the current position through the waypoints (world frame) at a cruise speed reached
/// and left with minimum-jerk ramps, and stops at the last waypoint. The heading points to a look-ahead point on
/// the path, so it turns smoothly at the corners.
/// Params: points [x1, y1, x2, y2, ...] (m), speed [m/s], ramp_time [s], lookahead [m], tolerance [m].
class UsvWaypoints : public UsvManeuver
{
public:
    using UsvManeuver::UsvManeuver;
    std::string type() const override { return "waypoints"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UsvCommand & cmd) override;

private:
    std::array<double, 2> pointAt(double s) const;
    double t0_{0.0};
    std::vector<std::array<double, 2>> points_;
    std::vector<double> cumulative_;   ///< path length at each vertex
    TravelProfile travel_;
    double final_yaw_{0.0};
};

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_USV_HPP_
