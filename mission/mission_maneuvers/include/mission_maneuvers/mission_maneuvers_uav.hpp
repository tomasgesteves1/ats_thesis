#ifndef MISSION_MANEUVERS__MISSION_MANEUVERS_UAV_HPP_
#define MISSION_MANEUVERS__MISSION_MANEUVERS_UAV_HPP_

#include "mission_maneuvers/mission_maneuvers_geometry.hpp"
#include "mission_maneuvers/mission_maneuvers_maneuver.hpp"

namespace mission_maneuvers
{

/// Settling test: true once a condition has held continuously for a given time.
class SettleTimer
{
public:
    bool update(bool condition, double t, double hold_time);
    void reset() { since_ = -1.0; }

private:
    double since_{-1.0};
};

/// Arms the UAV in offboard mode on the pad and climbs: first vertically off the pad, keeping its horizontal
/// position on it, then with a minimum-jerk transfer to a point given relative to the winch. The winch pulls the
/// UAV towards the pad until it is pull_release_height above the winch (it should be released within the vertical
/// leg). The yaw follows the boat heading. Done when the transfer is over and the UAV is within tolerance.
/// Params: vertical_clearance [m], clearance_duration [s], pull_release_height [m], offset [m] (relative to the
/// winch), duration [s] (of the transfer), tolerance [m], arm_timeout [s].
class Takeoff : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "takeoff"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    Vec3 relAt(double t) const;
    bool climbing_{false};
    bool pull_released_{false};
    double t_start_{0.0};
    double t0_{0.0};
    Vec3 rel0_{};
};

/// Minimum-jerk transfer to a point given relative to the winch, then waits until the UAV is settled there.
/// Params: offset [m], duration [s], tolerance [m], velocity_tolerance [m/s], settle [s], timeout [s] (after the
/// transfer; the maneuver then finishes anyway), yaw_from_reference (0/1).
class GotoBoat : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "goto"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    Vec3 rel0_{};
    SettleTimer settle_;
};

/// Orbit around the winch: minimum-jerk entry to the circle at the current bearing, then the angular speed is
/// ramped up, kept, and ramped down to stop after the requested laps. The UAV looks at the winch.
/// Params: radius [m], height [m], omega [rad/s] (sign = direction), laps, entry_duration [s], ramp_time [s].
class Orbit : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "orbit"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    Vec3 relAt(double t) const;
    double t0_{0.0};
    Vec3 rel0_{};
    double theta0_{0.0};
    double direction_{1.0};
    TravelProfile angle_;   ///< travelled angle (rad) after the entry
};

/// Tether-pulled landing on the pad above the winch: approach above the winch, stay centred (horizontal
/// tolerance) until settled, minimum-jerk descent to final_height above the pad with the landing pull requested,
/// slow final approach at final_speed down to depth below the pad, touchdown detection relative to the current
/// winch position and disarm. The yaw follows the boat heading.
/// Params: height [m], approach_duration [s], tolerance [m], velocity_tolerance [m/s], settle [s],
/// track_timeout [s], descent_duration [s], final_height [m], final_speed [m/s], final_ramp [s],
/// depth [m] (final reference below the deck, covers the heave),
/// touch_margin [m], touch_velocity [m/s], touch_time [s], disarm_timeout [s].
class Land : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "land"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    enum class Phase { kApproach, kTrack, kDescend, kTouch, kLanded };
    Vec3 relAt(double t) const;
    Phase phase_{Phase::kApproach};
    double t_phase_{0.0};
    double rest_height_{0.0};
    TravelProfile final_approach_;   ///< slow final approach to the pad
    Vec3 rel0_{};
    SettleTimer settle_;
};

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_UAV_HPP_
