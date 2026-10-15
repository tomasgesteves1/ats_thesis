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

/// Minimum-jerk transfer to a point given relative to the winch, in world axes or in the axes of the boat (the
/// point then turns with the boat: scout ahead of the bow, escort beside it), then waits until the UAV is settled.
/// Params: offset [m], boat_frame (0/1), duration [s], tolerance [m], velocity_tolerance [m/s], settle [s],
/// timeout [s] (after the transfer), fail_on_timeout (0: finishes anyway, 1: fails), yaw_from_reference (0/1).
class GotoBoat : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "goto"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    bool boat_frame_{false};
    Vec3 rel0_{};
    SettleTimer settle_;
};

/// Keeps a point relative to the winch (world or boat axes; the current one if no offset is given) for a duration
/// (0: until the step is ended by the mission, see 'until').
/// Params: offset [m], boat_frame (0/1), duration [s], yaw_from_reference (0/1).
class UavHold : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "hold"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    bool boat_frame_{false};
    Vec3 offset_{};
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

/// Minimum-jerk transfer to a point fixed in the world, then waits until the UAV is settled there. With look_at the
/// heading points to that point (e.g. a structure).
/// Params: position [m], duration [s], tolerance [m], velocity_tolerance [m/s], settle [s], timeout [s],
/// fail_on_timeout (0/1), look_at [x, y] (m).
class GotoWorld : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "goto_world"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    Vec3 p0_{};
    SettleTimer settle_;
};

/// Inspection arc around a structure: minimum-jerk entry to the start of the arc, then the UAV travels along the arc
/// (centre fixed in the world) from angle_start to angle_end while its height goes from height_start to height_end
/// (a helix), with the angular speed ramped up and down; the heading points to the centre.
/// Params: center [x, y] (m), radius [m], angle_start, angle_end [deg] (world), height_start, height_end [m]
/// (world), angular_speed [rad/s], ramp_time [s], entry_duration [s].
class Inspect : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "inspect"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    Vec3 worldAt(double t) const;
    double t0_{0.0};
    Vec3 p0_{};
    TravelProfile arc_;   ///< travelled angle (rad)
};

/// Vertical climb at a given speed up to a target height (world), to probe the height the UAV can reach with the
/// tether. When the UAV falls behind the reference by more than stall_error for stall_time (thrust saturated by the
/// weight of the tether, or tether range exhausted) the climb stops there and the maximum height is reported.
/// The climb is over when the height is within tolerance of the target, or settle_timeout after the reference
/// reached it.
/// Params: position [x, y] (m, the current one if not given), height [m], speed [m/s], ramp_time [s],
/// stall_error [m], stall_time [s], tolerance [m], settle_timeout [s], look_at [x, y] (m).
class Climb : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "climb"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    double x_{0.0};
    double y_{0.0};
    double z0_{0.0};
    double z_max_{0.0};
    bool stalled_{false};
    double z_hold_{0.0};
    TravelProfile rise_;
    SettleTimer stall_;
};

/// Search for the target while the boat moves: the UAV sweeps across the track of the boat (zig-zag in the boat
/// frame, ahead of the winch) and the maneuver ends when the target enters the field of view of a downward camera
/// (horizontal distance below height * tan(fov / 2)). Fails if the target is not found within the timeout.
/// Params: ahead [m], amplitude [m], period [s], height [m] (above the winch), fov [deg], entry_duration [s],
/// timeout [s] (0: none).
class Search : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "search"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    Vec3 relAt(double t) const;
    double t0_{0.0};
    Vec3 rel0_{};
};

/// Keeps the UAV above the target (predicted with constant velocity along the horizon), looking at it, with the
/// horizontal distance from the winch limited by the tether range; minimum-jerk entry from the current position.
/// Runs until it is ended by the mission ('until').
/// Params: height [m] (above the winch), max_horizontal [m] (from the winch), entry_duration [s].
class Chase : public UavManeuver
{
public:
    using UavManeuver::UavManeuver;
    std::string type() const override { return "chase"; }
    void start(const Context & ctx) override;
    Status update(const Context & ctx, UavCommand & cmd) override;

private:
    double t0_{0.0};
    Vec3 rel0_{};
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
    double t_descent_start_{0.0};    ///< the descent profile is kept through touchdown and after it
    TravelProfile final_approach_;   ///< slow final approach to the pad
    Vec3 rel0_{};
    SettleTimer settle_;
};

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_UAV_HPP_
