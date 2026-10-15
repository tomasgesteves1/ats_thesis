#ifndef MISSION_MANEUVERS__MISSION_MANEUVERS_GEOMETRY_HPP_
#define MISSION_MANEUVERS__MISSION_MANEUVERS_GEOMETRY_HPP_

#include <functional>
#include <vector>

#include "mission_maneuvers/mission_maneuvers_types.hpp"

namespace mission_maneuvers
{

/// Minimum-jerk profile on [0, 1] (zero velocity and acceleration at both ends); s is clamped.
double minJerk(double s);

/// Integral of the minimum-jerk profile from 0 to s (s in [0, 1]); equals 0.5 at s = 1.
double minJerkIntegral(double s);

/// Travel along a path of given length: the speed is ramped up with a minimum-jerk profile, kept, and ramped down
/// to stop at the end (the ramp is shortened when the path is too short to reach the cruise speed).
class TravelProfile
{
public:
    TravelProfile() = default;
    TravelProfile(double length, double speed, double ramp_time);
    double distanceAt(double tau) const;   ///< distance travelled tau seconds after the start
    double duration() const { return 2.0 * ramp_ + cruise_; }

private:
    double length_{0.0};
    double speed_{0.0};
    double ramp_{0.0};
    double cruise_{0.0};
};

double norm(const Vec3 & a);
double norm2(double x, double y);
Vec3 add(const Vec3 & a, const Vec3 & b);
Vec3 sub(const Vec3 & a, const Vec3 & b);
Vec3 scale(const Vec3 & a, double k);
Vec3 lerp(const Vec3 & a, const Vec3 & b, double s);
double wrapAngle(double a);

/// Planar displacement of the boat between now and now + tau, along the horizon predicted by its controller
/// (constant velocity beyond it, or for the whole interval when the horizon is missing or stale).
Vec3 boatDisplacement(const Context & ctx, double tau);

/// Heading of the boat at now + tau, along the predicted horizon (constant yaw rate when it is missing or stale).
double boatYawAt(const Context & ctx, double tau);

/// Rotation of a vector about the vertical axis.
Vec3 rotateZ(const Vec3 & v, double yaw);

/// Vector expressed in the boat frame (current heading) from world axes.
Vec3 toBoatFrame(const Context & ctx, const Vec3 & v);

/// Winch position used as origin of the boat-relative references: current planar position, mean height.
Vec3 anchorReference(const Context & ctx);

/// Reference horizon of the UAV given relative to the winch: point k is anchorReference + boat displacement over
/// k*dt + rel(t + k*dt), with rel in world axes or, if boat_frame, in the axes of the boat (rotated with its
/// predicted heading). The heading of every point is the boat heading.
std::vector<UavPose> boatRelativePath(const Context & ctx, const std::function<Vec3(double)> & rel,
                                      bool boat_frame = false);

/// UAV position relative to the winch reference (anchorReference).
Vec3 uavRelative(const Context & ctx);

/// UAV velocity relative to the boat.
Vec3 uavRelativeVelocity(const Context & ctx);

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_GEOMETRY_HPP_
