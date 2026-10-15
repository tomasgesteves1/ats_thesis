#ifndef MISSION_MANEUVERS__MISSION_MANEUVERS_TYPES_HPP_
#define MISSION_MANEUVERS__MISSION_MANEUVERS_TYPES_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

namespace mission_maneuvers
{

using Vec3 = std::array<double, 3>;

/// Outcome of a maneuver update.
enum class Status { kRunning, kDone, kFailed };

/// Sampling of a reference horizon expected by a controller (number of points and spacing).
struct HorizonSpec
{
    int steps{51};
    double dt{0.02};
};

/// State of the UAV (world frame).
struct UavState
{
    Vec3 p{};
    Vec3 v{};
    double yaw{0.0};
    bool armed{false};
    bool offboard{false};
};

/// Horizon predicted by the USV controller: planar positions and headings sampled every dt from stamp.
struct BoatHorizon
{
    double stamp{0.0};
    double dt{0.1};
    std::vector<std::array<double, 3>> x_y_yaw;
};

/// State of the USV and of the tether winch on it (world frame).
struct BoatState
{
    Vec3 p{};
    Vec3 v{};
    double yaw{0.0};
    double yaw_rate{0.0};
    Vec3 anchor{};              ///< current position of the winch
    double anchor_z_mean{0.0};  ///< running mean of the winch height (the references ignore the heave)
    BoatHorizon horizon;
};

/// Everything a maneuver may read.
struct Context
{
    double t{0.0};                   ///< time (s)
    UavState uav;
    BoatState boat;
    double uav_rest_height{0.0};     ///< height of the UAV above the winch when resting on the landing pad (m)
    HorizonSpec uav_horizon;
    HorizonSpec usv_horizon;
    double horizon_timeout{0.5};     ///< a boat horizon older than this is not used (s)
};

/// Reference pose of the UAV (world frame); only the yaw of the pose is used by the controller.
struct UavPose
{
    Vec3 p{};
    double yaw{0.0};
};

/// Reference pose of the USV (world frame).
struct UsvPose
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

/// Output of a UAV maneuver: reference horizon and requests to the autopilot, the winch and the controller.
struct UavCommand
{
    std::vector<UavPose> path;
    bool request_offboard{false};
    bool request_arm{false};
    bool request_disarm{false};
    bool tether_pull{false};         ///< the winch pulls the UAV towards the deck (takeoff and landing)
    bool yaw_from_reference{false};  ///< the UAV yaw follows the reference heading (the boat heading)
    bool low_altitude{false};        ///< the lower bound on the height is relaxed (takeoff and landing)
    std::string message;             ///< event to log (empty if none)
};

/// Output of a USV maneuver.
struct UsvCommand
{
    std::vector<UsvPose> path;
    std::string message;
};

/// Parameters of a maneuver, as read from the mission file (scalars are vectors of size 1).
class Params
{
public:
    Params() = default;
    explicit Params(std::map<std::string, std::vector<double>> values) : values_(std::move(values)) {}

    double get(const std::string & key, double fallback) const;
    std::vector<double> getVector(const std::string & key, const std::vector<double> & fallback) const;
    Vec3 getVec3(const std::string & key, const Vec3 & fallback) const;
    bool has(const std::string & key) const { return values_.count(key) > 0; }

private:
    std::map<std::string, std::vector<double>> values_;
};

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_TYPES_HPP_
