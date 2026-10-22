#ifndef MISSION_MANEUVERS__MISSION_MANEUVERS_MANEUVER_HPP_
#define MISSION_MANEUVERS__MISSION_MANEUVERS_MANEUVER_HPP_

#include <memory>
#include <string>

#include "mission_maneuvers/mission_maneuvers_types.hpp"

namespace mission_maneuvers
{

/// A UAV maneuver produces, at every update, the reference horizon of the UAV NMPC and the requests to the
/// autopilot and the winch, and reports when it is finished. After kDone it may keep being updated (it is then the
/// last maneuver of the sequence) and must keep producing a valid reference.
class UavManeuver
{
public:
    explicit UavManeuver(Params params) : params_(std::move(params)) {}
    virtual ~UavManeuver() = default;

    virtual std::string type() const = 0;
    virtual void start(const Context & ctx) = 0;
    virtual Status update(const Context & ctx, UavCommand & cmd) = 0;

protected:
    Params params_;
};

/// Same contract for the USV.
class UsvManeuver
{
public:
    explicit UsvManeuver(Params params) : params_(std::move(params)) {}
    virtual ~UsvManeuver() = default;

    virtual std::string type() const = 0;
    virtual void start(const Context & ctx) = 0;
    virtual Status update(const Context & ctx, UsvCommand & cmd) = 0;

protected:
    Params params_;
};

/// Creates the maneuver of the given type ("takeoff", "goto", "hold", "goto_world", "orbit",
/// "inspect", "climb", "search", "chase", "land"); nullptr if unknown.
std::unique_ptr<UavManeuver> createUavManeuver(const std::string & type, const Params & params);

/// Creates the maneuver of the given type ("hold", "waypoints", "follow", "support"); nullptr if unknown.
std::unique_ptr<UsvManeuver> createUsvManeuver(const std::string & type, const Params & params);

}  // namespace mission_maneuvers

#endif  // MISSION_MANEUVERS__MISSION_MANEUVERS_MANEUVER_HPP_
