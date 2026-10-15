#include "mission_maneuvers/mission_maneuvers_maneuver.hpp"
#include "mission_maneuvers/mission_maneuvers_uav.hpp"
#include "mission_maneuvers/mission_maneuvers_usv.hpp"

namespace mission_maneuvers
{

std::unique_ptr<UavManeuver> createUavManeuver(const std::string & type, const Params & params)
{
    if (type == "takeoff") { return std::make_unique<Takeoff>(params); }
    if (type == "goto") { return std::make_unique<GotoBoat>(params); }
    if (type == "hold") { return std::make_unique<UavHold>(params); }
    if (type == "goto_world") { return std::make_unique<GotoWorld>(params); }
    if (type == "inspect") { return std::make_unique<Inspect>(params); }
    if (type == "climb") { return std::make_unique<Climb>(params); }
    if (type == "search") { return std::make_unique<Search>(params); }
    if (type == "chase") { return std::make_unique<Chase>(params); }
    if (type == "orbit") { return std::make_unique<Orbit>(params); }
    if (type == "land") { return std::make_unique<Land>(params); }
    return nullptr;
}

std::unique_ptr<UsvManeuver> createUsvManeuver(const std::string & type, const Params & params)
{
    if (type == "hold") { return std::make_unique<UsvHold>(params); }
    if (type == "waypoints") { return std::make_unique<UsvWaypoints>(params); }
    if (type == "follow") { return std::make_unique<UsvFollow>(params); }
    return nullptr;
}

}  // namespace mission_maneuvers
