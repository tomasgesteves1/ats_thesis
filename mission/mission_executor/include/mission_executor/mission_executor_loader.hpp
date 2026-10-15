#ifndef MISSION_EXECUTOR__MISSION_EXECUTOR_LOADER_HPP_
#define MISSION_EXECUTOR__MISSION_EXECUTOR_LOADER_HPP_

#include <map>
#include <string>
#include <vector>

namespace mission_executor
{

/// One step of a vehicle sequence as written in the mission file.
struct StepSpec
{
    std::string id;      ///< unique within the vehicle sequence (defaults to the type)
    std::string type;    ///< maneuver type
    std::string after;   ///< "<vehicle>.<id>": the step waits until that step is done (empty = no wait)
    std::map<std::string, std::vector<double>> params;
};

/// A mission: one sequence of maneuvers per vehicle.
struct MissionSpec
{
    std::string name;
    std::vector<StepSpec> uav;
    std::vector<StepSpec> usv;
};

/// Reads a mission file. Every key of a step other than id, type and after is a parameter of the maneuver:
/// numbers become vectors of size 1, lists of numbers are kept, and lists of lists are flattened.
/// Throws std::runtime_error with a description when the file is invalid.
MissionSpec loadMission(const std::string & path);

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__MISSION_EXECUTOR_LOADER_HPP_
