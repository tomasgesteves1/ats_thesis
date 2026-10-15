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
    std::string until;   ///< "<vehicle>.<id>": the step is ended when that step is done (empty = by itself)
    std::map<std::string, std::vector<double>> params;
};

/// A mission: one sequence of maneuvers per vehicle, and the UAV sequence run if the mission is aborted (a
/// maneuver failed) while the UAV is armed.
struct MissionSpec
{
    std::string name;
    double timeout{0.0};   ///< maximum duration (s, simulation time; 0: the executor default)
    std::vector<StepSpec> uav;
    std::vector<StepSpec> usv;
    std::vector<StepSpec> uav_abort;
};

/// Reads a mission file. Every key of a step other than id, type, after and until is a parameter of the maneuver:
/// numbers become vectors of size 1, lists of numbers are kept, and lists of lists are flattened.
/// Throws std::runtime_error with a description when the file is invalid.
MissionSpec loadMission(const std::string & path);

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__MISSION_EXECUTOR_LOADER_HPP_
