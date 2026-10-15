#ifndef MISSION_EXECUTOR__MISSION_EXECUTOR_PIPELINE_HPP_
#define MISSION_EXECUTOR__MISSION_EXECUTOR_PIPELINE_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "mission_executor/mission_executor_loader.hpp"
#include "mission_executor/mission_executor_sequence.hpp"
#include "mission_maneuvers/mission_maneuvers_maneuver.hpp"

namespace mission_executor
{

/// Configuration of the executor (sampling of the controllers' references).
struct PipelineConfig
{
    mission_maneuvers::HorizonSpec uav_horizon;
    mission_maneuvers::HorizonSpec usv_horizon;
    double horizon_timeout{0.5};
    double default_rest_height{0.26};   ///< used when the mission starts with the UAV already armed
};

/// Measurements of one cycle.
struct Measurements
{
    double t{0.0};
    mission_maneuvers::UavState uav;
    mission_maneuvers::BoatState boat;   ///< anchor_z_mean is filled by the pipeline
    mission_maneuvers::TargetState target;
};

/// Commands of one cycle and the events to log.
struct Output
{
    mission_maneuvers::UavCommand uav;
    mission_maneuvers::UsvCommand usv;
    std::vector<std::string> events;
};

enum class MissionState { kNotLoaded, kRunning, kDone, kAborted };

/// Pure C++ orchestrator: builds the maneuver context from the measurements (mean winch height, resting height
/// of the UAV on the pad) and runs the UAV and USV sequences of the mission. When a maneuver fails the mission is
/// aborted: the boat holds its position and, if the UAV is armed, the abort sequence of the mission (by default a
/// landing on the pad) is run.
class MissionExecutorPipeline
{
public:
    explicit MissionExecutorPipeline(const PipelineConfig & config);

    /// Creates the maneuvers of the mission; false with a description if a type is unknown.
    bool load(const MissionSpec & mission, std::string & error);

    Output tick(const Measurements & m);

    MissionState state() const { return state_; }
    const std::string & name() const { return name_; }

private:
    mission_maneuvers::Context buildContext(const Measurements & m);
    void startAbort(const Measurements & m, Output & out);

    PipelineConfig config_;
    std::string name_;
    MissionState state_{MissionState::kNotLoaded};
    std::unique_ptr<Sequence<mission_maneuvers::UavManeuver, mission_maneuvers::UavCommand>> uav_;
    std::unique_ptr<Sequence<mission_maneuvers::UsvManeuver, mission_maneuvers::UsvCommand>> usv_;
    std::set<std::string> done_;
    std::vector<StepSpec> abort_spec_;   ///< UAV sequence run on abort (default: land on the pad)
    std::unique_ptr<Sequence<mission_maneuvers::UavManeuver, mission_maneuvers::UavCommand>> uav_abort_;
    std::unique_ptr<Sequence<mission_maneuvers::UsvManeuver, mission_maneuvers::UsvCommand>> usv_abort_;

    double anchor_z_sum_{0.0};
    long anchor_z_count_{0};
    bool rest_height_set_{false};
    double rest_height_{0.0};
    bool usv_idle_set_{false};
    bool abort_reported_{false};
    mission_maneuvers::UsvPose usv_idle_;
};

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__MISSION_EXECUTOR_PIPELINE_HPP_
