#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "mission_manager/mission_manager_registry.hpp"
#include "mission_manager/mission_manager_executor.hpp"

namespace mission_manager {

struct ParameterUpdate {
    std::string node;
    std::string param;
    std::string value;
};

class MissionManagerPipeline {
public:
    MissionManagerPipeline();
    ~MissionManagerPipeline() = default;

    // Load missions from folder
    bool initialize(const std::string& missions_dir);

    // Get list of all missions
    std::vector<MissionManifest> listMissions() const;

    // Start simulation for a mission
    bool startSimulation(const std::string& mission_id,
                         const std::map<std::string, std::string>& user_params,
                         std::string& error_msg);

    // Stop simulation
    bool stopSimulation(std::string& error_msg);

    // Start a mission with specific user parameters and collect pending params to set dynamically
    bool startMission(const std::string& mission_id,
                      const std::map<std::string, std::string>& user_params,
                      std::vector<ParameterUpdate>& pending_params,
                      std::string& error_msg);

    // Stop currently running mission
    bool stopMission(std::string& error_msg);

    // Tick health updates
    void tick();

    // Get status info
    void getStatus(std::string& control_state, std::string& sim_state, std::string& active_id, std::string& active_name) const;

private:
    std::unique_ptr<MissionManagerRegistry> registry_;
    std::unique_ptr<MissionManagerExecutor> sim_executor_;
    std::unique_ptr<MissionManagerExecutor> control_executor_;

    std::string stateToString(ExecutorState state) const;
    void handleSimExecutorStateChange(ExecutorState state, const std::string& msg);
    void handleControlExecutorStateChange(ExecutorState state, const std::string& msg);
};

} // namespace mission_manager
