#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include "mission_manager/mission_manager_pipeline.hpp"

// Interfaces
#include "mission_manager_interfaces/srv/list_missions.hpp"
#include "mission_manager_interfaces/srv/start_mission.hpp"
#include "mission_manager_interfaces/srv/stop_mission.hpp"
#include "mission_manager_interfaces/srv/get_mission_status.hpp"
#include "mission_manager_interfaces/srv/start_simulation.hpp"
#include "mission_manager_interfaces/srv/stop_simulation.hpp"

namespace mission_manager {

class MissionManagerNode : public rclcpp::Node {
public:
    MissionManagerNode();
    ~MissionManagerNode() = default;

private:
    std::unique_ptr<MissionManagerPipeline> pipeline_;
    
    // Services
    rclcpp::Service<mission_manager_interfaces::srv::ListMissions>::SharedPtr list_srv_;
    rclcpp::Service<mission_manager_interfaces::srv::StartMission>::SharedPtr start_srv_;
    rclcpp::Service<mission_manager_interfaces::srv::StopMission>::SharedPtr stop_srv_;
    rclcpp::Service<mission_manager_interfaces::srv::GetMissionStatus>::SharedPtr status_srv_;
    rclcpp::Service<mission_manager_interfaces::srv::StartSimulation>::SharedPtr start_sim_srv_;
    rclcpp::Service<mission_manager_interfaces::srv::StopSimulation>::SharedPtr stop_sim_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Queue of pending parameter updates to apply at runtime
    std::vector<ParameterUpdate> pending_params_;
    
    // Map of service clients to set parameters on other nodes
    std::map<std::string, rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr> param_clients_;

    // Service Callbacks
    void handleListMissions(const std::shared_ptr<mission_manager_interfaces::srv::ListMissions::Request> request,
                            std::shared_ptr<mission_manager_interfaces::srv::ListMissions::Response> response);

    void handleStartMission(const std::shared_ptr<mission_manager_interfaces::srv::StartMission::Request> request,
                            std::shared_ptr<mission_manager_interfaces::srv::StartMission::Response> response);

    void handleStopMission(const std::shared_ptr<mission_manager_interfaces::srv::StopMission::Request> request,
                           std::shared_ptr<mission_manager_interfaces::srv::StopMission::Response> response);

    void handleGetMissionStatus(const std::shared_ptr<mission_manager_interfaces::srv::GetMissionStatus::Request> request,
                                std::shared_ptr<mission_manager_interfaces::srv::GetMissionStatus::Response> response);

    void handleStartSimulation(const std::shared_ptr<mission_manager_interfaces::srv::StartSimulation::Request> request,
                               std::shared_ptr<mission_manager_interfaces::srv::StartSimulation::Response> response);

    void handleStopSimulation(const std::shared_ptr<mission_manager_interfaces::srv::StopSimulation::Request> request,
                              std::shared_ptr<mission_manager_interfaces::srv::StopSimulation::Response> response);

    // Timer Callback
    void timerCallback();

    // Helper to process pending parameters
    void processPendingParameters();
};

} // namespace mission_manager
