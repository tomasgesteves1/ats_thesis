#include "mission_manager/mission_manager_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

namespace mission_manager {

MissionManagerNode::MissionManagerNode()
    : Node("mission_manager_node")
{
    // Determine default missions directory using ament_index_cpp
    std::string default_missions_dir = "";
    try {
        default_missions_dir = ament_index_cpp::get_package_share_directory("mission_manager") + "/missions";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find share directory for mission_manager: %s. Using default path.", e.what());
        default_missions_dir = "src/mission/mission_manager/missions";
    }

    // Declare parameter (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<std::string>("missions_dir", default_missions_dir);
    std::string missions_dir = this->get_parameter("missions_dir").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading mission manifests from: %s", missions_dir.c_str());

    // Initialize pipeline
    pipeline_ = std::make_unique<MissionManagerPipeline>();
    if (!pipeline_->initialize(missions_dir)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize pipeline with directory: %s", missions_dir.c_str());
    }

    // Create services (Rule 4 of CODE_STANDARDS.md relative naming)
    list_srv_ = this->create_service<mission_manager_interfaces::srv::ListMissions>(
        "~/list_missions",
        std::bind(&MissionManagerNode::handleListMissions, this, std::placeholders::_1, std::placeholders::_2)
    );

    start_srv_ = this->create_service<mission_manager_interfaces::srv::StartMission>(
        "~/start_mission",
        std::bind(&MissionManagerNode::handleStartMission, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_srv_ = this->create_service<mission_manager_interfaces::srv::StopMission>(
        "~/stop_mission",
        std::bind(&MissionManagerNode::handleStopMission, this, std::placeholders::_1, std::placeholders::_2)
    );

    status_srv_ = this->create_service<mission_manager_interfaces::srv::GetMissionStatus>(
        "~/get_mission_status",
        std::bind(&MissionManagerNode::handleGetMissionStatus, this, std::placeholders::_1, std::placeholders::_2)
    );

    start_sim_srv_ = this->create_service<mission_manager_interfaces::srv::StartSimulation>(
        "~/start_simulation",
        std::bind(&MissionManagerNode::handleStartSimulation, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_sim_srv_ = this->create_service<mission_manager_interfaces::srv::StopSimulation>(
        "~/stop_simulation",
        std::bind(&MissionManagerNode::handleStopSimulation, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Timer at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MissionManagerNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Mission Manager Node initialized.");
}

void MissionManagerNode::handleListMissions(
    const std::shared_ptr<mission_manager_interfaces::srv::ListMissions::Request>,
    std::shared_ptr<mission_manager_interfaces::srv::ListMissions::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request: list_missions");
    auto manifests = pipeline_->listMissions();
    for (const auto& m : manifests) {
        mission_manager_interfaces::msg::MissionDefinition def;
        def.id = m.id;
        def.name = m.name;
        def.description = m.description;
        def.category = m.category;
        def.icon = m.icon;

        for (const auto& p : m.user_params) {
            mission_manager_interfaces::msg::UserParam up;
            up.key = p.key;
            up.label = p.label;
            up.type = p.type;
            up.default_value = p.default_value;
            up.min_value = p.min_value;
            up.max_value = p.max_value;
            up.unit = p.unit;
            up.options = p.options;
            def.user_params.push_back(up);
        }
        response->missions.push_back(def);
    }
}

void MissionManagerNode::handleStartMission(
    const std::shared_ptr<mission_manager_interfaces::srv::StartMission::Request> request,
    std::shared_ptr<mission_manager_interfaces::srv::StartMission::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request: start_mission (id: %s)", request->mission_id.c_str());

    std::map<std::string, std::string> user_params;
    if (request->param_keys.size() == request->param_values.size()) {
        for (size_t i = 0; i < request->param_keys.size(); ++i) {
            user_params[request->param_keys[i]] = request->param_values[i];
        }
    } else {
        response->success = false;
        response->message = "Mismatched parameter keys and values arrays size.";
        return;
    }

    std::string error_msg;
    bool success = pipeline_->startMission(request->mission_id, user_params, pending_params_, error_msg);
    response->success = success;
    response->message = success ? "Mission started successfully." : error_msg;

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Mission %s started. Applying %zu dynamic parameters...", 
                    request->mission_id.c_str(), pending_params_.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to start mission %s: %s", request->mission_id.c_str(), error_msg.c_str());
    }
}

void MissionManagerNode::handleStopMission(
    const std::shared_ptr<mission_manager_interfaces::srv::StopMission::Request>,
    std::shared_ptr<mission_manager_interfaces::srv::StopMission::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request: stop_mission");
    std::string error_msg;
    bool success = pipeline_->stopMission(error_msg);
    response->success = success;
    response->message = success ? "Mission stopped." : error_msg;
    
    // Clear pending updates
    pending_params_.clear();
}

void MissionManagerNode::handleGetMissionStatus(
    const std::shared_ptr<mission_manager_interfaces::srv::GetMissionStatus::Request>,
    std::shared_ptr<mission_manager_interfaces::srv::GetMissionStatus::Response> response)
{
    pipeline_->getStatus(response->control_state, response->sim_state, response->active_mission_id, response->active_mission_name);
}

void MissionManagerNode::handleStartSimulation(
    const std::shared_ptr<mission_manager_interfaces::srv::StartSimulation::Request> request,
    std::shared_ptr<mission_manager_interfaces::srv::StartSimulation::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request: start_simulation (mission_id: %s)", request->mission_id.c_str());

    std::map<std::string, std::string> user_params;
    if (request->param_keys.size() == request->param_values.size()) {
        for (size_t i = 0; i < request->param_keys.size(); ++i) {
            user_params[request->param_keys[i]] = request->param_values[i];
        }
    } else {
        response->success = false;
        response->message = "Mismatched parameter keys and values arrays size.";
        return;
    }

    std::string error_msg;
    bool success = pipeline_->startSimulation(request->mission_id, user_params, error_msg);
    response->success = success;
    response->message = success ? "Simulation started successfully." : error_msg;
}

void MissionManagerNode::handleStopSimulation(
    const std::shared_ptr<mission_manager_interfaces::srv::StopSimulation::Request>,
    std::shared_ptr<mission_manager_interfaces::srv::StopSimulation::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request: stop_simulation");
    std::string error_msg;
    bool success = pipeline_->stopSimulation(error_msg);
    response->success = success;
    response->message = success ? "Simulation stopped." : error_msg;
}

void MissionManagerNode::timerCallback()
{
    pipeline_->tick();
    processPendingParameters();
}

void MissionManagerNode::processPendingParameters()
{
    if (pending_params_.empty()) {
        return;
    }

    for (auto it = pending_params_.begin(); it != pending_params_.end(); ) {
        const auto& update = *it;
        
        // Ensure client exists
        auto client_it = param_clients_.find(update.node);
        if (client_it == param_clients_.end()) {
            std::string service_name = "/" + update.node + "/set_parameters";
            auto client = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);
            param_clients_[update.node] = client;
            client_it = param_clients_.find(update.node);
        }

        auto client = client_it->second;
        if (client->service_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Setting dynamic parameter: %s::%s = %s", 
                        update.node.c_str(), update.param.c_str(), update.value.c_str());

            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter param_msg;
            param_msg.name = update.param;

            // Guess parameter type from string representation
            if (update.value == "true") {
                param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
                param_msg.value.bool_value = true;
            } else if (update.value == "false") {
                param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
                param_msg.value.bool_value = false;
            } else {
                try {
                    size_t idx;
                    int val = std::stoi(update.value, &idx);
                    if (idx == update.value.size()) {
                        param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
                        param_msg.value.integer_value = val;
                    } else {
                        throw std::invalid_argument("");
                    }
                } catch (...) {
                    try {
                        size_t idx;
                        double val = std::stod(update.value, &idx);
                        if (idx == update.value.size()) {
                            param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                            param_msg.value.double_value = val;
                        } else {
                            throw std::invalid_argument("");
                        }
                    } catch (...) {
                        param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
                        param_msg.value.string_value = update.value;
                    }
                }
            }

            request->parameters.push_back(param_msg);

            // Call parameter setting service asynchronously
            std::string node_name = update.node;
            std::string param_name = update.param;
            client->async_send_request(request, [this, node_name, param_name](
                rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->results.empty() || !response->results[0].successful) {
                        RCLCPP_WARN(this->get_logger(), "Failed to set parameter %s on node %s: %s",
                                    param_name.c_str(), node_name.c_str(), 
                                    response->results.empty() ? "No response result" : response->results[0].reason.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Parameter %s on node %s applied successfully.",
                                    param_name.c_str(), node_name.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while setting parameter: %s", e.what());
                }
            });

            // Remove applied parameter update from the queue
            it = pending_params_.erase(it);
        } else {
            // Keep in queue and check next
            ++it;
        }
    }
}

} // namespace mission_manager
