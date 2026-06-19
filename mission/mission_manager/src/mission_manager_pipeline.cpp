#include "mission_manager/mission_manager_pipeline.hpp"
#include <iostream>

namespace mission_manager {

MissionManagerPipeline::MissionManagerPipeline() {
    registry_ = std::make_unique<MissionManagerRegistry>();
    sim_executor_ = std::make_unique<MissionManagerExecutor>(
        std::bind(&MissionManagerPipeline::handleSimExecutorStateChange, this, std::placeholders::_1, std::placeholders::_2)
    );
    control_executor_ = std::make_unique<MissionManagerExecutor>(
        std::bind(&MissionManagerPipeline::handleControlExecutorStateChange, this, std::placeholders::_1, std::placeholders::_2)
    );
}

bool MissionManagerPipeline::initialize(const std::string& missions_dir) {
    return registry_->loadFromDirectory(missions_dir);
}

std::vector<MissionManifest> MissionManagerPipeline::listMissions() const {
    return registry_->getMissions();
}

bool MissionManagerPipeline::startSimulation(const std::string& mission_id,
                                             const std::map<std::string, std::string>& user_params,
                                             std::string& error_msg) {
    auto manifest_opt = registry_->getMission(mission_id);
    if (!manifest_opt) {
        error_msg = "Mission manifest not found for id: " + mission_id;
        return false;
    }
    auto manifest = manifest_opt.value();
    if (manifest.simulation_launch.package.empty() || manifest.simulation_launch.file.empty()) {
        error_msg = "Mission has no simulation launch configured.";
        return false;
    }

    // Resolve parameter overrides for simulation launch args
    std::map<std::string, std::string> launch_args = manifest.simulation_launch.args;
    for (const auto& param : manifest.user_params) {
        if (!param.ros_mapping.launch_arg.empty()) {
            auto it = user_params.find(param.key);
            if (it != user_params.end()) {
                launch_args[param.ros_mapping.launch_arg] = it->second;
            }
        }
    }

    return sim_executor_->start(manifest.id, manifest.simulation_launch.package, manifest.simulation_launch.file, launch_args, "simulation");
}

bool MissionManagerPipeline::stopSimulation(std::string& error_msg) {
    if (sim_executor_->getState() == ExecutorState::IDLE) {
        error_msg = "Simulation is not running.";
        return true;
    }
    return sim_executor_->stop();
}

bool MissionManagerPipeline::startMission(const std::string& mission_id,
                                          const std::map<std::string, std::string>& user_params,
                                          std::vector<ParameterUpdate>& pending_params,
                                          std::string& error_msg) {
    auto manifest_opt = registry_->getMission(mission_id);
    if (!manifest_opt) {
        error_msg = "Mission manifest not found for id: " + mission_id;
        return false;
    }

    auto manifest = manifest_opt.value();

    if (manifest.control_launch.package.empty() || manifest.control_launch.file.empty()) {
        error_msg = "Mission has no control launch configured.";
        return false;
    }

    // Prepare launch args and runtime parameters to set
    std::map<std::string, std::string> launch_args = manifest.control_launch.args;
    pending_params.clear();

    for (const auto& param : manifest.user_params) {
        std::string value = param.default_value;
        auto it = user_params.find(param.key);
        if (it != user_params.end()) {
            value = it->second;
        }

        // Validate range if float/int
        if (param.type == "float" || param.type == "int") {
            try {
                double val = std::stod(value);
                if (val < param.min_value || val > param.max_value) {
                    std::cerr << "[Pipeline] Parameter " << param.key 
                              << " out of bounds (" << val << " vs [" << param.min_value << ", " << param.max_value << "]). Using boundary." << std::endl;
                    if (val < param.min_value) value = std::to_string(param.min_value);
                    if (val > param.max_value) value = std::to_string(param.max_value);
                }
            } catch (...) {
                std::cerr << "[Pipeline] Could not parse parameter value: " << value << ". Using default." << std::endl;
                value = param.default_value;
            }
        }

        if (!param.ros_mapping.launch_arg.empty()) {
            launch_args[param.ros_mapping.launch_arg] = value;
        }

        if (!param.ros_mapping.node.empty() && !param.ros_mapping.param.empty()) {
            ParameterUpdate update;
            update.node = param.ros_mapping.node;
            update.param = param.ros_mapping.param;
            update.value = value;
            pending_params.push_back(update);
        }
    }

    return control_executor_->start(manifest.id, manifest.control_launch.package, manifest.control_launch.file, launch_args, "control");
}

bool MissionManagerPipeline::stopMission(std::string& error_msg) {
    if (control_executor_->getState() == ExecutorState::IDLE) {
        error_msg = "No mission is currently running.";
        return true;
    }
    return control_executor_->stop();
}

void MissionManagerPipeline::tick() {
    sim_executor_->checkHealth();
    control_executor_->checkHealth();
}

void MissionManagerPipeline::getStatus(std::string& control_state, std::string& sim_state, std::string& active_id, std::string& active_name) const {
    control_state = stateToString(control_executor_->getState());
    sim_state = stateToString(sim_executor_->getState());
    
    active_id = control_executor_->getActiveMissionId();
    if (active_id.empty()) {
        active_id = sim_executor_->getActiveMissionId();
    }
    
    if (!active_id.empty()) {
        auto manifest_opt = registry_->getMission(active_id);
        if (manifest_opt) {
            active_name = manifest_opt->name;
        } else {
            active_name = active_id;
        }
    } else {
        active_name = "None";
    }
}

std::string MissionManagerPipeline::stateToString(ExecutorState state) const {
    switch (state) {
        case ExecutorState::IDLE: return "IDLE";
        case ExecutorState::STARTING: return "STARTING";
        case ExecutorState::RUNNING: return "RUNNING";
        case ExecutorState::STOPPING: return "STOPPING";
        case ExecutorState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void MissionManagerPipeline::handleSimExecutorStateChange(ExecutorState state, const std::string& msg) {
    std::cout << "[Pipeline] Simulation Executor state changed to: " << stateToString(state);
    if (!msg.empty()) {
        std::cout << " (" << msg << ")";
    }
    std::cout << std::endl;
}

void MissionManagerPipeline::handleControlExecutorStateChange(ExecutorState state, const std::string& msg) {
    std::cout << "[Pipeline] Control Executor state changed to: " << stateToString(state);
    if (!msg.empty()) {
        std::cout << " (" << msg << ")";
    }
    std::cout << std::endl;
}

} // namespace mission_manager
