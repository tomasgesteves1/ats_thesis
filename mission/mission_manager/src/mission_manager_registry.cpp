#include "mission_manager/mission_manager_registry.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <filesystem>

namespace mission_manager {

namespace fs = std::filesystem;

bool MissionManagerRegistry::loadFromDirectory(const std::string& directory_path) {
    missions_.clear();
    if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
        std::cerr << "[Registry] Directory does not exist: " << directory_path << std::endl;
        return false;
    }

    bool success = true;
    for (const auto& entry : fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
            if (!parseMissionFile(entry.path().string())) {
                success = false;
            }
        }
    }
    return success;
}

bool MissionManagerRegistry::parseMissionFile(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);

        if (!config["id"] || !config["name"]) {
            std::cerr << "[Registry] Invalid manifest: missing id or name in " << filepath << std::endl;
            return false;
        }

        MissionManifest manifest;
        manifest.id = config["id"].as<std::string>();
        manifest.name = config["name"].as<std::string>();
        manifest.description = config["description"] ? config["description"].as<std::string>() : "";
        manifest.category = config["category"] ? config["category"].as<std::string>() : "";
        manifest.icon = config["icon"] ? config["icon"].as<std::string>() : "";

        // Requirements
        if (config["requires"]) {
            auto req = config["requires"];
            if (req["simulation"]) manifest.requirements.simulation = req["simulation"].as<std::string>();
            if (req["tether"]) manifest.requirements.tether = req["tether"].as<bool>();
            if (req["vehicles"]) {
                for (const auto& v : req["vehicles"]) {
                    manifest.requirements.vehicles.push_back(v.as<std::string>());
                }
            }
        }

        // Simulation Launch config
        if (config["simulation_launch"]) {
            auto l = config["simulation_launch"];
            manifest.simulation_launch.package = l["package"].as<std::string>();
            manifest.simulation_launch.file = l["file"].as<std::string>();
            if (l["args"]) {
                for (const auto& pair : l["args"]) {
                    manifest.simulation_launch.args[pair.first.as<std::string>()] = pair.second.as<std::string>();
                }
            }
        }

        // Control Launch config
        if (config["control_launch"]) {
            auto l = config["control_launch"];
            manifest.control_launch.package = l["package"].as<std::string>();
            manifest.control_launch.file = l["file"].as<std::string>();
            if (l["args"]) {
                for (const auto& pair : l["args"]) {
                    manifest.control_launch.args[pair.first.as<std::string>()] = pair.second.as<std::string>();
                }
            }
        }

        // User params
        if (config["user_params"]) {
            for (const auto& param_node : config["user_params"]) {
                UserParam param;
                param.key = param_node["key"].as<std::string>();
                param.label = param_node["label"] ? param_node["label"].as<std::string>() : param.key;
                param.type = param_node["type"] ? param_node["type"].as<std::string>() : "string";
                
                if (param_node["default"].IsDefined()) {
                    param.default_value = param_node["default"].as<std::string>();
                }
                
                if (param_node["min"]) param.min_value = param_node["min"].as<double>();
                if (param_node["max"]) param.max_value = param_node["max"].as<double>();
                if (param_node["unit"]) param.unit = param_node["unit"].as<std::string>();
                
                if (param_node["options"]) {
                    for (const auto& opt : param_node["options"]) {
                        param.options.push_back(opt.as<std::string>());
                    }
                }

                if (param_node["ros_mapping"]) {
                    auto rm = param_node["ros_mapping"];
                    if (rm["node"]) param.ros_mapping.node = rm["node"].as<std::string>();
                    if (rm["param"]) param.ros_mapping.param = rm["param"].as<std::string>();
                    if (rm["launch_arg"]) param.ros_mapping.launch_arg = rm["launch_arg"].as<std::string>();
                }
                manifest.user_params.push_back(param);
            }
        }

        missions_[manifest.id] = manifest;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Registry] Error parsing mission file: " << filepath << " - " << e.what() << std::endl;
        return false;
    }
}

std::vector<MissionManifest> MissionManagerRegistry::getMissions() const {
    std::vector<MissionManifest> list;
    for (const auto& [id, m] : missions_) {
        list.push_back(m);
    }
    return list;
}

std::optional<MissionManifest> MissionManagerRegistry::getMission(const std::string& id) const {
    auto it = missions_.find(id);
    if (it != missions_.end()) {
        return it->second;
    }
    return std::nullopt;
}

} // namespace mission_manager
