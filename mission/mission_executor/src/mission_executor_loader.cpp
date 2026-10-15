#include "mission_executor/mission_executor_loader.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace mission_executor
{

namespace
{
void appendNumbers(const YAML::Node & node, std::vector<double> & out)
{
    if (node.IsScalar()) {
        out.push_back(node.as<double>());
    } else if (node.IsSequence()) {
        for (const auto & item : node) {
            appendNumbers(item, out);
        }
    } else {
        throw std::runtime_error("parameters must be numbers or lists of numbers");
    }
}

std::vector<StepSpec> loadSequence(const YAML::Node & node, const std::string & vehicle)
{
    std::vector<StepSpec> steps;
    if (!node) {
        return steps;
    }
    if (!node.IsSequence()) {
        throw std::runtime_error("'" + vehicle + "' must be a list of maneuvers");
    }
    for (const auto & item : node) {
        if (!item.IsMap() || !item["type"]) {
            throw std::runtime_error("every maneuver of '" + vehicle + "' needs a 'type'");
        }
        StepSpec step;
        step.type = item["type"].as<std::string>();
        step.id = item["id"] ? item["id"].as<std::string>() : step.type;
        step.after = item["after"] ? item["after"].as<std::string>() : "";
        step.until = item["until"] ? item["until"].as<std::string>() : "";
        for (const auto & kv : item) {
            const auto key = kv.first.as<std::string>();
            if (key == "type" || key == "id" || key == "after" || key == "until") {
                continue;
            }
            std::vector<double> values;
            try {
                appendNumbers(kv.second, values);
            } catch (const std::exception & e) {
                throw std::runtime_error(vehicle + "." + step.id + "." + key + ": " + e.what());
            }
            step.params[key] = values;
        }
        steps.push_back(step);
    }
    return steps;
}
}  // namespace

MissionSpec loadMission(const std::string & path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception & e) {
        throw std::runtime_error("cannot read mission file " + path + ": " + e.what());
    }
    MissionSpec mission;
    mission.name = root["name"] ? root["name"].as<std::string>() : path;
    mission.uav = loadSequence(root["uav"], "uav");
    mission.usv = loadSequence(root["usv"], "usv");
    mission.uav_abort = loadSequence(root["uav_abort"], "uav_abort");
    if (mission.uav.empty() && mission.usv.empty()) {
        throw std::runtime_error("mission " + path + " has no maneuvers");
    }
    return mission;
}

}  // namespace mission_executor
