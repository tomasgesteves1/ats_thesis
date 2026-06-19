#pragma once

#include <string>
#include <vector>
#include <map>
#include <optional>

namespace mission_manager {

struct RosMapping {
    std::string node;
    std::string param;
    std::string launch_arg;
};

struct UserParam {
    std::string key;
    std::string label;
    std::string type;
    std::string default_value;
    double min_value{0.0};
    double max_value{0.0};
    std::string unit;
    std::vector<std::string> options;
    RosMapping ros_mapping;
};

struct Requirements {
    std::string simulation;
    bool tether{false};
    std::vector<std::string> vehicles;
};

struct LaunchConfig {
    std::string package;
    std::string file;
    std::map<std::string, std::string> args;
};

struct MissionManifest {
    std::string id;
    std::string name;
    std::string description;
    std::string category;
    std::string icon;
    Requirements requirements;
    LaunchConfig simulation_launch;
    LaunchConfig control_launch;
    std::vector<UserParam> user_params;
};

class MissionManagerRegistry {
public:
    MissionManagerRegistry() = default;
    ~MissionManagerRegistry() = default;

    // Load all missions from a directory
    bool loadFromDirectory(const std::string& directory_path);

    // List all loaded missions as native C++ types
    std::vector<MissionManifest> getMissions() const;

    // Get a specific manifest
    std::optional<MissionManifest> getMission(const std::string& id) const;

private:
    std::map<std::string, MissionManifest> missions_;

    bool parseMissionFile(const std::string& filepath);
};

} // namespace mission_manager
