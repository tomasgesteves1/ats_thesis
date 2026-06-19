#pragma once

#include <string>
#include <map>
#include <functional>
#include <sys/types.h>

namespace mission_manager {

enum class ExecutorState {
    IDLE,
    STARTING,
    RUNNING,
    STOPPING,
    ERROR
};

class MissionManagerExecutor {
public:
    using StateCallback = std::function<void(ExecutorState, const std::string&)>;

    explicit MissionManagerExecutor(StateCallback on_state_change);
    ~MissionManagerExecutor();

    // Start a launch file with parameters
    bool start(const std::string& mission_id,
               const std::string& package,
               const std::string& launch_file,
               const std::map<std::string, std::string>& launch_args,
               const std::string& log_filename = "");

    // Stop active mission
    bool stop(double timeout_seconds = 5.0);

    // Force kill everything
    void forceKill();

    // Health check (call periodically)
    void checkHealth();

    ExecutorState getState() const { return state_; }
    std::string getActiveMissionId() const { return active_mission_id_; }

private:
    StateCallback on_state_change_;
    ExecutorState state_{ExecutorState::IDLE};
    pid_t child_pid_{-1};
    std::string active_mission_id_;

    void transitionTo(ExecutorState new_state, const std::string& message = "");
};

} // namespace mission_manager
