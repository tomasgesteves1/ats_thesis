#include "mission_manager/mission_manager_executor.hpp"
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <filesystem>

namespace mission_manager {

MissionManagerExecutor::MissionManagerExecutor(StateCallback on_state_change)
    : on_state_change_(on_state_change) {}

MissionManagerExecutor::~MissionManagerExecutor() {
    forceKill();
}

bool MissionManagerExecutor::start(const std::string& mission_id,
                                   const std::string& package,
                                   const std::string& launch_file,
                                   const std::map<std::string, std::string>& launch_args,
                                   const std::string& log_filename) {
    if (state_ != ExecutorState::IDLE && state_ != ExecutorState::ERROR) {
        std::cerr << "Cannot start mission. Executor is not IDLE." << std::endl;
        return false;
    }

    transitionTo(ExecutorState::STARTING, "Launching mission: " + mission_id);

    // Build arguments
    std::vector<std::string> args_vec;
    args_vec.push_back("ros2");
    args_vec.push_back("launch");
    args_vec.push_back(package);
    args_vec.push_back(launch_file);
    for (const auto& [k, v] : launch_args) {
        args_vec.push_back(k + ":=" + v);
    }

    std::vector<char*> argv;
    for (const auto& arg : args_vec) {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "Fork failed!" << std::endl;
        transitionTo(ExecutorState::ERROR, "Fork failed");
        return false;
    } else if (pid == 0) {
        // In child: Put child into its own process group
        setsid();

        // Redirect output to log file or /dev/null
        int fd = -1;
        if (!log_filename.empty()) {
            std::filesystem::create_directories("log");
            std::string log_path = "log/" + log_filename + ".log";
            fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        }

        if (fd < 0) {
            fd = open("/dev/null", O_WRONLY);
        }

        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        }

        execvp("ros2", argv.data());
        // If execvp returns, it failed
        exit(1);
    } else {
        // In parent
        child_pid_ = pid;
        active_mission_id_ = mission_id;
        transitionTo(ExecutorState::RUNNING, "Mission " + mission_id + " launched with PID " + std::to_string(pid));
        return true;
    }
}

bool MissionManagerExecutor::stop(double timeout_seconds) {
    if (child_pid_ <= 0) {
        transitionTo(ExecutorState::IDLE);
        return true;
    }

    transitionTo(ExecutorState::STOPPING, "Requesting graceful shutdown...");
    
    // Kill the entire process group
    kill(-child_pid_, SIGTERM);

    double elapsed = 0.0;
    double dt = 0.1;
    int status;
    while (elapsed < timeout_seconds) {
        pid_t res = waitpid(child_pid_, &status, WNOHANG);
        if (res == child_pid_) {
            child_pid_ = -1;
            active_mission_id_ = "";
            transitionTo(ExecutorState::IDLE, "Mission stopped gracefully.");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        elapsed += dt;
    }

    // Force kill if graceful stop timed out
    std::cerr << "Graceful shutdown timed out. Sending SIGKILL." << std::endl;
    kill(-child_pid_, SIGKILL);
    
    // Clean up zombie
    waitpid(child_pid_, &status, 0);
    child_pid_ = -1;
    active_mission_id_ = "";
    transitionTo(ExecutorState::IDLE, "Mission terminated by force.");
    return true;
}

void MissionManagerExecutor::forceKill() {
    if (child_pid_ > 0) {
        kill(-child_pid_, SIGKILL);
        int status;
        waitpid(child_pid_, &status, 0);
        child_pid_ = -1;
        active_mission_id_ = "";
        transitionTo(ExecutorState::IDLE, "Forced cleanup done.");
    }
}

void MissionManagerExecutor::checkHealth() {
    if (child_pid_ <= 0) {
        return;
    }

    int status;
    pid_t res = waitpid(child_pid_, &status, WNOHANG);
    if (res == child_pid_) {
        // Child exited!
        child_pid_ = -1;
        active_mission_id_ = "";
        
        if (WIFEXITED(status)) {
            int exit_code = WEXITSTATUS(status);
            if (exit_code == 0) {
                transitionTo(ExecutorState::IDLE, "Mission finished successfully.");
            } else {
                transitionTo(ExecutorState::ERROR, "Mission exited with code " + std::to_string(exit_code));
            }
        } else if (WIFSIGNALED(status)) {
            int sig = WTERMSIG(status);
            transitionTo(ExecutorState::ERROR, "Mission killed by signal " + std::to_string(sig));
        } else {
            transitionTo(ExecutorState::ERROR, "Mission terminated unexpectedly.");
        }
    }
}

void MissionManagerExecutor::transitionTo(ExecutorState new_state, const std::string& message) {
    state_ = new_state;
    if (on_state_change_) {
        on_state_change_(state_, message);
    }
}

} // namespace mission_manager
