#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <functional>

namespace uav_mpc {

enum class UavState {
    STANDBY,            // Waiting for valid odometry and pre-flight checks
    ARMING,             // Arming the drone
    TAKEOFF,            // Issuing default takeoff command
    WAIT_FOR_TAKEOFF,   // Monitoring altitude and vertical velocity stability
    SWITCH_OFFBOARD,    // Transitioning to Offboard Mode
    OFFBOARD_ACTIVE     // Actively sending MPC commands
};

class UavMpcStateMachine {
public:
    using CommandCallback = std::function<void(uint16_t, float, float, float)>;

    UavMpcStateMachine(rclcpp::Node* node, CommandCallback send_cmd_cb);

    void update(const px4_msgs::msg::VehicleStatus& status,
                const nav_msgs::msg::Odometry& odom,
                double hold_height,
                bool odom_valid);

    UavState getState() const { return state_; }
    const char* getStateName() const;

private:
    rclcpp::Node* node_;
    CommandCallback send_cmd_cb_;
    UavState state_;
    int arm_retry_counter_;
    int takeoff_retry_counter_;
    int offboard_retry_counter_;
    int stable_iterations_;
};

} // namespace uav_mpc
