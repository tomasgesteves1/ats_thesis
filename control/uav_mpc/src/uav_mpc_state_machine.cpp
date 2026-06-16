#include "uav_mpc/uav_mpc_state_machine.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <cmath>

namespace uav_mpc {

UavMpcStateMachine::UavMpcStateMachine(rclcpp::Node* node, CommandCallback send_cmd_cb)
    : node_(node),
      send_cmd_cb_(send_cmd_cb),
      state_(UavState::STANDBY),
      arm_retry_counter_(0),
      takeoff_retry_counter_(0),
      offboard_retry_counter_(0),
      stable_iterations_(0) {}

const char* UavMpcStateMachine::getStateName() const {
    switch (state_) {
        case UavState::STANDBY:          return "STANDBY";
        case UavState::ARMING:           return "ARMING";
        case UavState::TAKEOFF:          return "TAKEOFF";
        case UavState::WAIT_FOR_TAKEOFF: return "WAIT_FOR_TAKEOFF";
        case UavState::SWITCH_OFFBOARD:  return "SWITCH_OFFBOARD";
        case UavState::OFFBOARD_ACTIVE:  return "OFFBOARD_ACTIVE";
        default:                         return "UNKNOWN";
    }
}

void UavMpcStateMachine::update(const px4_msgs::msg::VehicleStatus& status,
                                const nav_msgs::msg::Odometry& odom,
                                double hold_height,
                                bool odom_valid) {
    switch (state_) {
        case UavState::STANDBY: {
            if (odom_valid && status.pre_flight_checks_pass) {
                RCLCPP_INFO(node_->get_logger(), "State transition: STANDBY -> ARMING. Pre-flight checks passed.");
                state_ = UavState::ARMING;
                arm_retry_counter_ = 0;
            } else {
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "Waiting in STANDBY. Odom valid: %s, Pre-flight checks: %s",
                    odom_valid ? "YES" : "NO", status.pre_flight_checks_pass ? "PASSED" : "FAILED");
            }
            break;
        }

        case UavState::ARMING: {
            if (status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(node_->get_logger(), "State transition: ARMING -> TAKEOFF. Vehicle ARMED.");
                state_ = UavState::TAKEOFF;
                takeoff_retry_counter_ = 0;
            } else {
                // Send ARM command every 1.5 seconds (30 cycles at 20Hz)
                if (arm_retry_counter_ % 30 == 0) {
                    RCLCPP_INFO(node_->get_logger(), "Sending arm command...");
                    send_cmd_cb_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f, 0.0f);
                }
                arm_retry_counter_++;
            }
            break;
        }

        case UavState::TAKEOFF: {
            RCLCPP_INFO(node_->get_logger(), "Sending takeoff command to %.2f m...", hold_height);
            send_cmd_cb_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0f, 0.0f, static_cast<float>(hold_height));
            
            RCLCPP_INFO(node_->get_logger(), "State transition: TAKEOFF -> WAIT_FOR_TAKEOFF.");
            state_ = UavState::WAIT_FOR_TAKEOFF;
            stable_iterations_ = 0;
            break;
        }

        case UavState::WAIT_FOR_TAKEOFF: {
            double current_z = odom.pose.pose.position.z;
            double current_vz = odom.twist.twist.linear.z;
            double height_error = std::abs(current_z - hold_height);

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "WAIT_FOR_TAKEOFF -> Z: %.2f m (target: %.2f m, error: %.2f m), Vz: %.2f m/s",
                current_z, hold_height, height_error, current_vz);

            // Wait until drone is at height and vertical velocity is near zero
            if (height_error < 0.20 && std::abs(current_vz) < 0.10) {
                stable_iterations_++;
            } else {
                stable_iterations_ = 0;
            }

            // Must remain stable for 2 seconds (40 iterations at 20Hz)
            if (stable_iterations_ >= 40) {
                RCLCPP_INFO(node_->get_logger(), "State transition: WAIT_FOR_TAKEOFF -> SWITCH_OFFBOARD. Drone stable at hover.");
                state_ = UavState::SWITCH_OFFBOARD;
                offboard_retry_counter_ = 0;
            }
            break;
        }

        case UavState::SWITCH_OFFBOARD: {
            if (status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                RCLCPP_INFO(node_->get_logger(), "State transition: SWITCH_OFFBOARD -> OFFBOARD_ACTIVE. Control switched to Offboard.");
                state_ = UavState::OFFBOARD_ACTIVE;
            } else {
                // Send Offboard mode command every 1.5 seconds (30 cycles at 20Hz)
                if (offboard_retry_counter_ % 30 == 0) {
                    RCLCPP_INFO(node_->get_logger(), "Sending Offboard mode command...");
                    send_cmd_cb_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f, 0.0f);
                }
                offboard_retry_counter_++;
            }
            break;
        }

        case UavState::OFFBOARD_ACTIVE: {
            // Control is actively routed to MPC, nothing to do here
            break;
        }
    }
}

} // namespace uav_mpc
