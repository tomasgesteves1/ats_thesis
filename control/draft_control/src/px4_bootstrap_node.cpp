#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

namespace draft_control {

enum class UavState {
    STANDBY,
    ARMING,
    TAKEOFF,
    WAIT_FOR_TAKEOFF,
    SWITCH_OFFBOARD,
    DONE
};

class Px4BootstrapNode : public rclcpp::Node {
public:
    Px4BootstrapNode() : Node("px4_bootstrap_node"), state_(UavState::STANDBY) {
        this->declare_parameter<double>("takeoff_height", 5.0);
        
        auto qos = rclcpp::SensorDataQoS();
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/px4_1/fmu/out/vehicle_status_v1", qos,
            std::bind(&Px4BootstrapNode::statusCallback, this, std::placeholders::_1)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&Px4BootstrapNode::odomCallback, this, std::placeholders::_1)
        );
        
        command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_1/fmu/in/vehicle_command", 10
        );
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&Px4BootstrapNode::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "PX4 Bootstrap Node initialized.");
    }

private:
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        latest_status_ = msg;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
    }
    
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param7 = param7;
        msg.target_system = 2; // Default for x500 in simulation
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
    }
    
    void controlLoop() {
        if (!latest_status_ || !latest_odom_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for vehicle status and odometry...");
            return;
        }
        
        double target_height = this->get_parameter("takeoff_height").as_double();
        
        switch (state_) {
            case UavState::STANDBY: {
                if (latest_status_->pre_flight_checks_pass) {
                    RCLCPP_INFO(this->get_logger(), "Transition: STANDBY -> ARMING. Pre-flight checks passed.");
                    state_ = UavState::ARMING;
                    counter_ = 0;
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                        "Pre-flight checks failing...");
                }
                break;
            }
            
            case UavState::ARMING: {
                if (latest_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                    RCLCPP_INFO(this->get_logger(), "Transition: ARMING -> TAKEOFF. Armed successfully.");
                    state_ = UavState::TAKEOFF;
                } else {
                    if (counter_ % 30 == 0) {
                        RCLCPP_INFO(this->get_logger(), "Sending ARM command...");
                        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                    }
                    counter_++;
                }
                break;
            }
            
            case UavState::TAKEOFF: {
                RCLCPP_INFO(this->get_logger(), "Sending Takeoff command to %.2f m...", target_height);
                publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0f, 0.0f, static_cast<float>(target_height));
                state_ = UavState::WAIT_FOR_TAKEOFF;
                stable_iterations_ = 0;
                break;
            }
            
            case UavState::WAIT_FOR_TAKEOFF: {
                double z = latest_odom_->pose.pose.position.z;
                double vz = latest_odom_->twist.twist.linear.z;
                double err = std::abs(z - target_height);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Takeoff -> Z: %.2f m (target: %.2f m, err: %.2f m), Vz: %.2f m/s", z, target_height, err, vz);
                
                if (err < 0.20 && std::abs(vz) < 0.10) {
                    stable_iterations_++;
                } else {
                    stable_iterations_ = 0;
                }
                
                if (stable_iterations_ >= 60) { // 3 seconds at 20Hz
                    RCLCPP_INFO(this->get_logger(), "Transition: WAIT_FOR_TAKEOFF -> SWITCH_OFFBOARD.");
                    state_ = UavState::SWITCH_OFFBOARD;
                    counter_ = 0;
                }
                break;
            }
            
            case UavState::SWITCH_OFFBOARD: {
                if (latest_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_INFO(this->get_logger(), "Bootstrap DONE! Vehicle in OFFBOARD mode.");
                    state_ = UavState::DONE;
                } else {
                    if (counter_ % 30 == 0) {
                        RCLCPP_INFO(this->get_logger(), "Sending OFFBOARD mode command...");
                        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
                    }
                    counter_++;
                }
                break;
            }
            
            case UavState::DONE: {
                RCLCPP_INFO(this->get_logger(), "Shutting down bootstrap node.");
                rclcpp::shutdown();
                break;
            }
        }
    }
    
    UavState state_;
    int counter_{0};
    int stable_iterations_{0};
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    px4_msgs::msg::VehicleStatus::SharedPtr latest_status_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

} // namespace draft_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<draft_control::Px4BootstrapNode>();
    rclcpp::spin(node);
    return 0;
}
