#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <cmath>

namespace test_uav_circle {

enum class UavState {
    STANDBY,
    ARMING,
    SWITCH_OFFBOARD,
    DONE
};

class TestUavCircleBootstrapNode : public rclcpp::Node {
public:
    TestUavCircleBootstrapNode() : Node("test_uav_circle_bootstrap_node"), state_(UavState::STANDBY) {
        auto qos = rclcpp::SensorDataQoS();
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/px4_1/fmu/out/vehicle_status_v1", qos,
            std::bind(&TestUavCircleBootstrapNode::statusCallback, this, std::placeholders::_1)
        );
        
        command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_1/fmu/in/vehicle_command", 10
        );
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&TestUavCircleBootstrapNode::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Test UAV Circle Bootstrap Node (MPC Takeoff) initialized.");
    }

private:
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        latest_status_ = msg;
    }
    
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param5 = std::nanf(""); // NAN to use current home / GPS position
        msg.param6 = std::nanf(""); // NAN to use current home / GPS position
        msg.target_system = 2; // Default for x500 in simulation
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
    }
    
    void controlLoop() {
        if (!latest_status_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for vehicle status...");
            return;
        }
        
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
                    RCLCPP_INFO(this->get_logger(), "Transition: ARMING -> SWITCH_OFFBOARD. Armed successfully.");
                    state_ = UavState::SWITCH_OFFBOARD;
                    counter_ = 0;
                } else {
                    if (counter_ % 30 == 0) {
                        RCLCPP_INFO(this->get_logger(), "Sending ARM command...");
                        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                    }
                    counter_++;
                }
                break;
            }
            
            case UavState::SWITCH_OFFBOARD: {
                if (latest_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_INFO(this->get_logger(), "Bootstrap DONE! Vehicle in OFFBOARD mode (MPC takeoff active).");
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
                RCLCPP_INFO(this->get_logger(), "Shutting down circle bootstrap node.");
                rclcpp::shutdown();
                break;
            }
        }
    }
    
    UavState state_;
    int counter_{0};
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    px4_msgs::msg::VehicleStatus::SharedPtr latest_status_;
};

} // namespace test_uav_circle

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_uav_circle::TestUavCircleBootstrapNode>();
    rclcpp::spin(node);
    return 0;
}
