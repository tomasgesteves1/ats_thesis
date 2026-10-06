#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include "test_uav_line/test_uav_line_pipeline.hpp"

namespace test_uav_line {

class TestUavLineNode : public rclcpp::Node {
public:
    TestUavLineNode();

private:
    void timerCallback();
    void publishPx4Setpoint(const std::vector<TrajectoryPoint>& points, double dt);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::unique_ptr<TestUavLinePipeline> pipeline_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_setpoint_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double start_time_{-1.0};
    double drone_z_{0.0};
    bool px4_offboard_{true};      ///< Feed the trajectory directly to the PX4 position controller
    double px4_origin_[3]{0.3, 0.0, 1.4};  ///< World position of the PX4 local-frame origin (ENU) [m]
    double start_altitude_{0.0};  ///< Altitude the UAV must reach before the clock starts [m]
    bool mission_finished_logged_{false};
};

} // namespace test_uav_line
