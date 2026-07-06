#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include "test_usv_circle/test_usv_circle_pipeline.hpp"

namespace test_usv_circle {

class TestUsvCircleNode : public rclcpp::Node {
public:
    TestUsvCircleNode();

private:
    void timerCallback();
    void droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double get_double_param(const std::string& name);
    int get_int_param(const std::string& name);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUsvCirclePipeline> pipeline_;

    // Drone flight tracking for coordinated trajectory start
    double drone_altitude_{0.0};
    bool drone_started_flying_{false};
    double trajectory_time_{0.0};
    double last_time_sec_{0.0};

    // Boat state tracking for holding position
    double boat_x_{0.0};
    double boat_y_{0.0};
    double boat_yaw_{0.0};
};

} // namespace test_usv_circle
