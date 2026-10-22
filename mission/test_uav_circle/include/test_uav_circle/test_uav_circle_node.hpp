#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include "test_uav_circle/test_uav_circle_pipeline.hpp"

namespace test_uav_circle {

class TestUavCircleNode : public rclcpp::Node {
public:
    TestUavCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUavCirclePipeline> pipeline_;

    double drone_altitude_{0.0};
    bool started_{false};
    double trajectory_time_{0.0};
    double last_time_sec_{0.0};
};

} // namespace test_uav_circle
