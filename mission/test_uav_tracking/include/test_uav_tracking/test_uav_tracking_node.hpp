#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include "test_uav_tracking/test_uav_tracking_pipeline.hpp"

namespace test_uav_tracking {

class TestUavTrackingNode : public rclcpp::Node {
public:
    TestUavTrackingNode();

private:
    void boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr latest_boat_odom_;
    std::unique_ptr<TestUavTrackingPipeline> pipeline_;
};

} // namespace test_uav_tracking
