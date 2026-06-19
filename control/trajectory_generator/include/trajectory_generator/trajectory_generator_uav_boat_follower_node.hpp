#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include "trajectory_generator/trajectory_generator_pipeline.hpp"

namespace trajectory_generator {

class UavTrajectoryBoatFollowerNode : public rclcpp::Node {
public:
    UavTrajectoryBoatFollowerNode();

private:
    void boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr latest_boat_odom_;
    std::unique_ptr<TrajectoryPipeline> pipeline_;
};

} // namespace trajectory_generator
