#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include "uav_trajectory/uav_trajectory_pipeline.hpp"

namespace uav_trajectory {

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
    std::unique_ptr<UavTrajectoryPipeline> pipeline_;
};

} // namespace uav_trajectory
