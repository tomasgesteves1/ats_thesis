#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

#include "usv_mpc/usv_mpc_pipeline.hpp"

namespace usv_mpc {

class UsvMpcNode : public rclcpp::Node {
public:
    UsvMpcNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UsvMpcPipeline> pipeline_;
};

} // namespace usv_mpc
