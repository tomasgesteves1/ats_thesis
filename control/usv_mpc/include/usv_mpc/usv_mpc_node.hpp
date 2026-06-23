#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>

#include "usv_mpc/usv_mpc_pipeline.hpp"

namespace usv_mpc {

class UsvMpcNode : public rclcpp::Node {
public:
    UsvMpcNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void trajectoryPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_horizon_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UsvMpcPipeline> pipeline_;
    bool target_initialized_;
};

} // namespace usv_mpc
