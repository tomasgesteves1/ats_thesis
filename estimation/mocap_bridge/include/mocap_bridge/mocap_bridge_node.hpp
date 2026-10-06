#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <memory>
#include "mocap_bridge/mocap_bridge_pipeline.hpp"

namespace mocap_bridge {

class MocapBridgeNode : public rclcpp::Node {
public:
    MocapBridgeNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishOriginCommand();

    std::unique_ptr<MocapBridgePipeline> pipeline_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vo_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_cmd_pub_;
    rclcpp::TimerBase::SharedPtr origin_timer_;

    bool set_global_origin_{true};
    double origin_lat_{42.35821841111111};
    double origin_lon_{-71.0479235555555};
    double origin_alt_{0.0};
    int target_system_{2};
    int origin_count_{0};
};

} // namespace mocap_bridge
