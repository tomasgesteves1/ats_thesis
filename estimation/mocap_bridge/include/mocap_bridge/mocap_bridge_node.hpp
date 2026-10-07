#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <memory>
#include <string>
#include "mocap_bridge/mocap_bridge_pipeline.hpp"

namespace mocap_bridge {

class MocapBridgeNode : public rclcpp::Node {
public:
    MocapBridgeNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publishOriginCommand();

    std::unique_ptr<MocapBridgePipeline> pipeline_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;

    // Core PX4 Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vo_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_cmd_pub_;

    // Debug / Visualization Publishers (ENU)
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odom_enu_pub_;

    // Ground Truth decomposed topics (Vector3Stamped)
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gt_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gt_euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gt_lin_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gt_ang_vel_pub_;

    // PX4 Estimated decomposed topics (Vector3Stamped)
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_lin_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_ang_vel_pub_;

    // Error topics (Vector3Stamped)
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_ang_vel_pub_;

    rclcpp::TimerBase::SharedPtr origin_timer_;

    // State cache for online error computation
    kinematics::EnuOdometry last_gt_enu_{};
    bool has_gt_{false};

    // Parameters
    bool publish_debug_topics_{true};
    bool euler_in_degrees_{false};
    bool set_global_origin_{true};
    double origin_lat_{42.35821841111111};
    double origin_lon_{-71.0479235555555};
    double origin_alt_{0.0};
    int target_system_{2};
    int origin_count_{0};
};

} // namespace mocap_bridge
