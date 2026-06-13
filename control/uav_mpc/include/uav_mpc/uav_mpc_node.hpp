#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

#include "uav_mpc/uav_mpc_pipeline.hpp"

namespace uav_mpc {

class UavMpcNode : public rclcpp::Node {
public:
    UavMpcNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void tetherLengthCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void controlLoop();
    void publishOffboardControlMode();
    void publishAttitudeSetpoint(const UavControlOutput& output);
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publishVisualizationMarkers(const UavControlOutput& output);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tether_length_sub_;
    
    // TF Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // PX4 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Visualization Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predicted_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_trajectory_pub_;

    // MPC tether force publisher (relative topic)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mpc_tether_force_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UavMpcPipeline> pipeline_;
    bool target_initialized_;
    uint64_t offboard_setpoint_counter_;
};

} // namespace uav_mpc
