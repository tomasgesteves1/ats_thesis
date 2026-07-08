#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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
    void trajectoryPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void tetherLengthCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void boatHorizonCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    void publishOffboardControlMode();
    void publishAttitudeSetpoint(const UavControlOutput& output);
    void publishVisualizationMarkers(const UavControlOutput& output);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tether_length_sub_;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr boat_horizon_sub_;
    
    // TF Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // PX4 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;

    // Visualization Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predicted_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_trajectory_pub_;

    // MPC tether force publisher (relative topic)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mpc_tether_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr virtual_tether_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr virtual_tether_limit_pub_;

    // MPC debug states and inputs publisher (relative topic)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpc_states_inputs_pub_;

    // Path publishers (relative topics)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predicted_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_reference_path_pub_;

    // Open-loop test publishers (relative topics)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr open_loop_predicted_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr open_loop_actual_path_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UavMpcPipeline> pipeline_;

    px4_msgs::msg::VehicleStatus latest_vehicle_status_;
    nav_msgs::msg::Odometry latest_odom_;

    bool target_initialized_;
    bool odom_received_;
    bool vehicle_status_received_;
    uint64_t offboard_setpoint_counter_;
    double px4_hover_thrust_;
    uint8_t current_system_id_;
    rclcpp::Time last_boat_horizon_time_;
    double active_tether_max_length_;

    // Open-loop variables
    bool open_loop_test_;
    bool open_loop_active_;
    size_t open_loop_step_;
    std::vector<UavControlOutput> open_loop_steps_;
    nav_msgs::msg::Path open_loop_predicted_path_;
    nav_msgs::msg::Path open_loop_actual_path_;
};

} // namespace uav_mpc
