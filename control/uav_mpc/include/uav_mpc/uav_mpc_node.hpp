#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <memory>

#include "uav_mpc/uav_mpc_pipeline.hpp"

namespace uav_mpc {

class UavMpcNode : public rclcpp::Node {
public:
    UavMpcNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void controlLoop();
    void publishOffboardControlMode();
    void publishAttitudeSetpoint(const std::vector<double>& control_cmd);
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    
    // PX4 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UavMpcPipeline> pipeline_;
    bool target_initialized_;
    double target_yaw_;
    uint64_t offboard_setpoint_counter_;

    // Último estado de atitude recebido da odometria
    double qx_, qy_, qz_, qw_;

    // Funções auxiliares matemáticas de atitude
    void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw);
    void rotationMatrixToQuaternion(double R[3][3], float q[4]);
};

} // namespace uav_mpc
