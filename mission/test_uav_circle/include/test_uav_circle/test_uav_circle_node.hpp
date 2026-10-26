#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <array>
#include <string>
#include <vector>
#include <memory>
#include "test_uav_circle/test_uav_circle_pipeline.hpp"

namespace test_uav_circle {

class TestUavCircleNode : public rclcpp::Node {
public:
    TestUavCircleNode();

private:
    void timerCallback();
    void publishPx4Setpoint(const std::vector<TrajectoryPoint>& points, double dt);
    std::vector<TrajectoryPoint> approachPoints(int steps, double dt, double dt_step);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUavCirclePipeline> pipeline_;

    double drone_altitude_{0.0};
    bool px4_offboard_{false};
    std::vector<std::array<double, 4>> waypoints_;  // [t, x, y, z]
    bool has_px4_odom_{false};
    std::array<double, 3> drone_gt_{0.0, 0.0, 0.0};   // ground truth (odometry origin), world ENU
    std::array<double, 3> drone_px4_{0.0, 0.0, 0.0};  // PX4 local position, converted to ENU
    std::array<double, 3> boat_gt_{0.0, 0.0, 0.0};
    bool has_drone_odom_{false};
    bool approach_init_{false};
    std::array<double, 3> approach_{0.0, 0.0, 0.0};  // reference that leads the UAV to the start of the table
    bool started_{false};
    double trajectory_time_{0.0};
    double last_time_sec_{0.0};
    double table_time_{0.0};
    double last_table_time_{0.0};
};

} // namespace test_uav_circle
