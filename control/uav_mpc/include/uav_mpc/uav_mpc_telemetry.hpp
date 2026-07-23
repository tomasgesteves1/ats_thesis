#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/time.hpp>
#include <vector>

namespace uav_mpc {
namespace telemetry {

/**
 * Calculate the winch tension (T0) based on tether length and density.
 */
double calculateWinchTension(double L_tether, double base_tension = 0.1, double linear_density = 0.020);

/**
 * Estimate the tether force magnitude assumed by the MPC model.
 */
double estimateTetherForce(double px, double py, double pz,
                           double anchor_x, double anchor_y, double anchor_z,
                           double T0_val, double eps = 0.02);

/**
 * Create a marker to visualize the predicted MPC trajectory.
 */
visualization_msgs::msg::Marker createPredictedTrajectoryMarker(
    const std::vector<std::vector<double>>& predicted_positions, 
    const rclcpp::Time& stamp);

/**
 * Create a marker to visualize the current target/reference point.
 */
visualization_msgs::msg::Marker createTargetPointMarker(
    const std::vector<double>& current_reference, 
    const rclcpp::Time& stamp);

/**
 * Create a marker to visualize the reference path.
 */
visualization_msgs::msg::Marker createReferenceTrajectoryMarker(
    const std::vector<std::vector<double>>& reference_path, 
    const rclcpp::Time& stamp);

} // namespace telemetry
} // namespace uav_mpc
