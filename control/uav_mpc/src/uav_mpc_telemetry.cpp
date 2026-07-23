#include "uav_mpc/uav_mpc_telemetry.hpp"
#include <cmath>

namespace uav_mpc {
namespace telemetry {

double calculateWinchTension(double L_tether, double base_tension, double linear_density) {
    const double g = 9.81;
    return base_tension + linear_density * L_tether * g;
}

double estimateTetherForce(double px, double py, double pz,
                           double anchor_x, double anchor_y, double anchor_z,
                           double T0_val, double eps) {
    double dx = anchor_x - px;
    double dy = anchor_y - py;
    double dz = anchor_z - pz;
    double s_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double s_norm_eps = std::sqrt(s_dist*s_dist + eps*eps);
    return T0_val * (s_dist / s_norm_eps);
}

visualization_msgs::msg::Marker createPredictedTrajectoryMarker(
    const std::vector<std::vector<double>>& predicted_positions, 
    const rclcpp::Time& stamp)
{
    visualization_msgs::msg::Marker line_msg;
    line_msg.header.frame_id = "world";
    line_msg.header.stamp = stamp;
    line_msg.ns = "predicted_trajectory";
    line_msg.id = 0;
    line_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_msg.action = visualization_msgs::msg::Marker::ADD;
    line_msg.pose.orientation.w = 1.0;
    
    line_msg.scale.x = 0.05; // Line width
    line_msg.color.r = 0.0;
    line_msg.color.g = 1.0;
    line_msg.color.b = 0.0;
    line_msg.color.a = 0.8;

    for (const auto& pos : predicted_positions) {
        geometry_msgs::msg::Point p;
        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        line_msg.points.push_back(p);
    }
    return line_msg;
}

visualization_msgs::msg::Marker createTargetPointMarker(
    const std::vector<double>& current_reference, 
    const rclcpp::Time& stamp)
{
    visualization_msgs::msg::Marker point_msg;
    point_msg.header.frame_id = "world";
    point_msg.header.stamp = stamp;
    point_msg.ns = "target_point";
    point_msg.id = 1;
    point_msg.type = visualization_msgs::msg::Marker::SPHERE;
    point_msg.action = visualization_msgs::msg::Marker::ADD;
    
    if (current_reference.size() == 3) {
        point_msg.pose.position.x = current_reference[0];
        point_msg.pose.position.y = current_reference[1];
        point_msg.pose.position.z = current_reference[2];
    }
    point_msg.pose.orientation.w = 1.0;

    point_msg.scale.x = 0.25; // 25 cm diameter
    point_msg.scale.y = 0.25;
    point_msg.scale.z = 0.25;

    point_msg.color.r = 1.0; // Red
    point_msg.color.g = 0.0;
    point_msg.color.b = 0.0;
    point_msg.color.a = 1.0;

    return point_msg;
}

visualization_msgs::msg::Marker createReferenceTrajectoryMarker(
    const std::vector<std::vector<double>>& reference_path, 
    const rclcpp::Time& stamp)
{
    visualization_msgs::msg::Marker ref_path_msg;
    ref_path_msg.header.frame_id = "world";
    ref_path_msg.header.stamp = stamp;
    ref_path_msg.ns = "reference_trajectory";
    ref_path_msg.id = 2;
    ref_path_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ref_path_msg.action = visualization_msgs::msg::Marker::ADD;
    ref_path_msg.pose.orientation.w = 1.0;

    ref_path_msg.scale.x = 0.03; // Line thickness
    ref_path_msg.color.r = 0.0;
    ref_path_msg.color.g = 0.5;
    ref_path_msg.color.b = 1.0;
    ref_path_msg.color.a = 0.9;

    for (const auto& pos : reference_path) {
        geometry_msgs::msg::Point p;
        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        ref_path_msg.points.push_back(p);
    }
    return ref_path_msg;
}

} // namespace telemetry
} // namespace uav_mpc
