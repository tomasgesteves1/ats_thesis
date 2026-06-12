#ifndef MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "moordyn_tether/moordyn_tether_pipeline.hpp"

namespace moordyn_tether
{

/// @brief ROS 2 wrapper node for the MoorDyn tether physics pipeline.
///
/// Responsibilities (ROS glue only, no math):
///  - TF lookups for anchor and body-center frames → positions in world frame.
///  - Odometry subscriptions → body-frame velocities and orientation quaternion.
///  - Assembles BodyState[2] and forwards it to MoordynTetherPipeline.
///  - Publishes resulting EntityWrench forces and cable geometry Marker.
class MoordynTetherNode : public rclcpp::Node
{
public:
    explicit MoordynTetherNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // --- Core loop ---
    void physicsLoop();
    void checkInitialization();

    // --- Odometry callbacks (velocity + orientation only, no position) ---
    void boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // --- TF lookup: fills positions and body_center_pos of both BodyStates.
    //     Returns false if any transform is unavailable. ---
    bool lookupBodyPositions();

    // --- Publishing helpers ---
    void publishWrench(const std::string & entity_name,
                       double fx, double fy, double fz);
    void publishGeometry(
        const std::vector<std::vector<double>> & nodes);
    void publishForceMarkers(const std::vector<double> & out_forces);
    void publishForceMagnitudes(const std::vector<double> & out_forces);

    // --- Pipeline log bridge ---
    void pipelineLogBridge(LogLevel level, const std::string & msg);

    // --- Parameters (loaded from YAML) ---
    std::string boat_anchor_frame_;   // TF frame of tether attachment on boat
    std::string boat_base_frame_;     // TF frame of boat body center (base_link)
    std::string drone_hook_frame_;    // TF frame of tether attachment on drone
    std::string drone_base_frame_;    // TF frame of drone body center (base_link)
    double      tf_timeout_s_;
    std::string boat_link_name_;      // Gazebo entity link name for wrench
    std::string drone_link_name_;     // Gazebo entity link name for wrench
    double      physics_rate_hz_;
    double      marker_line_width_;
    std::vector<double> marker_color_;

    // --- Force marker parameters ---
    std::string force_marker_topic_;
    double force_marker_max_force_;
    double force_marker_max_length_;
    std::vector<double> force_marker_color_;

    // --- Force magnitude parameters ---
    std::string force_mag_boat_topic_;
    std::string force_mag_drone_topic_;

    // --- Virtual winch parameters ---
    bool   winch_enabled_;
    std::string winch_mode_;
    double winch_slack_factor_;
    double winch_target_tension_;
    double winch_kp_tension_;
    double winch_kd_tension_;
    double winch_speed_limit_;
    double winch_min_length_;
    double winch_max_length_;

    // --- Filter parameters ---
    double force_filter_alpha_;

    // --- Aggregated kinematic state: [0]=boat, [1]=drone ---
    // Positions are filled by lookupBodyPositions() via TF.
    // Velocities and quaternion are filled by odom callbacks.
    std::array<BodyState, 2> body_states_{};

    // --- Odometry readiness flags ---
    bool boat_odom_received_{false};
    bool drone_odom_received_{false};

    // --- MoorDyn pipeline ---
    std::unique_ptr<MoordynTetherPipeline> pipeline_;
    bool initialized_{false};
    rclcpp::Time last_init_attempt_{0, 0, RCL_ROS_TIME};

    // --- TF ---
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- ROS interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      geometry_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr force_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               force_mag_boat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               force_mag_drone_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               tether_length_pub_;
    rclcpp::TimerBase::SharedPtr                                       physics_timer_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_
