#ifndef MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "moordyn_tether/tether_moordyn_pipeline.hpp"

namespace moordyn_tether
{

/// @brief ROS 2 wrapper node for the MoorDyn tether physics pipeline.
///
/// Subscribes to boat/drone odometry, feeds positions into MoorDyn,
/// and publishes resulting forces (EntityWrench) and cable geometry (Marker).
class MoordynTetherNode : public rclcpp::Node
{
public:
    explicit MoordynTetherNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // --- Callbacks ---
    void boatCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void droneCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void physicsLoop();
    void checkInitialization();

    // --- Publishing helpers ---
    void publishWrench(const std::string & entity_name,
                       double fx, double fy, double fz);
    void publishGeometry(
        const std::vector<std::vector<double>> & nodes);

    // --- Pipeline log bridge ---
    void pipelineLogBridge(LogLevel level, const std::string & msg);

    // --- Parameters (loaded from YAML) ---
    double boat_anchor_offset_z_;
    double drone_hook_offset_z_;
    std::string boat_link_name_;
    std::string drone_link_name_;
    double physics_rate_hz_;
    double marker_line_width_;
    std::vector<double> marker_color_;

    // --- State ---
    std::unique_ptr<TetherMoorDynPipeline> pipeline_;
    std::vector<double> current_positions_;
    bool initialized_{false};
    rclcpp::Time last_init_attempt_{0, 0, RCL_ROS_TIME};

    // --- ROS interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr geometry_pub_;
    rclcpp::TimerBase::SharedPtr physics_timer_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_NODE_HPP_
