#include "moordyn_tether/moordyn_tether_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>
#include <cmath>
#include <functional>

namespace moordyn_tether
{

MoordynTetherNode::MoordynTetherNode(const rclcpp::NodeOptions & options)
    : Node("moordyn_tether_node", options)
{
    // ---- Load parameters from YAML ----
    boat_anchor_offset_z_ = this->declare_parameter("boat_anchor_offset_z", 1.3);
    drone_hook_offset_z_  = this->declare_parameter("drone_hook_offset_z", 0.26);
    boat_link_name_ = this->declare_parameter("boat_link_name",
                          std::string("wamv::wamv/base_link"));
    drone_link_name_ = this->declare_parameter("drone_link_name",
                           std::string("x500::base_link"));
    physics_rate_hz_ = this->declare_parameter("physics_rate_hz", 50.0);
    marker_line_width_ = this->declare_parameter("marker_line_width", 0.01);
    marker_color_ = this->declare_parameter("marker_color",
                        std::vector<double>{0.2, 0.2, 0.2, 1.0});

    auto boat_odom_topic  = this->declare_parameter("boat_odom_topic",
                                std::string("boat/ground_truth/odometry"));
    auto drone_odom_topic = this->declare_parameter("drone_odom_topic",
                                std::string("drone/ground_truth/odometry"));
    auto wrench_topic     = this->declare_parameter("wrench_topic",
                                std::string("/world/wamv_world/wrench"));
    auto marker_topic     = this->declare_parameter("marker_topic",
                                std::string("tether_geometry_marker"));

    // ---- Create pipeline (pure C++ core) ----
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("moordyn_tether");
    std::string config_path = pkg_share + "/config/lines.txt";

    pipeline_ = std::make_unique<TetherMoorDynPipeline>(
        config_path,
        [this](LogLevel level, const std::string & msg) {
            pipelineLogBridge(level, msg);
        });

    // ---- State ----
    // [0,1,2] -> Boat (x,y,z), [3,4,5] -> Drone (x,y,z)
    current_positions_.assign(6, 0.0);

    // ---- Subscriptions ----
    using std::placeholders::_1;
    boat_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        boat_odom_topic, 10,
        std::bind(&MoordynTetherNode::boatCallback, this, _1));

    drone_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        drone_odom_topic, 10,
        std::bind(&MoordynTetherNode::droneCallback, this, _1));

    // ---- Publishers ----
    wrench_pub_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>(
        wrench_topic, 10);
    geometry_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        marker_topic, 10);

    // ---- Physics timer ----
    auto period_ms = static_cast<int>(1000.0 / physics_rate_hz_);
    physics_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MoordynTetherNode::physicsLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "MoordynTetherNode initialized. Physics rate: %.1f Hz",
                physics_rate_hz_);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void MoordynTetherNode::boatCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_positions_[0] = msg->pose.pose.position.x;
    current_positions_[1] = msg->pose.pose.position.y;
    current_positions_[2] = msg->pose.pose.position.z + boat_anchor_offset_z_;
    checkInitialization();
}

void MoordynTetherNode::droneCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_positions_[3] = msg->pose.pose.position.x;
    current_positions_[4] = msg->pose.pose.position.y;
    current_positions_[5] = msg->pose.pose.position.z + drone_hook_offset_z_;
    checkInitialization();
}

void MoordynTetherNode::checkInitialization()
{
    if (initialized_) {
        return;
    }

    // Throttle initialization attempts to once per second
    auto now = this->get_clock()->now();
    if ((now - last_init_attempt_).seconds() < 1.0) {
        return;
    }
    last_init_attempt_ = now;

    constexpr double kPositionThreshold = 0.001;
    constexpr double kAltitudeThreshold = 0.1;

    bool boat_ready =
        std::abs(current_positions_[0]) > kPositionThreshold ||
        std::abs(current_positions_[1]) > kPositionThreshold ||
        current_positions_[2] > kAltitudeThreshold;

    bool drone_ready =
        std::abs(current_positions_[3]) > kPositionThreshold ||
        std::abs(current_positions_[4]) > kPositionThreshold ||
        current_positions_[5] > kAltitudeThreshold;

    if (boat_ready && drone_ready)
    {
        if (pipeline_->initialize(current_positions_))
        {
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "MoorDyn initialized successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "MoorDyn_Init FAILED. Check that the number of "
                         "Coupled points in lines.txt matches (expected 2).");
        }
    }
}

void MoordynTetherNode::physicsLoop()
{
    if (!initialized_) {
        return;
    }

    std::vector<double> out_forces;
    std::vector<std::vector<double>> cable_nodes;
    double dt = 1.0 / physics_rate_hz_;

    if (pipeline_->step(current_positions_, dt, out_forces, cable_nodes))
    {
        // Apply forces on boat and drone via Gazebo wrench bridge
        publishWrench(boat_link_name_,
                      out_forces[0], out_forces[1], out_forces[2]);
        publishWrench(drone_link_name_,
                      out_forces[3], out_forces[4], out_forces[5]);

        // Publish cable geometry for RViz visualization
        publishGeometry(cable_nodes);
    }
}

// ---------------------------------------------------------------------------
// Publishing helpers
// ---------------------------------------------------------------------------

void MoordynTetherNode::publishWrench(const std::string & entity_name,
                                      double fx, double fy, double fz)
{
    ros_gz_interfaces::msg::EntityWrench msg;
    msg.entity.name = entity_name;
    msg.entity.type = 3;  // LINK
    msg.wrench.force.x = fx;
    msg.wrench.force.y = fy;
    msg.wrench.force.z = fz;
    wrench_pub_->publish(msg);
}

void MoordynTetherNode::publishGeometry(
    const std::vector<std::vector<double>> & nodes)
{
    if (nodes.empty()) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "tether_catenary";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = marker_line_width_;

    // Apply RGBA from parameter (defaults to dark grey)
    marker.color.r = static_cast<float>(marker_color_[0]);
    marker.color.g = static_cast<float>(marker_color_[1]);
    marker.color.b = static_cast<float>(marker_color_[2]);
    marker.color.a = static_cast<float>(marker_color_[3]);

    for (const auto & node_pos : nodes)
    {
        geometry_msgs::msg::Point p;
        p.x = node_pos[0];
        p.y = node_pos[1];
        p.z = node_pos[2];
        marker.points.push_back(p);
    }

    geometry_pub_->publish(marker);
}

// ---------------------------------------------------------------------------
// Pipeline log bridge
// ---------------------------------------------------------------------------

void MoordynTetherNode::pipelineLogBridge(LogLevel level,
                                          const std::string & msg)
{
    switch (level)
    {
    case LogLevel::kInfo:
        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        break;
    case LogLevel::kWarn:
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        break;
    case LogLevel::kError:
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        break;
    }
}

}  // namespace moordyn_tether
