#include "moordyn_tether/moordyn_tether_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <cmath>

namespace moordyn_tether
{

// ===========================================================================
// Constructor
// ===========================================================================

MoordynTetherNode::MoordynTetherNode(const rclcpp::NodeOptions & options)
    : Node("moordyn_tether_node", options)
{
    // ---- Load parameters from YAML ----
    boat_anchor_frame_ = this->declare_parameter("boat_anchor_frame",
                              std::string("boat/tether_anchor"));
    boat_base_frame_   = this->declare_parameter("boat_base_frame",
                              std::string("boat/base_link"));
    drone_hook_frame_  = this->declare_parameter("drone_hook_frame",
                              std::string("drone/base_link"));
    drone_base_frame_  = this->declare_parameter("drone_base_frame",
                              std::string("drone/base_link"));
    tf_timeout_s_      = this->declare_parameter("tf_timeout_s", 0.1);

    boat_link_name_    = this->declare_parameter("boat_link_name",
                              std::string("wamv::wamv/base_link"));
    drone_link_name_   = this->declare_parameter("drone_link_name",
                              std::string("x500::base_link"));

    physics_rate_hz_   = this->declare_parameter("physics_rate_hz", 50.0);
    marker_line_width_ = this->declare_parameter("marker_line_width", 0.01);
    marker_color_      = this->declare_parameter("marker_color",
                              std::vector<double>{0.2, 0.2, 0.2, 1.0});

    auto boat_odom_topic  = this->declare_parameter("boat_odom_topic",
                                std::string("boat/ground_truth/odometry"));
    auto drone_odom_topic = this->declare_parameter("drone_odom_topic",
                                std::string("drone/ground_truth/odometry"));
    auto wrench_topic     = this->declare_parameter("wrench_topic",
                                std::string("/world/wamv_world/wrench"));
    auto marker_topic     = this->declare_parameter("marker_topic",
                                std::string("tether_geometry_marker"));

    force_marker_topic_ = this->declare_parameter("force_marker_topic",
                              std::string("tether_force_markers"));
    force_marker_max_force_ = this->declare_parameter("force_marker_max_force", 10.0);
    force_marker_max_length_ = this->declare_parameter("force_marker_max_length", 2.0);
    force_marker_color_ = this->declare_parameter("force_marker_color",
                              std::vector<double>{1.0, 0.0, 0.0, 1.0});

    force_mag_boat_topic_ = this->declare_parameter("force_magnitude_boat_topic",
                              std::string("tether_force_boat_mag"));
    force_mag_drone_topic_ = this->declare_parameter("force_magnitude_drone_topic",
                              std::string("tether_force_drone_mag"));

    // ---- Virtual winch parameters ----
    winch_enabled_      = this->declare_parameter("winch_enabled", false);
    winch_mode_         = this->declare_parameter("winch_mode", std::string("geometric"));
    winch_slack_factor_ = this->declare_parameter("winch_slack_factor", 1.15);
    winch_target_tension_ = this->declare_parameter("winch_target_tension", 1.0);
    winch_kp_tension_   = this->declare_parameter("winch_kp_tension", 1.0);
    winch_kd_tension_   = this->declare_parameter("winch_kd_tension", 0.1);
    winch_speed_limit_  = this->declare_parameter("winch_speed_limit", 10.0);
    winch_min_length_   = this->declare_parameter("winch_min_length", 3.0);
    winch_max_length_   = this->declare_parameter("winch_max_length", 100.0);

    // ---- Filter parameters ----
    force_filter_alpha_ = this->declare_parameter("force_filter_alpha", 0.2);

    // ---- TF2 buffer and listener ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Create pipeline (pure C++ core) ----
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("moordyn_tether");
    std::string config_path = pkg_share + "/config/lines.txt";

    pipeline_ = std::make_unique<MoordynTetherPipeline>(
        config_path,
        [this](LogLevel level, const std::string & msg) {
            pipelineLogBridge(level, msg);
        });

    // Forward winch configuration to the pipeline.
    WinchConfig wcfg;
    wcfg.enabled      = winch_enabled_;
    if (winch_mode_ == "tension") {
        wcfg.mode = WinchMode::TENSION;
    } else {
        wcfg.mode = WinchMode::GEOMETRIC;
    }
    wcfg.slack_factor  = winch_slack_factor_;
    wcfg.target_tension = winch_target_tension_;
    wcfg.kp_tension    = winch_kp_tension_;
    wcfg.kd_tension    = winch_kd_tension_;
    wcfg.winch_speed_limit = winch_speed_limit_;
    wcfg.min_length    = winch_min_length_;
    wcfg.max_length    = winch_max_length_;
    pipeline_->setWinchConfig(wcfg);

    // Set up filter
    pipeline_->setFilterAlpha(force_filter_alpha_);

    // ---- Odometry subscriptions (velocity and orientation only) ----
    using std::placeholders::_1;
    boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        boat_odom_topic, 10,
        std::bind(&MoordynTetherNode::boatOdomCallback, this, _1));
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        drone_odom_topic, 10,
        std::bind(&MoordynTetherNode::droneOdomCallback, this, _1));

    // ---- Publishers ----
    wrench_pub_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>(
        wrench_topic, 10);
    geometry_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        marker_topic, 10);
    force_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        force_marker_topic_, 10);
    force_mag_boat_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        force_mag_boat_topic_, 10);
    force_mag_drone_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        force_mag_drone_topic_, 10);
    distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "~/tether_distance", 10);
    tether_length_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "~/tether_length", 10);

    // ---- Physics timer ----
    auto period_ms = static_cast<int>(1000.0 / physics_rate_hz_);
    physics_timer_ = this->create_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MoordynTetherNode::physicsLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "MoordynTetherNode initialized. Physics rate: %.1f Hz\n"
                "  Boat  anchor: '%s'  base: '%s'\n"
                "  Drone hook:   '%s'  base: '%s'",
                physics_rate_hz_,
                boat_anchor_frame_.c_str(), boat_base_frame_.c_str(),
                drone_hook_frame_.c_str(),  drone_base_frame_.c_str());
}

// ===========================================================================
// Odometry callbacks — store velocity + orientation only, no position math
// ===========================================================================

void MoordynTetherNode::boatOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto & s = body_states_[0];

    // Orientation quaternion [qx, qy, qz, qw]
    s.quat[0] = msg->pose.pose.orientation.x;
    s.quat[1] = msg->pose.pose.orientation.y;
    s.quat[2] = msg->pose.pose.orientation.z;
    s.quat[3] = msg->pose.pose.orientation.w;

    // Body-frame linear velocity (expressed in child_frame_id = body frame)
    s.lin_vel_body[0] = msg->twist.twist.linear.x;
    s.lin_vel_body[1] = msg->twist.twist.linear.y;
    s.lin_vel_body[2] = msg->twist.twist.linear.z;

    // Body-frame angular velocity
    s.ang_vel_body[0] = msg->twist.twist.angular.x;
    s.ang_vel_body[1] = msg->twist.twist.angular.y;
    s.ang_vel_body[2] = msg->twist.twist.angular.z;

    boat_odom_received_ = true;
}

void MoordynTetherNode::droneOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto & s = body_states_[1];

    s.quat[0] = msg->pose.pose.orientation.x;
    s.quat[1] = msg->pose.pose.orientation.y;
    s.quat[2] = msg->pose.pose.orientation.z;
    s.quat[3] = msg->pose.pose.orientation.w;

    s.lin_vel_body[0] = msg->twist.twist.linear.x;
    s.lin_vel_body[1] = msg->twist.twist.linear.y;
    s.lin_vel_body[2] = msg->twist.twist.linear.z;

    s.ang_vel_body[0] = msg->twist.twist.angular.x;
    s.ang_vel_body[1] = msg->twist.twist.angular.y;
    s.ang_vel_body[2] = msg->twist.twist.angular.z;

    drone_odom_received_ = true;
}

// ===========================================================================
// TF lookup — fills anchor_pos and body_center_pos for both BodyStates
// ===========================================================================

bool MoordynTetherNode::lookupBodyPositions()
{
    const auto timeout = tf2::durationFromSec(tf_timeout_s_);

    try {
        // Boat: anchor attachment point
        auto tf_boat_anchor = tf_buffer_->lookupTransform(
            "world", boat_anchor_frame_, tf2::TimePointZero, timeout);
        body_states_[0].anchor_pos[0] = tf_boat_anchor.transform.translation.x;
        body_states_[0].anchor_pos[1] = tf_boat_anchor.transform.translation.y;
        body_states_[0].anchor_pos[2] = tf_boat_anchor.transform.translation.z;

        // Boat: body center (for lever arm computation in pipeline)
        auto tf_boat_base = tf_buffer_->lookupTransform(
            "world", boat_base_frame_, tf2::TimePointZero, timeout);
        body_states_[0].body_center_pos[0] = tf_boat_base.transform.translation.x;
        body_states_[0].body_center_pos[1] = tf_boat_base.transform.translation.y;
        body_states_[0].body_center_pos[2] = tf_boat_base.transform.translation.z;

        // Drone: hook attachment point
        auto tf_drone_hook = tf_buffer_->lookupTransform(
            "world", drone_hook_frame_, tf2::TimePointZero, timeout);
        body_states_[1].anchor_pos[0] = tf_drone_hook.transform.translation.x;
        body_states_[1].anchor_pos[1] = tf_drone_hook.transform.translation.y;
        body_states_[1].anchor_pos[2] = tf_drone_hook.transform.translation.z;

        // Drone: body center
        auto tf_drone_base = tf_buffer_->lookupTransform(
            "world", drone_base_frame_, tf2::TimePointZero, timeout);
        body_states_[1].body_center_pos[0] = tf_drone_base.transform.translation.x;
        body_states_[1].body_center_pos[1] = tf_drone_base.transform.translation.y;
        body_states_[1].body_center_pos[2] = tf_drone_base.transform.translation.z;

        return true;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF lookup failed: %s", ex.what());
        return false;
    }
}

// ===========================================================================
// Initialization gate
// ===========================================================================

void MoordynTetherNode::checkInitialization()
{
    // Throttle attempts to once per second
    auto now = this->get_clock()->now();
    if ((now - last_init_attempt_).seconds() < 1.0) {
        return;
    }
    last_init_attempt_ = now;

    if (!boat_odom_received_ || !drone_odom_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for odometry on both boat and drone topics...");
        return;
    }

    if (!lookupBodyPositions()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for TF frames '%s' / '%s' / '%s' / '%s'...",
                             boat_anchor_frame_.c_str(), boat_base_frame_.c_str(),
                             drone_hook_frame_.c_str(),  drone_base_frame_.c_str());
        return;
    }

    if (pipeline_->initialize(body_states_)) {
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "MoorDyn initialized. "
                    "Boat anchor: [%.2f, %.2f, %.2f] | Drone hook: [%.2f, %.2f, %.2f]",
                    body_states_[0].anchor_pos[0],
                    body_states_[0].anchor_pos[1],
                    body_states_[0].anchor_pos[2],
                    body_states_[1].anchor_pos[0],
                    body_states_[1].anchor_pos[1],
                    body_states_[1].anchor_pos[2]);
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "MoorDyn_Init FAILED. Check that lines.txt has exactly 2 Coupled points.");
    }
}

// ===========================================================================
// Physics loop (runs at physics_rate_hz_)
// ===========================================================================

void MoordynTetherNode::physicsLoop()
{
    if (!initialized_) {
        checkInitialization();
        return;
    }

    // Refresh positions from TF; velocities are already up-to-date from callbacks
    if (!lookupBodyPositions()) {
        return;
    }

    std::vector<double> out_forces;
    std::vector<std::vector<double>> cable_nodes;
    const double dt = 1.0 / physics_rate_hz_;

    if (pipeline_->step(body_states_, dt, out_forces, cable_nodes)) {
        publishWrench(boat_link_name_,
                      out_forces[0], out_forces[1], out_forces[2]);
        publishWrench(drone_link_name_,
                      out_forces[3], out_forces[4], out_forces[5]);
        publishGeometry(cable_nodes);
        publishForceMarkers(out_forces);
        publishForceMagnitudes(out_forces);

        // Publish distance between anchors
        const double dx = body_states_[0].anchor_pos[0] - body_states_[1].anchor_pos[0];
        const double dy = body_states_[0].anchor_pos[1] - body_states_[1].anchor_pos[1];
        const double dz = body_states_[0].anchor_pos[2] - body_states_[1].anchor_pos[2];
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        
        std_msgs::msg::Float64 dist_msg;
        dist_msg.data = dist;
        distance_pub_->publish(dist_msg);

        // Publish exact tether unstretched length
        std_msgs::msg::Float64 len_msg;
        len_msg.data = pipeline_->getTetherLength();
        tether_length_pub_->publish(len_msg);
    }
}

// ===========================================================================
// Publishing helpers
// ===========================================================================

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
    marker.header.stamp    = this->get_clock()->now();
    marker.ns              = "tether_catenary";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.scale.x         = marker_line_width_;
    marker.color.r         = static_cast<float>(marker_color_[0]);
    marker.color.g         = static_cast<float>(marker_color_[1]);
    marker.color.b         = static_cast<float>(marker_color_[2]);
    marker.color.a         = static_cast<float>(marker_color_[3]);

    for (const auto & node_pos : nodes) {
        geometry_msgs::msg::Point p;
        p.x = node_pos[0];
        p.y = node_pos[1];
        p.z = node_pos[2];
        marker.points.push_back(p);
    }

    geometry_pub_->publish(marker);
}

void MoordynTetherNode::publishForceMarkers(const std::vector<double> & out_forces)
{
    if (out_forces.size() < 6) {
        return;
    }

    visualization_msgs::msg::MarkerArray msg;

    auto create_arrow = [this](int id, const double* anchor_pos, const double* force, const std::string& ns) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp    = this->get_clock()->now();
        marker.ns              = ns;
        marker.id              = id;
        marker.type            = visualization_msgs::msg::Marker::ARROW;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        
        // Arrow scale: x is shaft diameter, y is head diameter, z is head length.
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = static_cast<float>(force_marker_color_[0]);
        marker.color.g = static_cast<float>(force_marker_color_[1]);
        marker.color.b = static_cast<float>(force_marker_color_[2]);
        marker.color.a = static_cast<float>(force_marker_color_[3]);

        geometry_msgs::msg::Point p_start;
        p_start.x = anchor_pos[0];
        p_start.y = anchor_pos[1];
        p_start.z = anchor_pos[2];

        geometry_msgs::msg::Point p_end;
        double force_mag = std::sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);

        if (force_mag < 1e-6)
        {
            p_end = p_start;
            // Hide arrow if force is zero
            marker.scale.x = 0.0;
            marker.scale.y = 0.0;
            marker.scale.z = 0.0;
        }
        else
        {
            // Cap the force magnitude
            double capped_force = std::min(force_mag, force_marker_max_force_);
            
            // Map the capped force to the maximum length linearly
            double display_length = (capped_force / force_marker_max_force_) * force_marker_max_length_;
            
            p_end.x = anchor_pos[0] + (force[0] / force_mag) * display_length;
            p_end.y = anchor_pos[1] + (force[1] / force_mag) * display_length;
            p_end.z = anchor_pos[2] + (force[2] / force_mag) * display_length;
        }

        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        return marker;
    };

    msg.markers.push_back(create_arrow(0, body_states_[0].anchor_pos, &out_forces[0], "boat_force"));
    msg.markers.push_back(create_arrow(1, body_states_[1].anchor_pos, &out_forces[3], "drone_force"));

    force_marker_pub_->publish(msg);
}

void MoordynTetherNode::publishForceMagnitudes(const std::vector<double> & out_forces)
{
    if (out_forces.size() < 6) {
        return;
    }

    auto calc_mag = [](double fx, double fy, double fz) {
        return std::sqrt(fx * fx + fy * fy + fz * fz);
    };

    std_msgs::msg::Float64 boat_msg;
    boat_msg.data = calc_mag(out_forces[0], out_forces[1], out_forces[2]);
    force_mag_boat_pub_->publish(boat_msg);

    std_msgs::msg::Float64 drone_msg;
    drone_msg.data = calc_mag(out_forces[3], out_forces[4], out_forces[5]);
    force_mag_drone_pub_->publish(drone_msg);
}

// ===========================================================================
// Pipeline log bridge
// ===========================================================================

void MoordynTetherNode::pipelineLogBridge(LogLevel level,
                                          const std::string & msg)
{
    switch (level) {
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
