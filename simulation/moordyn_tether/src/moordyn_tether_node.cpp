#include "moordyn_tether/moordyn_tether_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
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
                                std::string("/world/wamv_world/wrench/persistent"));
    wrench_rebase_period_s_ = this->declare_parameter("wrench_rebase_period_s", 10.0);
    wrench_deadband_n_ = this->declare_parameter("wrench_deadband_n", 0.02);
    max_linear_velocity_ = this->declare_parameter("max_linear_velocity", 50.0);
    max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 50.0);
    wrench_max_force_n_ = this->declare_parameter("wrench_max_force_n", 10.0);
    wrench_max_rate_n_per_s_ = this->declare_parameter("wrench_max_rate_n_per_s", 100.0);
    auto wrench_clear_topic = this->declare_parameter("wrench_clear_topic",
                                std::string("/world/wamv_world/wrench/clear"));
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

    // ---- Straight tether model for short lines and winch pull ----
    straight_switch_distance_ = this->declare_parameter("straight_switch_distance", 1.4);
    straight_exit_distance_   = this->declare_parameter("straight_exit_distance", 1.6);
    pull_tension_             = this->declare_parameter("pull_tension", 4.0);
    pull_rate_                = this->declare_parameter("pull_rate", 4.0);

    // ---- Filter parameters ----
    force_filter_cutoff_hz_ = this->declare_parameter("force_filter_cutoff_hz", 0.7);

    // ---- TF2 buffer and listener ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Create pipeline (pure C++ core) ----
    createPipeline();

    // ---- Odometry subscriptions (velocity and orientation only) ----
    using std::placeholders::_1;
    boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        boat_odom_topic, 10,
        std::bind(&MoordynTetherNode::boatOdomCallback, this, _1));
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        drone_odom_topic, 10,
        std::bind(&MoordynTetherNode::droneOdomCallback, this, _1));

    external_force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "~/external_force_drone", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            external_force_drone_ = {msg->x, msg->y, msg->z};
        });

    pull_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/tether_pull", 10,
        std::bind(&MoordynTetherNode::pullCallback, this, _1));

    // ---- Publishers ----
    wrench_pub_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>(
        wrench_topic, 10);
    wrench_clear_pub_ = this->create_publisher<ros_gz_interfaces::msg::Entity>(
        wrench_clear_topic, 10);
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

MoordynTetherNode::~MoordynTetherNode()
{
    // Clear / zero out persistent wrenches in Gazebo upon node destruction
    clearWrench(boat_link_name_);
    clearWrench(drone_link_name_);
}

// ===========================================================================
// Pipeline creation (also used to initialise MoorDyn again after the straight model)
// ===========================================================================

void MoordynTetherNode::createPipeline()
{
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
    pipeline_->setFilterCutoff(force_filter_cutoff_hz_);
}

// ===========================================================================
// Odometry callbacks — store velocity + orientation only, no position math
// ===========================================================================

// A single corrupted odometry sample (e.g. an angular velocity of hundreds of rad/s) becomes, through the lever
// arm, an absurd velocity of the cable end and a tension wave of tens of newtons. Such samples are discarded and
// the previous velocity is kept.
bool MoordynTetherNode::odometryIsSane(const nav_msgs::msg::Odometry & msg, const char * who)
{
    const auto & l = msg.twist.twist.linear;
    const auto & w = msg.twist.twist.angular;
    const auto & q = msg.pose.pose.orientation;
    const double lin = std::sqrt(l.x * l.x + l.y * l.y + l.z * l.z);
    const double ang = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const bool ok = std::isfinite(lin) && std::isfinite(ang) && lin <= max_linear_velocity_ &&
                    ang <= max_angular_velocity_ && qn > 0.9 && qn < 1.1;
    if (!ok) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Discarding corrupted %s odometry sample: |v|=%.1f m/s, |w|=%.1f rad/s, |q|=%.2f", who, lin, ang, qn);
    }
    return ok;
}

void MoordynTetherNode::boatOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!odometryIsSane(*msg, "boat")) { return; }
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
    if (!odometryIsSane(*msg, "drone")) { return; }
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
        // Both ends are read at the same instant, the latest one available for both (each frame is updated by its
        // own odometry, so their latest transforms have different ages)
        const auto stamp_boat = tf_buffer_->lookupTransform(
            "world", boat_anchor_frame_, tf2::TimePointZero, timeout).header.stamp;
        const auto stamp_drone = tf_buffer_->lookupTransform(
            "world", drone_hook_frame_, tf2::TimePointZero, timeout).header.stamp;
        const auto common = rclcpp::Time(stamp_boat) < rclcpp::Time(stamp_drone) ? stamp_boat : stamp_drone;
        const tf2::TimePoint at = tf2_ros::fromMsg(common);
        ends_stamp_ = rclcpp::Time(common).seconds();

        // Boat: anchor attachment point
        auto tf_boat_anchor = tf_buffer_->lookupTransform(
            "world", boat_anchor_frame_, at, timeout);
        body_states_[0].anchor_pos[0] = tf_boat_anchor.transform.translation.x;
        body_states_[0].anchor_pos[1] = tf_boat_anchor.transform.translation.y;
        body_states_[0].anchor_pos[2] = tf_boat_anchor.transform.translation.z;

        // Boat: body center (for lever arm computation in pipeline)
        auto tf_boat_base = tf_buffer_->lookupTransform(
            "world", boat_base_frame_, at, timeout);
        body_states_[0].body_center_pos[0] = tf_boat_base.transform.translation.x;
        body_states_[0].body_center_pos[1] = tf_boat_base.transform.translation.y;
        body_states_[0].body_center_pos[2] = tf_boat_base.transform.translation.z;

        // Drone: hook attachment point
        auto tf_drone_hook = tf_buffer_->lookupTransform(
            "world", drone_hook_frame_, at, timeout);
        body_states_[1].anchor_pos[0] = tf_drone_hook.transform.translation.x;
        body_states_[1].anchor_pos[1] = tf_drone_hook.transform.translation.y;
        body_states_[1].anchor_pos[2] = tf_drone_hook.transform.translation.z;

        // Drone: body center
        auto tf_drone_base = tf_buffer_->lookupTransform(
            "world", drone_base_frame_, at, timeout);
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
        // The simulation runs on while MoorDyn initialises: the first step moves the ends from where the line was
        // initialised to where they are then, over the time elapsed, instead of making them jump there
        for (int k = 0; k < 2; ++k) {
            for (int i = 0; i < 3; ++i) { last_end_pos_[k][i] = body_states_[k].anchor_pos[i]; }
        }
        last_ends_stamp_ = ends_stamp_;
        has_last_ends_ = true;
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
    // Refresh positions from TF; velocities are already up-to-date from callbacks
    if (!lookupBodyPositions()) {
        return;
    }

    // MoorDyn starts each end of the step at the position it is given and moves it with the given velocity, and any
    // mismatch with the position given at the next step (odometry velocities against TF positions, a fixed step
    // against callbacks that come late or bunched) is a jump of the end, which a stiff line turns into tens of
    // newtons. Each step therefore spans the time elapsed between two readings of the ends: it starts from the
    // previous reading and moves the ends to the current one, with the finite differences as velocities.
    if (has_last_ends_ && ends_stamp_ <= last_ends_stamp_ + 1e-6) {
        return;   // no new transforms since the last step
    }
    const double elapsed = has_last_ends_ ? ends_stamp_ - last_ends_stamp_ : 0.0;
    const bool continuous = elapsed > 0.0 && elapsed < 1.0;
    const double dt = continuous ? elapsed : 1.0 / physics_rate_hz_;
    std::array<BodyState, 2> ends{};
    for (int k = 0; k < 2; ++k) {
        for (int i = 0; i < 3; ++i) {
            const double p = body_states_[k].anchor_pos[i];
            const double p0 = continuous ? last_end_pos_[k][i] : p;
            ends[k].anchor_pos[i] = ends[k].body_center_pos[i] = p0;
            ends[k].lin_vel_body[i] = (p - p0) / dt;   // identity orientation: body axes = world axes
            last_end_pos_[k][i] = p;
        }
    }
    last_ends_stamp_ = ends_stamp_;
    has_last_ends_ = true;

    std::vector<double> out_forces;
    std::vector<std::vector<double>> cable_nodes;

    // Vector from the drone hook to the boat anchor
    const double dx = body_states_[0].anchor_pos[0] - body_states_[1].anchor_pos[0];
    const double dy = body_states_[0].anchor_pos[1] - body_states_[1].anchor_pos[1];
    const double dz = body_states_[0].anchor_pos[2] - body_states_[1].anchor_pos[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Short lines use the straight model: from the start while the UAV rests near the winch, and in flight when
    // the pull is requested (landing) and the span falls below the switch distance
    if (!straight_mode_ && dist <= straight_switch_distance_ && (pull_requested_ || !initialized_)) {
        straight_mode_ = true;
        const auto & f = limited_force_[drone_link_name_];
        straight_tension_ = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);  // continuity of the magnitude
        RCLCPP_INFO(this->get_logger(),
            "Span %.2f m <= %.2f m: straight tether model (initial tension %.2f N)",
            dist, straight_switch_distance_, straight_tension_);
    }
    // Back to MoorDyn once the pull is released and ramped to zero and the span exceeds the exit distance (takeoff):
    // MoorDyn is created again and initialised with the current geometry, with the line already off the deck
    if (straight_mode_ && !pull_requested_ && straight_tension_ <= 0.0 && dist > straight_exit_distance_) {
        RCLCPP_INFO(this->get_logger(), "Span %.2f m > %.2f m with the pull released: initialising MoorDyn",
                    dist, straight_exit_distance_);
        straight_mode_ = false;
        if (initialized_) {
            initialized_ = false;
            createPipeline();
        }
        last_init_attempt_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }
    if (!straight_mode_ && !initialized_) {
        checkInitialization();
        return;
    }

    bool stepped = false;
    if (straight_mode_) {
        // Straight massless line: tension ramped towards the pull (zero when not requested), from the drone to the
        // winch; no force on the boat
        const double target = pull_requested_ ? pull_tension_ : 0.0;
        const double step = pull_rate_ * dt;
        straight_tension_ += std::clamp(target - straight_tension_, -step, step);
        const double k = dist > 1e-3 ? straight_tension_ / dist : 0.0;
        out_forces = {0.0, 0.0, 0.0, k * dx, k * dy, k * dz};
        cable_nodes = {
            {body_states_[0].anchor_pos[0], body_states_[0].anchor_pos[1], body_states_[0].anchor_pos[2]},
            {body_states_[1].anchor_pos[0], body_states_[1].anchor_pos[1], body_states_[1].anchor_pos[2]}};
        stepped = true;
    } else {
        stepped = pipeline_->step(ends, dt, out_forces, cable_nodes);
    }

    if (stepped) {
        // The force sent to Gazebo is limited in magnitude and in rate of change, so that a numerical excursion
        // of the cable solver cannot push the vehicles; the raw force is still published for diagnosis.
        const auto fb = limitForce(boat_link_name_, {out_forces[0], out_forces[1], out_forces[2]}, dt);
        const auto fd = limitForce(drone_link_name_, {out_forces[3] + external_force_drone_[0],
                                                      out_forces[4] + external_force_drone_[1],
                                                      out_forces[5] + external_force_drone_[2]}, dt);
        publishWrench(boat_link_name_, fb[0], fb[1], fb[2]);
        publishWrench(drone_link_name_, fd[0], fd[1], fd[2]);
        publishGeometry(cable_nodes);
        publishForceMarkers(out_forces);
        publishForceMagnitudes(out_forces);

        // Publish distance between anchors
        std_msgs::msg::Float64 dist_msg;
        dist_msg.data = dist;
        distance_pub_->publish(dist_msg);

        // Publish exact tether unstretched length (the span in the straight model)
        std_msgs::msg::Float64 len_msg;
        len_msg.data = straight_mode_ ? dist : pipeline_->getTetherLength();
        tether_length_pub_->publish(len_msg);
    }
}

void MoordynTetherNode::pullCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data != pull_requested_) {
        RCLCPP_INFO(this->get_logger(), "Tether pull %s", msg->data ? "requested" : "released");
    }
    pull_requested_ = msg->data;
}

// ===========================================================================
// Publishing helpers
// ===========================================================================

std::array<double, 3> MoordynTetherNode::limitForce(const std::string & entity_name,
                                                    const std::array<double, 3> & raw, double dt)
{
    auto & last = limited_force_[entity_name];   // starts at zero
    std::array<double, 3> f = raw;
    bool limited = false;

    // 1) magnitude
    const double mag = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (mag > wrench_max_force_n_) {
        const double k = wrench_max_force_n_ / mag;
        for (auto & v : f) { v *= k; }
        limited = true;
    }

    // 2) rate of change with respect to the force applied in the previous cycle
    const double max_step = wrench_max_rate_n_per_s_ * dt;
    const std::array<double, 3> d{f[0] - last[0], f[1] - last[1], f[2] - last[2]};
    const double dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (dn > max_step) {
        const double k = max_step / dn;
        for (int i = 0; i < 3; ++i) { f[i] = last[i] + d[i] * k; }
        limited = true;
    }

    if (limited) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Tether force on %s limited: raw %.1f N -> applied %.1f N", entity_name.c_str(), mag,
            std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]));
    }
    last = f;
    return f;
}

void MoordynTetherNode::clearWrench(const std::string & entity_name)
{
    ros_gz_interfaces::msg::Entity clear_msg;
    clear_msg.name = entity_name;
    clear_msg.type = 3;  // LINK
    wrench_clear_pub_->publish(clear_msg);
}

// Gazebo keeps every persistent wrench it receives, applies all of them at every simulation step, and only
// forgets them when the entity is cleared. The applied force is therefore the sum of the messages, which has
// two consequences handled here:
//  - the force is updated by publishing the DIFFERENCE to the force already applied, so the sum is always the
//    current force and the result does not depend on the order in which messages reach Gazebo;
//  - the list grows with every message and slows the simulation down, so it is periodically emptied with a
//    clear, and the absolute force is set again one cycle later (the clear and the set travel on different
//    topics, so sending them together does not guarantee the order and the set could be erased).
// The force is missing for one cycle after every clear; the next cycle applies twice the force to keep the impulse.
void MoordynTetherNode::publishWrench(const std::string & entity_name,
                                      double fx, double fy, double fz)
{
    auto & st = wrench_states_[entity_name];
    const auto now = this->get_clock()->now();
    const std::array<double, 3> f{fx, fy, fz};

    auto publish_set = [&](const std::array<double, 3> & value) {
        ros_gz_interfaces::msg::EntityWrench msg;
        msg.entity.name = entity_name;
        msg.entity.type = 3;  // LINK
        msg.wrench.force.x = value[0];
        msg.wrench.force.y = value[1];
        msg.wrench.force.z = value[2];
        wrench_pub_->publish(msg);
    };

    if (!st.initialised || (now - st.last_rebase).seconds() >= wrench_rebase_period_s_)
    {
        clearWrench(entity_name);
        st.sent = {0.0, 0.0, 0.0};
        st.cleared = true;
        st.initialised = true;
        st.last_rebase = now;
        return;
    }

    if (st.cleared)
    {
        // The force was missing during the cycle that followed the clear: apply twice the force for one cycle to
        // restore the impulse; the difference to the actual force is sent in the next cycle (delta below).
        const std::array<double, 3> boosted{2.0 * f[0], 2.0 * f[1], 2.0 * f[2]};
        publish_set(boosted);
        st.sent = boosted;
        st.cleared = false;
        return;
    }

    const std::array<double, 3> delta{f[0] - st.sent[0], f[1] - st.sent[1], f[2] - st.sent[2]};
    // Dead band: the applied force stays within wrench_deadband_n of the requested one, and the number of entries
    // that Gazebo has to apply at every step is kept small
    if (std::abs(delta[0]) > wrench_deadband_n_ || std::abs(delta[1]) > wrench_deadband_n_ ||
        std::abs(delta[2]) > wrench_deadband_n_)
    {
        publish_set(delta);
        st.sent = f;
    }
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
