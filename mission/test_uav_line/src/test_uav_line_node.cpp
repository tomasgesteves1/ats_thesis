#include "test_uav_line/test_uav_line_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <limits>

namespace test_uav_line {

TestUavLineNode::TestUavLineNode()
    : Node("uav_trajectory_line_node")
{
    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<double>("line_length", 20.0);
    this->declare_parameter<double>("line_yaw", 0.0);
    this->declare_parameter<double>("center_x", 0.0);
    this->declare_parameter<double>("center_y", 0.0);
    this->declare_parameter<double>("min_altitude", 3.0);
    this->declare_parameter<double>("max_altitude", 10.0);
    this->declare_parameter<double>("acceleration", 2.0);
    this->declare_parameter<double>("hover_time", 12.0);
    this->declare_parameter<double>("pause_time", 4.0);
    this->declare_parameter<std::vector<double>>("speed_levels", std::vector<double>{2.0, 3.0});
    this->declare_parameter<bool>("perpendicular_lines", true);
    this->declare_parameter<double>("circle_radius", 5.0);
    this->declare_parameter<double>("circle_speed", 2.5);
    this->declare_parameter<int>("circle_laps", 2);
    this->declare_parameter<double>("yaw_amplitude", 1.2);
    this->declare_parameter<int>("yaw_cycles", 3);
    this->declare_parameter<bool>("px4_offboard", true);
    this->declare_parameter<double>("px4_origin_x", 0.0);
    this->declare_parameter<double>("px4_origin_y", 0.0);
    this->declare_parameter<double>("px4_origin_z", 0.0);
    this->declare_parameter<int>("horizon_stages", 50);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<double>("update_rate_hz", 50.0);
    this->declare_parameter<std::string>("world_frame", "world");

    LineTrajectoryConfig cfg;
    cfg.line_length = this->get_parameter("line_length").as_double();
    cfg.line_yaw = this->get_parameter("line_yaw").as_double();
    cfg.center_x = this->get_parameter("center_x").as_double();
    cfg.center_y = this->get_parameter("center_y").as_double();
    cfg.min_altitude = this->get_parameter("min_altitude").as_double();
    cfg.max_altitude = this->get_parameter("max_altitude").as_double();
    cfg.acceleration = this->get_parameter("acceleration").as_double();
    cfg.hover_time = this->get_parameter("hover_time").as_double();
    cfg.pause_time = this->get_parameter("pause_time").as_double();
    cfg.speed_levels = this->get_parameter("speed_levels").as_double_array();
    cfg.perpendicular_lines = this->get_parameter("perpendicular_lines").as_bool();
    cfg.circle_radius = this->get_parameter("circle_radius").as_double();
    cfg.circle_speed = this->get_parameter("circle_speed").as_double();
    cfg.circle_laps = this->get_parameter("circle_laps").as_int();
    cfg.yaw_amplitude = this->get_parameter("yaw_amplitude").as_double();
    cfg.yaw_cycles = this->get_parameter("yaw_cycles").as_int();

    px4_offboard_ = this->get_parameter("px4_offboard").as_bool();
    px4_origin_[0] = this->get_parameter("px4_origin_x").as_double();
    px4_origin_[1] = this->get_parameter("px4_origin_y").as_double();
    px4_origin_[2] = this->get_parameter("px4_origin_z").as_double();

    pipeline_ = std::make_unique<TestUavLinePipeline>();
    pipeline_->init(cfg);

    // The mission clock only starts once the UAV has taken off and reached the hover altitude,
    // so that the whole sequence is flown regardless of how long arming/takeoff take.
    start_altitude_ = 0.5 * (cfg.min_altitude + cfg.max_altitude) - 1.0;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TestUavLineNode::odomCallback, this, std::placeholders::_1));

    if (px4_offboard_) {
        // Trajectory goes straight to the PX4 position controller (offboard), without the NMPC
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_1/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
        px4_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_1/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    }

    // Relative publishers (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("mission_finished", 10);

    const double hz = this->get_parameter("update_rate_hz").as_double();
    const double period_ms = 1000.0 / std::max(0.1, hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&TestUavLineNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(),
                "Test UAV Line Node initialized: Length=%.1fm, Alt Range=[%.1fm, %.1fm], Total Duration=%.1fs",
                cfg.line_length, cfg.min_altitude, cfg.max_altitude, pipeline_->getTotalDuration());
}

void TestUavLineNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    drone_z_ = msg->pose.pose.position.z;
}

void TestUavLineNode::publishPx4Setpoint(const std::vector<TrajectoryPoint>& points, double dt) {
    const uint64_t stamp_us = this->get_clock()->now().nanoseconds() / 1000;

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = stamp_us;
    mode.position = true;
    mode.velocity = true;
    mode.acceleration = true;
    mode.attitude = false;
    mode.body_rate = false;
    mode.thrust_and_torque = false;
    mode.direct_actuator = false;
    offboard_mode_pub_->publish(mode);

    const auto& p = points.front();
    // Acceleration feedforward by finite difference of the reference velocity
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (points.size() > 1 && dt > 0.0) {
        ax = (points[1].vx - p.vx) / dt;
        ay = (points[1].vy - p.vy) / dt;
        az = (points[1].vz - p.vz) / dt;
    }

    // World ENU -> PX4 local NED, relative to the PX4 origin (spawn point):
    // N = y, E = x, D = -z ; yaw_ned = pi/2 - yaw_enu
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = stamp_us;
    sp.position[0] = static_cast<float>(p.py - px4_origin_[1]);
    sp.position[1] = static_cast<float>(p.px - px4_origin_[0]);
    sp.position[2] = static_cast<float>(-(p.pz - px4_origin_[2]));
    sp.velocity[0] = static_cast<float>(p.vy);
    sp.velocity[1] = static_cast<float>(p.vx);
    sp.velocity[2] = static_cast<float>(-p.vz);
    sp.acceleration[0] = static_cast<float>(ay);
    sp.acceleration[1] = static_cast<float>(ax);
    sp.acceleration[2] = static_cast<float>(-az);
    sp.jerk[0] = sp.jerk[1] = sp.jerk[2] = std::numeric_limits<float>::quiet_NaN();
    sp.yaw = static_cast<float>(M_PI_2 - p.yaw);
    sp.yawspeed = std::numeric_limits<float>::quiet_NaN();
    px4_setpoint_pub_->publish(sp);
}

void TestUavLineNode::timerCallback() {
    const double now_sec = this->get_clock()->now().seconds();

    if (start_time_ < 0.0 && drone_z_ >= start_altitude_) {
        start_time_ = now_sec;
        RCLCPP_INFO(this->get_logger(), "UAV reached %.1f m: starting trajectory sequence.", drone_z_);
    }

    // Until then the reference is the initial hover point (elapsed = 0)
    const double elapsed = (start_time_ < 0.0) ? 0.0 : now_sec - start_time_;
    const int steps = this->get_parameter("horizon_stages").as_int();
    const double dt = this->get_parameter("control_period").as_double();
    const std::string world_frame = this->get_parameter("world_frame").as_string();

    const auto points = pipeline_->generateHorizon(elapsed, steps, dt);

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = world_frame;
    path_msg.header.stamp = this->get_clock()->now();

    for (const auto& pt : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = world_frame;
        pose.header.stamp = path_msg.header.stamp;
        pose.pose.position.x = pt.px;
        pose.pose.position.y = pt.py;
        pose.pose.position.z = pt.pz;
        // Reference yaw travels in the pose orientation (rotation about world Z)
        pose.pose.orientation.z = std::sin(0.5 * pt.yaw);
        pose.pose.orientation.w = std::cos(0.5 * pt.yaw);
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    if (px4_offboard_) {
        publishPx4Setpoint(points, dt);
    }

    const bool finished = pipeline_->isFinished(elapsed);
    std_msgs::msg::Bool finished_msg;
    finished_msg.data = finished;
    finished_pub_->publish(finished_msg);

    if (finished && !mission_finished_logged_) {
        RCLCPP_INFO(this->get_logger(),
                    "Test UAV Line Mission COMPLETED! Holding hover above boat.");
        mission_finished_logged_ = true;
    }
}

} // namespace test_uav_line
