#include "test_uav_circle/test_uav_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <limits>

namespace test_uav_circle {

TestUavCircleNode::TestUavCircleNode() 
    : Node("uav_trajectory_circle_node") 
{
    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<double>("circle_radius", 3.0);
    this->declare_parameter<double>("circle_omega", 0.2);
    this->declare_parameter<double>("circle_height", 7.0);
    this->declare_parameter<double>("circle_center_x", 0.0);
    this->declare_parameter<double>("circle_center_y", 0.0);
    this->declare_parameter<double>("circle_z_amplitude", 0.0);  // height oscillation along the lap (m)
    this->declare_parameter<int>("horizon_stages", 50);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<double>("update_rate_hz", 50.0);
    this->declare_parameter<std::string>("world_frame", "world");
    // When waiting, the reference holds the start of the circle until the drone reaches start_altitude, so the circle
    // starts at the same instant as the boat's (same trigger in test_usv_circle); otherwise it follows the clock.
    this->declare_parameter<bool>("wait_for_drone", false);
    this->declare_parameter<double>("start_altitude", 3.0);
    // Baseline without the NMPC: the circle is sent straight to the PX4 position controller (offboard), which knows
    // nothing of the tether. The reference is the point tracked by the NMPC, drone_z_offset above the odometry origin.
    this->declare_parameter<bool>("px4_offboard", false);
    this->declare_parameter<double>("drone_z_offset", 0.265);

    pipeline_ = std::make_unique<TestUavCirclePipeline>();

    // Relative publisher (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/ground_truth/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto& p = msg->pose.pose.position;
            drone_altitude_ = p.z;
            drone_gt_ = {p.x, p.y, p.z};
        });

    px4_offboard_ = this->get_parameter("px4_offboard").as_bool();
    if (px4_offboard_) {
        boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/boat/ground_truth/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                const auto& p = msg->pose.pose.position;
                boat_gt_ = {p.x, p.y, p.z};
            });
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_1/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                // NED -> ENU
                drone_px4_ = {msg->position[1], msg->position[0], -msg->position[2]};
                has_px4_odom_ = true;
            });
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_1/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
        px4_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_1/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    }

    double hz = this->get_parameter("update_rate_hz").as_double();
    double period_ms = 1000.0 / std::max(0.1, hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&TestUavCircleNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Test UAV Circle Node initialized.");
}

void TestUavCircleNode::timerCallback() {
    double now_sec = this->get_clock()->now().seconds();

    double radius = this->get_parameter("circle_radius").as_double();
    double omega = this->get_parameter("circle_omega").as_double();
    double height = this->get_parameter("circle_height").as_double();
    double center_x = this->get_parameter("circle_center_x").as_double();
    double center_y = this->get_parameter("circle_center_y").as_double();
    int steps = this->get_parameter("horizon_stages").as_int();
    double dt = this->get_parameter("control_period").as_double();
    std::string world_frame = this->get_parameter("world_frame").as_string();

    double t_ref = now_sec;
    if (this->get_parameter("wait_for_drone").as_bool()) {
        double dt_step = (last_time_sec_ > 0.0) ? (now_sec - last_time_sec_) : 0.0;
        last_time_sec_ = now_sec;
        if (!started_ && drone_altitude_ > this->get_parameter("start_altitude").as_double()) {
            started_ = true;
            RCLCPP_INFO(this->get_logger(), "Drone altitude is %.2f m. Starting UAV circle.", drone_altitude_);
        }
        if (started_) trajectory_time_ += dt_step;
        t_ref = trajectory_time_;
    }

    // Before the start the reference is the first point of the circle, at rest
    auto points = pipeline_->generateCircle(
        t_ref, radius, (this->get_parameter("wait_for_drone").as_bool() && !started_) ? 0.0 : omega,
        height, center_x, center_y, this->get_parameter("circle_z_amplitude").as_double(), steps, dt
    );

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
        pose.pose.orientation.w = 1.0; // Default orientation
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    if (px4_offboard_) {
        publishPx4Setpoint(points, dt);
    }
}

void TestUavCircleNode::publishPx4Setpoint(const std::vector<TrajectoryPoint>& points, double dt) {
    const uint64_t stamp_us = this->get_clock()->now().nanoseconds() / 1000;

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = stamp_us;
    mode.position = true;
    mode.velocity = true;
    mode.acceleration = true;
    offboard_mode_pub_->publish(mode);

    // The setpoint is expressed in the local frame of PX4 by adding the measured offset between its position estimate
    // and the ground truth, so that it does not depend on where the local origin of PX4 is
    const auto& p = points.front();
    const double dz = this->get_parameter("drone_z_offset").as_double();
    std::array<double, 3> offset{0.0, 0.0, 0.0};
    if (has_px4_odom_) {
        for (int k = 0; k < 3; ++k) offset[k] = drone_px4_[k] - drone_gt_[k];
    }
    const double x = p.px + offset[0], y = p.py + offset[1], z = p.pz - dz + offset[2];

    // Acceleration feedforward by finite difference of the reference velocity
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (points.size() > 1 && dt > 0.0) {
        ax = (points[1].vx - p.vx) / dt;
        ay = (points[1].vy - p.vy) / dt;
        az = (points[1].vz - p.vz) / dt;
    }
    // As the NMPC, the UAV looks at the boat
    const double yaw_enu = std::atan2(boat_gt_[1] - drone_gt_[1], boat_gt_[0] - drone_gt_[0]);

    // World ENU -> PX4 local NED: N = y, E = x, D = -z ; yaw_ned = pi/2 - yaw_enu
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = stamp_us;
    sp.position = {static_cast<float>(y), static_cast<float>(x), static_cast<float>(-z)};
    sp.velocity = {static_cast<float>(p.vy), static_cast<float>(p.vx), static_cast<float>(-p.vz)};
    sp.acceleration = {static_cast<float>(ay), static_cast<float>(ax), static_cast<float>(-az)};
    sp.jerk = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
               std::numeric_limits<float>::quiet_NaN()};
    sp.yaw = static_cast<float>(M_PI_2 - yaw_enu);
    sp.yawspeed = std::numeric_limits<float>::quiet_NaN();
    px4_setpoint_pub_->publish(sp);
}

} // namespace test_uav_circle
