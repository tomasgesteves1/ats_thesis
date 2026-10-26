#include "test_uav_circle/test_uav_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace test_uav_circle {

namespace {
// Waypoints "t,a,b,c; t,a,b,c; ..." (times from the start of the trajectory, increasing)
std::vector<std::array<double, 4>> parseWaypoints(const std::string& text) {
    std::vector<std::array<double, 4>> out;
    std::stringstream rows(text);
    std::string row;
    while (std::getline(rows, row, ';')) {
        std::array<double, 4> w{};
        std::stringstream cols(row);
        std::string col;
        int k = 0;
        while (k < 4 && std::getline(cols, col, ',')) w[k++] = std::stod(col);
        if (k == 4) out.push_back(w);
    }
    return out;
}

// Last waypoint whose time has been reached (the first one before that)
const std::array<double, 4>& activeWaypoint(const std::vector<std::array<double, 4>>& wps, double t) {
    size_t i = 0;
    while (i + 1 < wps.size() && wps[i + 1][0] <= t) ++i;
    return wps[i];
}
}  // namespace

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
    // Instead of the circle, a sequence of fixed points (step references), "t,x,y,z; ..."
    this->declare_parameter<std::string>("waypoints", "");
    // Instead of the circle, a periodic trajectory tabulated in a CSV file (t,x,y,z,vx,vy,vz,...). The UAV is first led
    // to its first point (climb, then horizontal, at approach_speed), and the trajectory starts once it is within
    // start_tolerance of it, entering from hover over ramp_time.
    this->declare_parameter<std::string>("trajectory_file", "");
    this->declare_parameter<double>("ramp_time", 8.0);
    this->declare_parameter<double>("approach_speed", 1.0);
    this->declare_parameter<double>("start_tolerance", 0.3);
    this->declare_parameter<bool>("trajectory_loop", true);  // false: hold the last point of the table after its end

    pipeline_ = std::make_unique<TestUavCirclePipeline>();

    // Relative publisher (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/ground_truth/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto& p = msg->pose.pose.position;
            drone_altitude_ = p.z;
            drone_gt_ = {p.x, p.y, p.z};
            has_drone_odom_ = true;
        });

    px4_offboard_ = this->get_parameter("px4_offboard").as_bool();
    waypoints_ = parseWaypoints(this->get_parameter("waypoints").as_string());
    if (!waypoints_.empty()) {
        RCLCPP_INFO(this->get_logger(), "UAV waypoint mode: %zu waypoints.", waypoints_.size());
    }
    const std::string table = this->get_parameter("trajectory_file").as_string();
    if (!table.empty()) {
        pipeline_->setTableLoop(this->get_parameter("trajectory_loop").as_bool());
        if (pipeline_->loadTable(table)) {
            RCLCPP_INFO(this->get_logger(), "UAV table mode: %s.", table.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not read the trajectory file %s.", table.c_str());
        }
    }
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
    // The table has its own start (approach to its first point)
    if (this->get_parameter("wait_for_drone").as_bool() && !pipeline_->hasTable()) {
        double dt_step = (last_time_sec_ > 0.0) ? (now_sec - last_time_sec_) : 0.0;
        last_time_sec_ = now_sec;
        if (!started_ && drone_altitude_ > this->get_parameter("start_altitude").as_double()) {
            started_ = true;
            RCLCPP_INFO(this->get_logger(), "Drone altitude is %.2f m. Starting UAV circle.", drone_altitude_);
        }
        if (started_) trajectory_time_ += dt_step;
        t_ref = trajectory_time_;
    }

    // Before the start the reference is the first point of the circle (or the first waypoint), at rest
    std::vector<TrajectoryPoint> points;
    if (pipeline_->hasTable()) {
        const double dt_step = (last_table_time_ > 0.0) ? (now_sec - last_table_time_) : 0.0;
        last_table_time_ = now_sec;
        if (started_) {
            table_time_ += dt_step;
            points = pipeline_->sampleTable(table_time_, this->get_parameter("ramp_time").as_double(), steps, dt);
        } else {
            points = approachPoints(steps, dt, dt_step);
        }
    } else if (!waypoints_.empty()) {
        const auto& w = activeWaypoint(waypoints_, t_ref);
        points.assign(steps + 1, TrajectoryPoint{w[1], w[2], w[3], 0.0, 0.0, 0.0});
    } else {
        points = pipeline_->generateCircle(
            t_ref, radius, (this->get_parameter("wait_for_drone").as_bool() && !started_) ? 0.0 : omega,
            height, center_x, center_y, this->get_parameter("circle_z_amplitude").as_double(), steps, dt
        );
    }

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

std::vector<TrajectoryPoint> TestUavCircleNode::approachPoints(int steps, double dt, double dt_step) {
    // The approach point moves at approach_speed towards the first point of the table, climbing first and then
    // horizontally, but never more than a short leash ahead of the UAV, so that it waits for the takeoff
    const auto first = pipeline_->firstTablePoint();
    const std::array<double, 3> goal{first.px, first.py, first.pz};
    const double speed = this->get_parameter("approach_speed").as_double();
    constexpr double leash = 1.0;
    if (!has_drone_odom_) {
        return std::vector<TrajectoryPoint>(steps + 1, TrajectoryPoint{goal[0], goal[1], goal[2], 0.0, 0.0, 0.0});
    }
    // Point of the UAV tracked by the controllers, drone_z_offset above the odometry origin
    const std::array<double, 3> uav{drone_gt_[0], drone_gt_[1],
                                    drone_gt_[2] + this->get_parameter("drone_z_offset").as_double()};
    if (!approach_init_) {
        approach_ = uav;
        approach_init_ = true;
    }
    // Advances a point by a distance along the approach path; returns the unit direction of motion (zero at the goal)
    auto advance = [&goal](std::array<double, 3>& p, double dist) {
        std::array<double, 3> dir{0.0, 0.0, 0.0};
        const double dz = goal[2] - p[2];
        if (std::abs(dz) > 1e-6) {
            const double s = std::min(dist, std::abs(dz));
            dir[2] = (dz > 0.0) ? 1.0 : -1.0;
            p[2] += dir[2] * s;
            dist -= s;
            if (dist <= 0.0) return dir;
        }
        const double ex = goal[0] - p[0], ey = goal[1] - p[1], e = std::hypot(ex, ey);
        if (e > 1e-6) {
            const double s = std::min(dist, e);
            dir = {ex / e, ey / e, 0.0};
            p[0] += dir[0] * s;
            p[1] += dir[1] * s;
        }
        return dir;
    };
    double lead = 0.0;
    for (int k = 0; k < 3; ++k) lead += (approach_[k] - uav[k]) * (approach_[k] - uav[k]);
    if (std::sqrt(lead) < leash) advance(approach_, speed * dt_step);

    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);
    std::array<double, 3> p = approach_;
    for (int i = 0; i <= steps; ++i) {
        auto q = p;
        const auto dir = advance(q, 1e-3);  // direction of motion at this point
        points.push_back(TrajectoryPoint{p[0], p[1], p[2], speed * dir[0], speed * dir[1], speed * dir[2]});
        advance(p, speed * dt);
    }

    double err = 0.0, gap = 0.0;
    for (int k = 0; k < 3; ++k) {
        err += (uav[k] - goal[k]) * (uav[k] - goal[k]);
        gap += (approach_[k] - goal[k]) * (approach_[k] - goal[k]);
    }
    if (gap < 1e-6 && std::sqrt(err) < this->get_parameter("start_tolerance").as_double()) {
        started_ = true;
        table_time_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "UAV at the first point of the table. Starting the trajectory.");
    }
    return points;
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
