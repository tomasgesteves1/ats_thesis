#include "test_usv_circle/test_usv_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <sstream>

namespace test_usv_circle {

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

TestUsvCircleNode::TestUsvCircleNode() 
    : Node("usv_trajectory_circle_node") 
{
    // Declare parameters with dynamic typing enabled to support both integer and double values from UI (Rule 2 of CODE_STANDARDS.md)
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;

    this->declare_parameter("circle_radius", rclcpp::ParameterValue(15.0), desc);
    this->declare_parameter("circle_omega", rclcpp::ParameterValue(0.08), desc); // rad/s
    this->declare_parameter("circle_height", rclcpp::ParameterValue(0.0), desc);  // Water level
    this->declare_parameter("circle_center_x", rclcpp::ParameterValue(0.0), desc);
    this->declare_parameter("circle_center_y", rclcpp::ParameterValue(0.0), desc);
    this->declare_parameter("horizon_stages", rclcpp::ParameterValue(20), desc);      // Matching N=20 of usv_mpc
    this->declare_parameter("control_period", rclcpp::ParameterValue(0.1), desc);  // Matching dt=0.1s of usv_mpc
    this->declare_parameter("update_rate_hz", rclcpp::ParameterValue(10.0), desc); // 10Hz update rate
    this->declare_parameter("world_frame", rclcpp::ParameterValue("world"), desc);
    this->declare_parameter("wait_for_drone", rclcpp::ParameterValue(false), desc); // Wait for drone to takeoff before starting trajectory
    this->declare_parameter("start_altitude", rclcpp::ParameterValue(3.0), desc);  // Drone altitude that starts the trajectory (m)
    // Instead of the circle, a sequence of fixed poses (step references), "t,x,y,yaw_deg; ..."
    this->declare_parameter("waypoints", rclcpp::ParameterValue(std::string("")), desc);
    waypoints_ = parseWaypoints(this->get_parameter("waypoints").as_string());
    // Instead of the circle, a trajectory tabulated in a CSV file (t,x,y,psi,...), started start_delay seconds after
    // the first odometry of the boat, without waiting for the drone
    this->declare_parameter("trajectory_file", rclcpp::ParameterValue(std::string("")), desc);
    this->declare_parameter("trajectory_loop", rclcpp::ParameterValue(true), desc);
    this->declare_parameter("start_delay", rclcpp::ParameterValue(20.0), desc);

    // Dynamic testing configuration inputs from Dashboard
    this->declare_parameter("mode_step", rclcpp::ParameterValue(false), desc);
    this->declare_parameter("mode_open_loop", rclcpp::ParameterValue(false), desc);
    this->declare_parameter("step_x", rclcpp::ParameterValue(10.0), desc);
    this->declare_parameter("step_y", rclcpp::ParameterValue(0.0), desc);
    this->declare_parameter("step_yaw", rclcpp::ParameterValue(0.0), desc); // Degrees
    this->declare_parameter("open_loop_X", rclcpp::ParameterValue(500.0), desc);
    this->declare_parameter("open_loop_Y", rclcpp::ParameterValue(0.0), desc);
    this->declare_parameter("open_loop_N", rclcpp::ParameterValue(0.0), desc);

    pipeline_ = std::make_unique<TestUsvCirclePipeline>();
    const std::string table = this->get_parameter("trajectory_file").as_string();
    if (!table.empty()) {
        if (pipeline_->loadTable(table, this->get_parameter("trajectory_loop").as_bool())) {
            RCLCPP_INFO(this->get_logger(), "USV table mode: %s.", table.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not read the trajectory file %s.", table.c_str());
        }
    }

    // Relative publishers (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("cmd_wrench", 10);

    // Global subscriber to drone odometry to coordinate takeoff/trajectory start
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/ground_truth/odometry", 10,
        std::bind(&TestUsvCircleNode::droneOdomCallback, this, std::placeholders::_1)
    );

    // Global subscriber to boat odometry to keep current position before takeoff
    boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/boat/ground_truth/odometry", 10,
        std::bind(&TestUsvCircleNode::boatOdomCallback, this, std::placeholders::_1)
    );

    double hz = this->get_double_param("update_rate_hz");
    double period_ms = 1000.0 / std::max(0.1, hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&TestUsvCircleNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Test USV MPC & Thruster Mapper Node initialized.");
}

double TestUsvCircleNode::get_double_param(const std::string& name) {
    auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(param.as_int());
    }
    return param.as_double();
}

int TestUsvCircleNode::get_int_param(const std::string& name) {
    auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return static_cast<int>(param.as_double());
    }
    return param.as_int();
}

void TestUsvCircleNode::timerCallback() {
    bool mode_step = this->get_parameter("mode_step").as_bool();
    bool mode_open_loop = this->get_parameter("mode_open_loop").as_bool();
    std::string world_frame = this->get_parameter("world_frame").as_string();
    int steps = this->get_int_param("horizon_stages");

    // =========================================================================
    // CASE 1: OPEN-LOOP FORCE TESTING (Bypass MPC, publish directly to Mapper)
    // =========================================================================
    if (mode_open_loop) {
        double ol_x = this->get_double_param("open_loop_X");
        double ol_y = this->get_double_param("open_loop_Y");
        double ol_n = this->get_double_param("open_loop_N");

        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        wrench_msg.header.stamp = this->get_clock()->now();
        wrench_msg.header.frame_id = "boat/base_link";
        wrench_msg.wrench.force.x = ol_x;
        wrench_msg.wrench.force.y = ol_y;
        wrench_msg.wrench.torque.z = ol_n;

        wrench_pub_->publish(wrench_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TEST OPEN-LOOP ACTIVE: force inputs X=%.1f N, Y=%.1f N, N=%.1f Nm", ol_x, ol_y, ol_n);

        // Publish empty reference path to clear MPC buffer
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = world_frame;
        path_msg.header.stamp = this->get_clock()->now();
        path_pub_->publish(path_msg);
        return;
    }

    // =========================================================================
    // CASE 2: STEP RESPONSE MODE (Position regulation step target)
    // =========================================================================
    if (mode_step) {
        double step_x = this->get_double_param("step_x");
        double step_y = this->get_double_param("step_y");
        double step_yaw_deg = this->get_double_param("step_yaw");
        double yaw_rad = step_yaw_deg * M_PI / 180.0;

        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = world_frame;
        path_msg.header.stamp = this->get_clock()->now();

        // Calculate step orientation quaternion (yaw-only)
        double half_yaw = yaw_rad * 0.5;
        double qz = std::sin(half_yaw);
        double qw = std::cos(half_yaw);

        for (int i = 0; i <= steps; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = world_frame;
            pose.header.stamp = path_msg.header.stamp;
            pose.pose.position.x = step_x;
            pose.pose.position.y = step_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = qz;
            pose.pose.orientation.w = qw;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TEST STEP RESPONSE ACTIVE: Target Pose [X=%.2f m, Y=%.2f m, Yaw=%.2f deg]", step_x, step_y, step_yaw_deg);
        return;
    }

    // =========================================================================
    // CASE 3: CIRCULAR TRAJECTORY TRACKING MODE (Normal dynamic tracking)
    // =========================================================================
    double now_sec = this->get_clock()->now().seconds();
    double radius = this->get_double_param("circle_radius");
    double omega = this->get_double_param("circle_omega");
    double height = this->get_double_param("circle_height");
    double center_x = this->get_double_param("circle_center_x");
    double center_y = this->get_double_param("circle_center_y");
    double dt = this->get_double_param("control_period");

    double dt_step = (last_time_sec_ > 0.0) ? (now_sec - last_time_sec_) : 0.0;
    last_time_sec_ = now_sec;

    if (pipeline_->hasTable()) {
        // The trajectory time starts at -start_delay with the first odometry of the boat; before zero the reference
        // holds the first pose of the table
        if (has_boat_odom_) {
            if (!table_started_) {
                table_time_ = -this->get_double_param("start_delay");
                table_started_ = true;
            } else {
                table_time_ += dt_step;
            }
        }
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = world_frame;
        path_msg.header.stamp = this->get_clock()->now();
        for (const auto& q : pipeline_->sampleTable(table_time_, steps, dt)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = q[0];
            pose.pose.position.y = q[1];
            pose.pose.orientation.z = std::sin(0.5 * q[2]);
            pose.pose.orientation.w = std::cos(0.5 * q[2]);
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
        return;
    }

    bool wait_for_drone = this->get_parameter("wait_for_drone").as_bool();

    if (wait_for_drone && !drone_started_flying_) {
        if (drone_altitude_ > this->get_double_param("start_altitude")) {
            drone_started_flying_ = true;
            RCLCPP_INFO(this->get_logger(), "Drone altitude is %.2f m. Drone is flying! Starting boat trajectory.", drone_altitude_);
        } else {
            // Publish static reference at current boat position to keep it still
            auto path_msg = nav_msgs::msg::Path();
            path_msg.header.frame_id = world_frame;
            path_msg.header.stamp = this->get_clock()->now();

            double half_yaw = boat_yaw_ * 0.5;
            double qz = std::sin(half_yaw);
            double qw = std::cos(half_yaw);

            for (int i = 0; i <= steps; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = world_frame;
                pose.header.stamp = path_msg.header.stamp;
                pose.pose.position.x = boat_x_;
                pose.pose.position.y = boat_y_;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = qz;
                pose.pose.orientation.w = qw;
                path_msg.poses.push_back(pose);
            }

            path_pub_->publish(path_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Boat waiting for drone to start flying (current altitude: %.2f m)...", drone_altitude_);
            
            last_time_sec_ = now_sec; // Keep last_time updated to prevent sudden jumps
            return;
        }
    }
    trajectory_time_ += dt_step;

    if (!waypoints_.empty()) {
        const auto& w = activeWaypoint(waypoints_, trajectory_time_);
        const double half_yaw = 0.5 * w[3] * M_PI / 180.0;
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = world_frame;
        path_msg.header.stamp = this->get_clock()->now();
        for (int i = 0; i <= steps; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = w[1];
            pose.pose.position.y = w[2];
            pose.pose.orientation.z = std::sin(half_yaw);
            pose.pose.orientation.w = std::cos(half_yaw);
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
        return;
    }

    auto points = pipeline_->generateCircle(
        trajectory_time_, radius, omega, height, center_x, center_y, steps, dt
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

        // Correct tangent alignment heading (yaw = atan2(vy, vx))
        double yaw = std::atan2(pt.vy, pt.vx);
        double half_yaw = yaw * 0.5;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(half_yaw);
        pose.pose.orientation.w = std::cos(half_yaw);

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

void TestUsvCircleNode::droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    drone_altitude_ = msg->pose.pose.position.z;
}

void TestUsvCircleNode::boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_boat_odom_ = true;
    boat_x_ = msg->pose.pose.position.x;
    boat_y_ = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    boat_yaw_ = 2.0 * std::atan2(qz, qw);
}

} // namespace test_usv_circle
