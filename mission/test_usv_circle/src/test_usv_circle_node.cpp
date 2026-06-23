#include "test_usv_circle/test_usv_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace test_usv_circle {

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

    // Relative publishers (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("cmd_wrench", 10);

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

    auto points = pipeline_->generateCircle(
        now_sec, radius, omega, height, center_x, center_y, steps, dt
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

} // namespace test_usv_circle
