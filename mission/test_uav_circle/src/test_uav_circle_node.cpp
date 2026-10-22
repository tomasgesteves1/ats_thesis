#include "test_uav_circle/test_uav_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

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

    pipeline_ = std::make_unique<TestUavCirclePipeline>();

    // Relative publisher (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/ground_truth/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { drone_altitude_ = msg->pose.pose.position.z; });

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
}

} // namespace test_uav_circle
