#include "trajectory_generator/trajectory_generator_uav_circle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace trajectory_generator {

UavTrajectoryCircleNode::UavTrajectoryCircleNode() 
    : Node("uav_trajectory_circle_node") 
{
    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<double>("circle_radius", 3.0);
    this->declare_parameter<double>("circle_omega", 0.2);
    this->declare_parameter<double>("circle_height", 7.0);
    this->declare_parameter<double>("circle_center_x", 0.0);
    this->declare_parameter<double>("circle_center_y", 0.0);
    this->declare_parameter<int>("horizon_stages", 50);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<double>("update_rate_hz", 50.0);
    this->declare_parameter<std::string>("world_frame", "world");

    pipeline_ = std::make_unique<TrajectoryPipeline>();

    // Relative publisher (Rule 4 of CODE_STANDARDS.md)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);

    double hz = this->get_parameter("update_rate_hz").as_double();
    double period_ms = 1000.0 / std::max(0.1, hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&UavTrajectoryCircleNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "UAV Trajectory Circle Node initialized.");
}

void UavTrajectoryCircleNode::timerCallback() {
    double now_sec = this->get_clock()->now().seconds();

    double radius = this->get_parameter("circle_radius").as_double();
    double omega = this->get_parameter("circle_omega").as_double();
    double height = this->get_parameter("circle_height").as_double();
    double center_x = this->get_parameter("circle_center_x").as_double();
    double center_y = this->get_parameter("circle_center_y").as_double();
    int steps = this->get_parameter("horizon_stages").as_int();
    double dt = this->get_parameter("control_period").as_double();
    std::string world_frame = this->get_parameter("world_frame").as_string();

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
        pose.pose.orientation.w = 1.0; // Default orientation
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

} // namespace trajectory_generator
