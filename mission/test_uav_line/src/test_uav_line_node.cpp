#include "test_uav_line/test_uav_line_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

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

    pipeline_ = std::make_unique<TestUavLinePipeline>();
    pipeline_->init(cfg);

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

void TestUavLineNode::timerCallback() {
    const double now_sec = this->get_clock()->now().seconds();

    if (start_time_ < 0.0) {
        start_time_ = now_sec;
    }

    const double elapsed = now_sec - start_time_;
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
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

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
