#include "frame_manager/frame_manager_node.hpp"

namespace frame_manager {

FrameManagerNode::FrameManagerNode() : Node("frame_manager") {
    // Parameters
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("boat_frame", "boat/base_link");
    this->declare_parameter<std::string>("drone_frame", "drone/base_link");
    this->declare_parameter<std::string>("boat_odom_topic", "/boat/ground_truth/odometry");
    this->declare_parameter<std::string>("drone_odom_topic", "/drone/ground_truth/odometry");
    this->declare_parameter<double>("drone_z_offset", 0.265);
    this->declare_parameter<double>("update_rate_hz", 50.0);

    // Build pipeline config
    PipelineConfig config;
    config.world_frame = this->get_parameter("world_frame").as_string();
    config.boat_frame = this->get_parameter("boat_frame").as_string();
    config.drone_frame = this->get_parameter("drone_frame").as_string();
    config.drone_z_offset = this->get_parameter("drone_z_offset").as_double();

    pipeline_ = std::make_unique<FrameManagerPipeline>(config);

    // ROS Infrastructure
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    auto qos = rclcpp::SensorDataQoS();

    boat_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("boat_odom_topic").as_string(), qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { last_boat_odom_ = msg; });

    drone_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("drone_odom_topic").as_string(), qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { last_drone_odom_ = msg; });

    // Timer setup
    double hz = this->get_parameter("update_rate_hz").as_double();
    double period_sec = 1.0 / std::max(0.1, hz);
    
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period_sec),
        std::bind(&FrameManagerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Frame Manager: Initialized at %.2f Hz", hz);
}

void FrameManagerNode::timer_callback() {
    auto transforms = pipeline_->run(last_boat_odom_, last_drone_odom_, this->now());
    
    if (!transforms.empty()) {
        tf_broadcaster_->sendTransform(transforms);
    }
}

} // namespace frame_manager
