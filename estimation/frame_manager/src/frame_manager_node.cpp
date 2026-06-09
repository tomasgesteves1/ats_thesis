#include "frame_manager/frame_manager_node.hpp"

namespace frame_manager {

FrameManagerNode::FrameManagerNode() : Node("frame_manager") {
    // Parameters
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("boat_frame", "boat/base_link");
    this->declare_parameter<std::string>("drone_frame", "drone/base_link");
    this->declare_parameter<std::string>("boat_odom_topic", "boat/ground_truth/odometry");
    this->declare_parameter<std::string>("drone_odom_topic", "drone/ground_truth/odometry");
    this->declare_parameter<double>("boat_z_offset", 0.0);
    this->declare_parameter<double>("drone_z_offset", 0.265);
    this->declare_parameter<double>("boat_tether_z_offset", 1.3);
    this->declare_parameter<double>("update_rate_hz", 50.0);

    // Build pipeline config
    PipelineConfig config;
    config.world_frame = this->get_parameter("world_frame").as_string();
    config.boat_base_frame = this->get_parameter("boat_frame").as_string();
    config.drone_base_frame = this->get_parameter("drone_frame").as_string();
    config.boat_z_offset = this->get_parameter("boat_z_offset").as_double();
    config.drone_z_offset = this->get_parameter("drone_z_offset").as_double();
    config.boat_tether_z_offset = this->get_parameter("boat_tether_z_offset").as_double();

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

    RCLCPP_INFO(this->get_logger(), "Frame Manager (Gold Standard) initialized at %.2f Hz", hz);
}

void FrameManagerNode::timer_callback() {
    // 1. Convert ROS messages to Pure C++ structs
    std::unique_ptr<OdometryData> boat_data = nullptr;
    if (last_boat_odom_) {
        boat_data = std::make_unique<OdometryData>();
        boat_data->position = {last_boat_odom_->pose.pose.position.x, 
                               last_boat_odom_->pose.pose.position.y, 
                               last_boat_odom_->pose.pose.position.z};
        boat_data->orientation = {last_boat_odom_->pose.pose.orientation.x,
                                  last_boat_odom_->pose.pose.orientation.y,
                                  last_boat_odom_->pose.pose.orientation.z,
                                  last_boat_odom_->pose.pose.orientation.w};
    }

    std::unique_ptr<OdometryData> drone_data = nullptr;
    if (last_drone_odom_) {
        drone_data = std::make_unique<OdometryData>();
        drone_data->position = {last_drone_odom_->pose.pose.position.x, 
                                last_drone_odom_->pose.pose.position.y, 
                                last_drone_odom_->pose.pose.position.z};
        drone_data->orientation = {last_drone_odom_->pose.pose.orientation.x,
                                   last_drone_odom_->pose.pose.orientation.y,
                                   last_drone_odom_->pose.pose.orientation.z,
                                   last_drone_odom_->pose.pose.orientation.w};
    }

    // 2. Run Pure C++ Pipeline
    double now_sec = this->now().seconds();
    auto results = pipeline_->run(boat_data.get(), drone_data.get(), now_sec);

    // 3. Convert results back to ROS messages for broadcasting
    if (!results.empty()) {
        std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
        for (const auto& res : results) {
            geometry_msgs::msg::TransformStamped m;
            m.header.stamp = rclcpp::Time(static_cast<int64_t>(res.stamp_sec * 1e9), RCL_ROS_TIME);
            m.header.frame_id = res.frame_id;
            m.child_frame_id = res.child_frame_id;
            m.transform.translation.x = res.translation.x;
            m.transform.translation.y = res.translation.y;
            m.transform.translation.z = res.translation.z;
            m.transform.rotation.x = res.rotation.x;
            m.transform.rotation.y = res.rotation.y;
            m.transform.rotation.z = res.rotation.z;
            m.transform.rotation.w = res.rotation.w;
            tf_msgs.push_back(m);
        }
        tf_broadcaster_->sendTransform(tf_msgs);
    }
}

} // namespace frame_manager
