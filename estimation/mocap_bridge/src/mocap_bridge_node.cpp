#include "mocap_bridge/mocap_bridge_node.hpp"

namespace mocap_bridge {

MocapBridgeNode::MocapBridgeNode() : Node("mocap_bridge") {
    // 1. Declare Parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<std::string>("odom_topic", "/drone/ground_truth/odometry");
    this->declare_parameter<std::string>("px4_vo_topic", "/px4_1/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<double>("position_variance", 0.001);
    this->declare_parameter<double>("orientation_variance", 0.0001);
    this->declare_parameter<double>("velocity_variance", 0.001);

    this->declare_parameter<std::string>("px4_command_topic", "/px4_1/fmu/in/vehicle_command");
    this->declare_parameter<bool>("set_global_origin", true);
    this->declare_parameter<double>("origin_latitude", 42.35821841111111);
    this->declare_parameter<double>("origin_longitude", -71.0479235555555);
    this->declare_parameter<double>("origin_altitude", 0.0);
    this->declare_parameter<int>("target_system", 2);

    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string px4_vo_topic = this->get_parameter("px4_vo_topic").as_string();
    std::string px4_command_topic = this->get_parameter("px4_command_topic").as_string();
    set_global_origin_ = this->get_parameter("set_global_origin").as_bool();
    origin_lat_ = this->get_parameter("origin_latitude").as_double();
    origin_lon_ = this->get_parameter("origin_longitude").as_double();
    origin_alt_ = this->get_parameter("origin_altitude").as_double();
    target_system_ = this->get_parameter("target_system").as_int();

    BridgeConfig config;
    config.pos_variance = static_cast<float>(this->get_parameter("position_variance").as_double());
    config.ang_variance = static_cast<float>(this->get_parameter("orientation_variance").as_double());
    config.vel_variance = static_cast<float>(this->get_parameter("velocity_variance").as_double());

    pipeline_ = std::make_unique<MocapBridgePipeline>(config);

    // 2. ROS Infrastructure
    auto qos = rclcpp::SensorDataQoS();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos,
        std::bind(&MocapBridgeNode::odomCallback, this, std::placeholders::_1));

    px4_vo_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_vo_topic, qos);

    px4_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_command_topic, 10);

    if (set_global_origin_) {
        origin_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MocapBridgeNode::publishOriginCommand, this));
    }

    RCLCPP_INFO(this->get_logger(),
                "MoCap Bridge initialized: [%s] -> [%s], command: [%s]",
                odom_topic.c_str(), px4_vo_topic.c_str(), px4_command_topic.c_str());
}

void MocapBridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematics::EnuOdometry enu{};
    enu.position = {msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z};

    enu.orientation = {msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w};

    enu.linear_vel = {msg->twist.twist.linear.x,
                      msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z};

    enu.angular_vel = {msg->twist.twist.angular.x,
                       msg->twist.twist.angular.y,
                       msg->twist.twist.angular.z};

    kinematics::NedOdometry ned = pipeline_->process(enu);

    px4_msgs::msg::VehicleOdometry vo_msg{};
    const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;
    vo_msg.timestamp = now_us;
    vo_msg.timestamp_sample = now_us;

    vo_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    vo_msg.position[0] = ned.position[0];
    vo_msg.position[1] = ned.position[1];
    vo_msg.position[2] = ned.position[2];

    vo_msg.q[0] = ned.orientation[0];
    vo_msg.q[1] = ned.orientation[1];
    vo_msg.q[2] = ned.orientation[2];
    vo_msg.q[3] = ned.orientation[3];

    vo_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    vo_msg.velocity[0] = ned.linear_vel[0];
    vo_msg.velocity[1] = ned.linear_vel[1];
    vo_msg.velocity[2] = ned.linear_vel[2];

    vo_msg.angular_velocity[0] = ned.angular_vel[0];
    vo_msg.angular_velocity[1] = ned.angular_vel[1];
    vo_msg.angular_velocity[2] = ned.angular_vel[2];

    const auto& cfg = pipeline_->getConfig();
    vo_msg.position_variance[0] = cfg.pos_variance;
    vo_msg.position_variance[1] = cfg.pos_variance;
    vo_msg.position_variance[2] = cfg.pos_variance;

    vo_msg.orientation_variance[0] = cfg.ang_variance;
    vo_msg.orientation_variance[1] = cfg.ang_variance;
    vo_msg.orientation_variance[2] = cfg.ang_variance;

    vo_msg.velocity_variance[0] = cfg.vel_variance;
    vo_msg.velocity_variance[1] = cfg.vel_variance;
    vo_msg.velocity_variance[2] = cfg.vel_variance;

    vo_msg.reset_counter = 0;
    vo_msg.quality = 100;

    px4_vo_pub_->publish(vo_msg);
}

void MocapBridgeNode::publishOriginCommand() {
    if (!set_global_origin_ || origin_count_ >= 10) {
        if (origin_timer_) {
            origin_timer_->cancel();
        }
        return;
    }
    origin_count_++;

    px4_msgs::msg::VehicleCommand cmd{};
    const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;
    cmd.timestamp = now_us;
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
    cmd.param5 = origin_lat_;
    cmd.param6 = origin_lon_;
    cmd.param7 = static_cast<float>(origin_alt_);
    cmd.target_system = static_cast<uint8_t>(target_system_);
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    px4_cmd_pub_->publish(cmd);

    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Broadcasting EKF Global Origin (lat=%.7f, lon=%.7f, alt=%.2f) to target_system=%d",
                     origin_lat_, origin_lon_, origin_alt_, target_system_);
}

} // namespace mocap_bridge
