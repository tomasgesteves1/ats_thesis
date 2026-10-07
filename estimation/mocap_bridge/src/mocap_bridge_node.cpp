#include "mocap_bridge/mocap_bridge_node.hpp"

namespace mocap_bridge {

MocapBridgeNode::MocapBridgeNode() : Node("mocap_bridge") {
    // 1. Declare Parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<std::string>("odom_topic", "/drone/ground_truth/odometry");
    this->declare_parameter<std::string>("px4_vo_topic", "/px4_1/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("px4_odom_topic", "/px4_1/fmu/out/vehicle_odometry");
    this->declare_parameter<bool>("publish_debug_topics", true);
    this->declare_parameter<bool>("euler_in_degrees", false);

    this->declare_parameter<double>("position_variance", 0.001);
    this->declare_parameter<double>("orientation_variance", 0.0001);
    this->declare_parameter<double>("velocity_variance", 0.001);

    this->declare_parameter<std::string>("px4_command_topic", "/px4_1/fmu/in/vehicle_command");
    this->declare_parameter<bool>("set_global_origin", true);
    this->declare_parameter<double>("origin_latitude", 42.35821841111111);
    this->declare_parameter<double>("origin_longitude", -71.0479235555555);
    this->declare_parameter<double>("origin_altitude", 0.0);
    this->declare_parameter<int>("target_system", 2);

    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string px4_vo_topic = this->get_parameter("px4_vo_topic").as_string();
    const std::string px4_odom_topic = this->get_parameter("px4_odom_topic").as_string();
    const std::string px4_command_topic = this->get_parameter("px4_command_topic").as_string();

    publish_debug_topics_ = this->get_parameter("publish_debug_topics").as_bool();
    euler_in_degrees_ = this->get_parameter("euler_in_degrees").as_bool();
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

    if (publish_debug_topics_) {
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_odom_topic, qos,
            std::bind(&MocapBridgeNode::px4OdomCallback, this, std::placeholders::_1));

        // Full Odometry in ENU for 3D Visualizer
        px4_odom_enu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "drone/px4_estimated/odometry", qos);

        // Ground Truth decomposed topics (Vector3Stamped)
        gt_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/ground_truth/position", qos);
        gt_euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/ground_truth/euler", qos);
        gt_lin_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/ground_truth/linear_velocity", qos);
        gt_ang_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/ground_truth/angular_velocity", qos);

        // PX4 Estimated decomposed topics (Vector3Stamped)
        px4_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/px4_estimated/position", qos);
        px4_euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/px4_estimated/euler", qos);
        px4_lin_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/px4_estimated/linear_velocity", qos);
        px4_ang_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/px4_estimated/angular_velocity", qos);

        // Direct Error topics (Vector3Stamped)
        err_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/debug/error/position", qos);
        err_euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/debug/error/euler", qos);
        err_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/debug/error/linear_velocity", qos);
        err_ang_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "drone/debug/error/angular_velocity", qos);
    }

    if (set_global_origin_) {
        origin_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MocapBridgeNode::publishOriginCommand, this));
    }

    RCLCPP_INFO(this->get_logger(),
                "MoCap Bridge initialized: [%s] -> [%s], feedback: [%s], debug: %s",
                odom_topic.c_str(), px4_vo_topic.c_str(), px4_odom_topic.c_str(),
                publish_debug_topics_ ? "ENABLED" : "DISABLED");
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

    last_gt_enu_ = enu;
    has_gt_ = true;

    kinematics::NedOdometry ned = pipeline_->processEnuToNed(enu);

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

    // Publish GT decomposed topics for easy Foxglove plotting
    if (publish_debug_topics_) {
        if (gt_pos_pub_) {
            geometry_msgs::msg::Vector3Stamped pos_msg;
            pos_msg.header = msg->header;
            pos_msg.vector.x = enu.position[0];
            pos_msg.vector.y = enu.position[1];
            pos_msg.vector.z = enu.position[2];
            gt_pos_pub_->publish(pos_msg);
        }

        if (gt_euler_pub_) {
            const kinematics::EulerAngles euler = pipeline_->computeEuler(enu.orientation);
            const double scale = euler_in_degrees_ ? (180.0 / M_PI) : 1.0;
            geometry_msgs::msg::Vector3Stamped euler_msg;
            euler_msg.header = msg->header;
            euler_msg.vector.x = euler.roll * scale;
            euler_msg.vector.y = euler.pitch * scale;
            euler_msg.vector.z = euler.yaw * scale;
            gt_euler_pub_->publish(euler_msg);
        }

        if (gt_lin_vel_pub_) {
            geometry_msgs::msg::Vector3Stamped lin_vel_msg;
            lin_vel_msg.header = msg->header;
            lin_vel_msg.header.frame_id = "drone/base_link";
            lin_vel_msg.vector.x = enu.linear_vel[0];
            lin_vel_msg.vector.y = enu.linear_vel[1];
            lin_vel_msg.vector.z = enu.linear_vel[2];
            gt_lin_vel_pub_->publish(lin_vel_msg);
        }

        if (gt_ang_vel_pub_) {
            geometry_msgs::msg::Vector3Stamped ang_vel_msg;
            ang_vel_msg.header = msg->header;
            ang_vel_msg.header.frame_id = "drone/base_link";
            ang_vel_msg.vector.x = enu.angular_vel[0];
            ang_vel_msg.vector.y = enu.angular_vel[1];
            ang_vel_msg.vector.z = enu.angular_vel[2];
            gt_ang_vel_pub_->publish(ang_vel_msg);
        }
    }
}

void MocapBridgeNode::px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    if (!publish_debug_topics_) {
        return;
    }

    kinematics::NedOdometry ned{};
    ned.position = {msg->position[0], msg->position[1], msg->position[2]};
    ned.orientation = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
    ned.linear_vel = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    ned.angular_vel = {msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]};
    ned.velocity_frame = msg->velocity_frame;

    const kinematics::EnuOdometry est_enu = pipeline_->processNedToEnu(ned);
    const auto now = this->get_clock()->now();

    // 1. Publish estimated state converted to ENU odometry (for 3D visualizers & plotters)
    if (px4_odom_enu_pub_) {
        nav_msgs::msg::Odometry odom_enu_msg;
        odom_enu_msg.header.stamp = now;
        odom_enu_msg.header.frame_id = "world";
        odom_enu_msg.child_frame_id = "drone/base_link";

        odom_enu_msg.pose.pose.position.x = est_enu.position[0];
        odom_enu_msg.pose.pose.position.y = est_enu.position[1];
        odom_enu_msg.pose.pose.position.z = est_enu.position[2];

        odom_enu_msg.pose.pose.orientation.x = est_enu.orientation[0];
        odom_enu_msg.pose.pose.orientation.y = est_enu.orientation[1];
        odom_enu_msg.pose.pose.orientation.z = est_enu.orientation[2];
        odom_enu_msg.pose.pose.orientation.w = est_enu.orientation[3];

        odom_enu_msg.twist.twist.linear.x = est_enu.linear_vel[0];
        odom_enu_msg.twist.twist.linear.y = est_enu.linear_vel[1];
        odom_enu_msg.twist.twist.linear.z = est_enu.linear_vel[2];

        odom_enu_msg.twist.twist.angular.x = est_enu.angular_vel[0];
        odom_enu_msg.twist.twist.angular.y = est_enu.angular_vel[1];
        odom_enu_msg.twist.twist.angular.z = est_enu.angular_vel[2];

        px4_odom_enu_pub_->publish(odom_enu_msg);
    }

    // 2. Publish decomposed estimated state for easy Foxglove plotting
    if (px4_pos_pub_) {
        geometry_msgs::msg::Vector3Stamped pos_msg;
        pos_msg.header.stamp = now;
        pos_msg.header.frame_id = "world";
        pos_msg.vector.x = est_enu.position[0];
        pos_msg.vector.y = est_enu.position[1];
        pos_msg.vector.z = est_enu.position[2];
        px4_pos_pub_->publish(pos_msg);
    }

    const double scale = euler_in_degrees_ ? (180.0 / M_PI) : 1.0;
    if (px4_euler_pub_) {
        const kinematics::EulerAngles est_euler = pipeline_->computeEuler(est_enu.orientation);
        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp = now;
        euler_msg.header.frame_id = "world";
        euler_msg.vector.x = est_euler.roll * scale;
        euler_msg.vector.y = est_euler.pitch * scale;
        euler_msg.vector.z = est_euler.yaw * scale;
        px4_euler_pub_->publish(euler_msg);
    }

    if (px4_lin_vel_pub_) {
        geometry_msgs::msg::Vector3Stamped lin_vel_msg;
        lin_vel_msg.header.stamp = now;
        lin_vel_msg.header.frame_id = "drone/base_link";
        lin_vel_msg.vector.x = est_enu.linear_vel[0];
        lin_vel_msg.vector.y = est_enu.linear_vel[1];
        lin_vel_msg.vector.z = est_enu.linear_vel[2];
        px4_lin_vel_pub_->publish(lin_vel_msg);
    }

    if (px4_ang_vel_pub_) {
        geometry_msgs::msg::Vector3Stamped ang_vel_msg;
        ang_vel_msg.header.stamp = now;
        ang_vel_msg.header.frame_id = "drone/base_link";
        ang_vel_msg.vector.x = est_enu.angular_vel[0];
        ang_vel_msg.vector.y = est_enu.angular_vel[1];
        ang_vel_msg.vector.z = est_enu.angular_vel[2];
        px4_ang_vel_pub_->publish(ang_vel_msg);
    }

    // 3. Compute and publish error metrics if ground truth is available
    if (has_gt_) {
        const kinematics::OdometryError err = pipeline_->computeError(last_gt_enu_, est_enu);

        if (err_pos_pub_) {
            geometry_msgs::msg::Vector3Stamped pos_err_msg;
            pos_err_msg.header.stamp = now;
            pos_err_msg.header.frame_id = "world";
            pos_err_msg.vector.x = err.position_error[0];
            pos_err_msg.vector.y = err.position_error[1];
            pos_err_msg.vector.z = err.position_error[2];
            err_pos_pub_->publish(pos_err_msg);
        }

        if (err_euler_pub_) {
            geometry_msgs::msg::Vector3Stamped att_err_msg;
            att_err_msg.header.stamp = now;
            att_err_msg.header.frame_id = "world";
            att_err_msg.vector.x = err.attitude_error.roll * scale;
            att_err_msg.vector.y = err.attitude_error.pitch * scale;
            att_err_msg.vector.z = err.attitude_error.yaw * scale;
            err_euler_pub_->publish(att_err_msg);
        }

        if (err_vel_pub_) {
            geometry_msgs::msg::Vector3Stamped vel_err_msg;
            vel_err_msg.header.stamp = now;
            vel_err_msg.header.frame_id = "drone/base_link";
            vel_err_msg.vector.x = err.linear_vel_error[0];
            vel_err_msg.vector.y = err.linear_vel_error[1];
            vel_err_msg.vector.z = err.linear_vel_error[2];
            err_vel_pub_->publish(vel_err_msg);
        }

        if (err_ang_vel_pub_) {
            geometry_msgs::msg::Vector3Stamped ang_err_msg;
            ang_err_msg.header.stamp = now;
            ang_err_msg.header.frame_id = "drone/base_link";
            ang_err_msg.vector.x = err.angular_vel_error[0];
            ang_err_msg.vector.y = err.angular_vel_error[1];
            ang_err_msg.vector.z = err.angular_vel_error[2];
            err_ang_vel_pub_->publish(ang_err_msg);
        }
    }
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
