#include "mission_executor/mission_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>

namespace mission_executor
{

namespace
{
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Odometry twist (child/body frame) expressed in the world frame.
mission_maneuvers::Vec3 worldVelocity(const nav_msgs::msg::Odometry & o)
{
    const auto & q = o.pose.pose.orientation;
    const auto & v = o.twist.twist.linear;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    return {(1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y - z * w) * v.y + 2 * (x * z + y * w) * v.z,
            2 * (x * y + z * w) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z - x * w) * v.z,
            2 * (x * z - y * w) * v.x + 2 * (y * z + x * w) * v.y + (1 - 2 * (x * x + y * y)) * v.z};
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}
}  // namespace

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
    : Node("mission_executor_node", options)
{
    const auto mission = declare_parameter<std::string>("mission", "landing_demo");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    anchor_frame_ = declare_parameter<std::string>("anchor_frame", "boat/tether_anchor");
    uav_frame_ = declare_parameter<std::string>("uav_frame", "drone/base_link");
    px4_target_system_ = static_cast<int>(declare_parameter<int64_t>("px4_target_system", 2));
    px4_command_period_ = declare_parameter<double>("px4_command_period", 1.0);
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);

    PipelineConfig config;
    config.uav_horizon.steps = static_cast<int>(declare_parameter<int64_t>("uav_horizon_steps", 51));
    config.uav_horizon.dt = declare_parameter<double>("uav_horizon_dt", 0.02);
    config.usv_horizon.steps = static_cast<int>(declare_parameter<int64_t>("usv_horizon_steps", 21));
    config.usv_horizon.dt = declare_parameter<double>("usv_horizon_dt", 0.1);
    config.horizon_timeout = declare_parameter<double>("horizon_timeout", 0.5);
    config.default_rest_height = declare_parameter<double>("default_rest_height", 0.26);
    pipeline_ = std::make_unique<MissionExecutorPipeline>(config);

    const auto path = resolveMissionPath(mission);
    try {
        std::string error;
        if (!pipeline_->load(loadMission(path), error)) {
            RCLCPP_ERROR(get_logger(), "Mission %s not loaded: %s", path.c_str(), error.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Mission '%s' loaded from %s", pipeline_->name().c_str(), path.c_str());
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Mission not loaded: %s", e.what());
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    uav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "uav/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { uav_odom_ = msg; });
    usv_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "usv/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { usv_odom_ = msg; });
    usv_horizon_sub_ = create_subscription<nav_msgs::msg::Path>(
        "usv/mpc_horizon", 10, [this](nav_msgs::msg::Path::SharedPtr msg) { usv_horizon_ = msg; });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "px4/vehicle_status", rclcpp::SensorDataQoS(),
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) { status_ = msg; });

    uav_path_pub_ = create_publisher<nav_msgs::msg::Path>("uav/reference_path", 10);
    usv_path_pub_ = create_publisher<nav_msgs::msg::Path>("usv/reference_path", 10);
    tether_pull_pub_ = create_publisher<std_msgs::msg::Bool>("tether/pull", 10);
    yaw_mode_pub_ = create_publisher<std_msgs::msg::Bool>("uav/yaw_from_reference", 10);
    events_pub_ = create_publisher<std_msgs::msg::String>("mission/events", 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("px4/vehicle_command", 10);

    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() { loop(); });
}

std::string MissionExecutorNode::resolveMissionPath(const std::string & mission) const
{
    if (std::filesystem::path(mission).is_absolute()) {
        return mission;
    }
    return ament_index_cpp::get_package_share_directory("mission_executor") + "/missions/" + mission + ".yaml";
}

bool MissionExecutorNode::gatherMeasurements(Measurements & m)
{
    if (!uav_odom_ || !usv_odom_ || !status_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry and PX4 status...");
        return false;
    }
    try {
        const auto a = tf_buffer_->lookupTransform(world_frame_, anchor_frame_, tf2::TimePointZero);
        const auto d = tf_buffer_->lookupTransform(world_frame_, uav_frame_, tf2::TimePointZero);
        m.boat.anchor = {a.transform.translation.x, a.transform.translation.y, a.transform.translation.z};
        m.uav.p = {d.transform.translation.x, d.transform.translation.y, d.transform.translation.z};
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for TF: %s", ex.what());
        return false;
    }
    m.t = now().seconds();
    m.uav.v = worldVelocity(*uav_odom_);
    m.uav.yaw = yawOf(uav_odom_->pose.pose.orientation);
    m.uav.armed = status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    m.uav.offboard = status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    const auto & bp = usv_odom_->pose.pose.position;
    m.boat.p = {bp.x, bp.y, bp.z};
    m.boat.v = worldVelocity(*usv_odom_);
    m.boat.yaw = yawOf(usv_odom_->pose.pose.orientation);
    if (usv_horizon_ && !usv_horizon_->poses.empty()) {
        m.boat.horizon.stamp = rclcpp::Time(usv_horizon_->header.stamp).seconds();
        m.boat.horizon.dt = get_parameter("usv_horizon_dt").as_double();
        m.boat.horizon.x_y_yaw.clear();
        for (const auto & p : usv_horizon_->poses) {
            m.boat.horizon.x_y_yaw.push_back({p.pose.position.x, p.pose.position.y, yawOf(p.pose.orientation)});
        }
    }
    return true;
}

void MissionExecutorNode::loop()
{
    Measurements m;
    if (!gatherMeasurements(m)) {
        return;
    }
    const auto out = pipeline_->tick(m);
    for (const auto & e : out.events) {
        RCLCPP_INFO(get_logger(), "%s", e.c_str());
        std_msgs::msg::String msg;
        msg.data = e;
        events_pub_->publish(msg);
    }
    if (pipeline_->state() == MissionState::kNotLoaded) {
        return;
    }
    publishCommands(out);
    sendPx4Commands(out.uav);
}

void MissionExecutorNode::publishCommands(const Output & out)
{
    const auto stamp = now();
    nav_msgs::msg::Path uav_path;
    uav_path.header.stamp = stamp;
    uav_path.header.frame_id = world_frame_;
    for (const auto & pose : out.uav.path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = uav_path.header;
        ps.pose.position.x = pose.p[0];
        ps.pose.position.y = pose.p[1];
        ps.pose.position.z = pose.p[2];
        ps.pose.orientation = yawQuaternion(pose.yaw);
        uav_path.poses.push_back(ps);
    }
    uav_path_pub_->publish(uav_path);

    nav_msgs::msg::Path usv_path;
    usv_path.header = uav_path.header;
    for (const auto & pose : out.usv.path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = usv_path.header;
        ps.pose.position.x = pose.x;
        ps.pose.position.y = pose.y;
        ps.pose.orientation = yawQuaternion(pose.yaw);
        usv_path.poses.push_back(ps);
    }
    usv_path_pub_->publish(usv_path);

    std_msgs::msg::Bool b;
    b.data = out.uav.tether_pull;
    tether_pull_pub_->publish(b);
    b.data = out.uav.yaw_from_reference;
    yaw_mode_pub_->publish(b);
}

void MissionExecutorNode::sendPx4Commands(const mission_maneuvers::UavCommand & cmd)
{
    if ((now() - last_px4_command_).seconds() < px4_command_period_) {
        return;
    }
    const bool armed = status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    bool sent = false;
    if (cmd.request_disarm && armed) {
        // forced disarm (magic number 21196 of MAV_CMD_COMPONENT_ARM_DISARM): the UAV is on the pad
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
        sent = true;
    } else {
        if (cmd.request_offboard && !offboard) {
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            sent = true;
        }
        if (cmd.request_arm && !armed && status_->pre_flight_checks_pass) {
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            sent = true;
        }
    }
    if (sent) {
        last_px4_command_ = now();
    }
}

void MissionExecutorNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param5 = std::nanf("");
    msg.param6 = std::nanf("");
    msg.target_system = static_cast<uint8_t>(px4_target_system_);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
}

}  // namespace mission_executor
