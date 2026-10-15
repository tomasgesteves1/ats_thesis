#include "intruder_sim/intruder_sim_node.hpp"

#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace intruder_sim
{

IntruderSimNode::IntruderSimNode(const rclcpp::NodeOptions & options) : Node("intruder_sim_node", options)
{
    IntruderConfig cfg;
    const auto flat = declare_parameter<std::vector<double>>("patrol", {60.0, 40.0, 70.0, 40.0});
    for (size_t i = 0; i + 1 < flat.size(); i += 2) {
        cfg.patrol.push_back({flat[i], flat[i + 1]});
    }
    cfg.patrol_speed = declare_parameter<double>("patrol_speed", 0.6);
    cfg.flee_speed = declare_parameter<double>("flee_speed", 1.0);
    cfg.acceleration = declare_parameter<double>("acceleration", 0.3);
    cfg.turn_rate = declare_parameter<double>("turn_rate", 0.3);
    cfg.waypoint_radius = declare_parameter<double>("waypoint_radius", 2.0);
    cfg.alert_radius = declare_parameter<double>("alert_radius", 10.0);
    cfg.capture_radius = declare_parameter<double>("capture_radius", 8.0);
    cfg.capture_time = declare_parameter<double>("capture_time", 3.0);
    const auto zone = declare_parameter<std::vector<double>>("zone", {0.0, 100.0, -50.0, 50.0});
    if (zone.size() == 4) {
        cfg.zone = {zone[0], zone[1], zone[2], zone[3]};
    }
    cfg.zone_margin = declare_parameter<double>("zone_margin", 8.0);
    entity_name_ = declare_parameter<std::string>("entity_name", "wamv_intruder");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    child_frame_ = declare_parameter<std::string>("child_frame", "intruder/base_link");
    z_ = declare_parameter<double>("z", -0.05);
    dt_ = 1.0 / declare_parameter<double>("rate_hz", 20.0);
    pipeline_ = std::make_unique<IntruderSimPipeline>(cfg);

    uav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "uav/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { uav_odom_ = msg; });
    usv_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "usv/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { usv_odom_ = msg; });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    set_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>("set_pose");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_timer(std::chrono::duration<double>(dt_), [this]() { loop(); });
    RCLCPP_INFO(get_logger(), "Intruder '%s' patrolling from (%.1f, %.1f)", entity_name_.c_str(),
                cfg.patrol[0][0], cfg.patrol[0][1]);
}

void IntruderSimNode::loop()
{
    if (!uav_odom_ || !usv_odom_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for the UAV and USV odometry...");
        return;
    }
    const auto & a = uav_odom_->pose.pose.position;
    const auto & b = usv_odom_->pose.pose.position;
    const auto event = pipeline_->step(dt_, {a.x, a.y}, {b.x, b.y});
    if (!event.empty()) {
        RCLCPP_INFO(get_logger(), "%s", event.c_str());
    }
    const auto & s = pipeline_->state();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = s.x;
    odom.pose.pose.position.y = s.y;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation.z = std::sin(0.5 * s.yaw);
    odom.pose.pose.orientation.w = std::cos(0.5 * s.yaw);
    odom.twist.twist.linear.x = s.speed;   // body frame (the intruder moves along its heading)
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = s.x;
    tf.transform.translation.y = s.y;
    tf.transform.translation.z = z_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    setGazeboPose(s);
}

void IntruderSimNode::setGazeboPose(const IntruderState & s)
{
    if (request_pending_ || !set_pose_client_->service_is_ready()) {
        return;
    }
    auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    req->entity.name = entity_name_;
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    req->pose.position.x = s.x;
    req->pose.position.y = s.y;
    req->pose.position.z = z_;
    req->pose.orientation.z = std::sin(0.5 * s.yaw);
    req->pose.orientation.w = std::cos(0.5 * s.yaw);
    request_pending_ = true;
    set_pose_client_->async_send_request(
        req, [this](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture) { request_pending_ = false; });
}

}  // namespace intruder_sim
