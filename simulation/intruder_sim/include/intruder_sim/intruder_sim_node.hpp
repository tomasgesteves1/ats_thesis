#ifndef INTRUDER_SIM__INTRUDER_SIM_NODE_HPP_
#define INTRUDER_SIM__INTRUDER_SIM_NODE_HPP_

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "intruder_sim/intruder_sim_pipeline.hpp"

namespace intruder_sim
{

/// ROS wrapper of the intruder: reads the pursuers' odometry, advances the behaviour at a fixed rate, moves the
/// static Gazebo model through the set_pose service and publishes its odometry and TF (ground truth).
class IntruderSimNode : public rclcpp::Node
{
public:
    explicit IntruderSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void loop();
    void setGazeboPose(const IntruderState & s);

    std::unique_ptr<IntruderSimPipeline> pipeline_;
    std::string entity_name_;
    std::string world_frame_;
    std::string child_frame_;
    double z_{0.0};
    double dt_{0.05};

    nav_msgs::msg::Odometry::SharedPtr uav_odom_;
    nav_msgs::msg::Odometry::SharedPtr usv_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uav_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr usv_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool request_pending_{false};
};

}  // namespace intruder_sim

#endif  // INTRUDER_SIM__INTRUDER_SIM_NODE_HPP_
