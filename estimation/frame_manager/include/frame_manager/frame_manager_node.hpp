#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

#include "frame_manager/frame_manager_pipeline.hpp"

namespace frame_manager {

/**
 * @brief ROS 2 Node for managing coordinate frames and transformations.
 */
class FrameManagerNode : public rclcpp::Node {
public:
    FrameManagerNode();

private:
    /**
     * @brief Periodic task to process data and publish transforms.
     */
    void timer_callback();

    // ROS Infrastructure
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest data cache
    nav_msgs::msg::Odometry::SharedPtr last_boat_odom_;
    nav_msgs::msg::Odometry::SharedPtr last_drone_odom_;

    // Logic Pipeline
    std::unique_ptr<FrameManagerPipeline> pipeline_;
};

} // namespace frame_manager
