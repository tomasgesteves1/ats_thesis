#include "frame_manager/frame_manager_pipeline.hpp"
#include "frame_manager/frame_manager_geometry.hpp"

namespace frame_manager {

FrameManagerPipeline::FrameManagerPipeline(const PipelineConfig& config) 
    : config_(config) {}

std::vector<geometry_msgs::msg::TransformStamped> FrameManagerPipeline::run(
    const nav_msgs::msg::Odometry::SharedPtr& boat_odom,
    const nav_msgs::msg::Odometry::SharedPtr& drone_odom,
    const rclcpp::Time& stamp)
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    if (boat_odom) {
        transforms.push_back(process_boat(boat_odom, stamp));
    }

    if (drone_odom) {
        transforms.push_back(process_drone(drone_odom, stamp));
    }

    return transforms;
}

geometry_msgs::msg::TransformStamped FrameManagerPipeline::process_boat(
    const nav_msgs::msg::Odometry::SharedPtr& msg, const rclcpp::Time& stamp) 
{
    auto t = geometry::odometry_to_transform(msg, config_.world_frame, config_.boat_frame);
    t.header.stamp = stamp;
    return t;
}

geometry_msgs::msg::TransformStamped FrameManagerPipeline::process_drone(
    const nav_msgs::msg::Odometry::SharedPtr& msg, const rclcpp::Time& stamp) 
{
    auto t = geometry::odometry_to_transform(msg, config_.world_frame, config_.drone_frame, config_.drone_z_offset);
    t.header.stamp = stamp;
    return t;
}

} // namespace frame_manager
