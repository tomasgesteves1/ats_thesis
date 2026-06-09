#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>

namespace frame_manager {

/**
 * @brief Configuration parameters for the Frame Manager Pipeline.
 */
struct PipelineConfig {
    std::string world_frame;
    std::string boat_frame;
    std::string drone_frame;
    double drone_z_offset;
};

class FrameManagerPipeline {
public:
    explicit FrameManagerPipeline(const PipelineConfig& config);

    /**
     * @brief Main processing logic. Transforms odometry data into a vector of transforms.
     * @param boat_odom Latest boat odometry.
     * @param drone_odom Latest drone odometry.
     * @param stamp Timestamp to apply to all transforms.
     * @return Vector of ready-to-publish transforms.
     */
    std::vector<geometry_msgs::msg::TransformStamped> run(
        const nav_msgs::msg::Odometry::SharedPtr& boat_odom,
        const nav_msgs::msg::Odometry::SharedPtr& drone_odom,
        const rclcpp::Time& stamp);

private:
    geometry_msgs::msg::TransformStamped process_boat(const nav_msgs::msg::Odometry::SharedPtr& msg, const rclcpp::Time& stamp);
    geometry_msgs::msg::TransformStamped process_drone(const nav_msgs::msg::Odometry::SharedPtr& msg, const rclcpp::Time& stamp);

    PipelineConfig config_;
};

} // namespace frame_manager
