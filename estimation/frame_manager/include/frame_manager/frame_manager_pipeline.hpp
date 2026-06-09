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
    std::string boat_base_frame;
    std::string drone_base_frame;
    
    // Explicit Mapping Offsets (Gazebo -> BaseLink)
    double boat_z_offset;
    double drone_z_offset;
    double boat_tether_z_offset;
};

class FrameManagerPipeline {
public:
    explicit FrameManagerPipeline(const PipelineConfig& config);

    /**
     * @brief Main processing logic. Creates the explicit mapping tree.
     * Tree: world -> vehicle/ground_truth -> vehicle/base_link
     */
    std::vector<geometry_msgs::msg::TransformStamped> run(
        const nav_msgs::msg::Odometry::SharedPtr& boat_odom,
        const nav_msgs::msg::Odometry::SharedPtr& drone_odom,
        const rclcpp::Time& stamp);

private:
    /**
     * @brief Creates the dynamic and static transforms for a vehicle.
     */
    void process_vehicle(
        const nav_msgs::msg::Odometry::SharedPtr& odom,
        const std::string& base_frame_name,
        double z_offset,
        const rclcpp::Time& stamp,
        std::vector<geometry_msgs::msg::TransformStamped>& out_transforms);

    PipelineConfig config_;
};

} // namespace frame_manager
