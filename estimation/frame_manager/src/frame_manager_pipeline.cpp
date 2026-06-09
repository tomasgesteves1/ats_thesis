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

    // 1. Process Boat + Tether Anchor
    if (boat_odom) {
        process_vehicle(boat_odom, config_.boat_base_frame, config_.boat_z_offset, stamp, transforms);
        
        // Add Tether Anchor relative to Boat BaseLink
        geometry_msgs::msg::TransformStamped t_anchor;
        t_anchor.header.stamp = stamp;
        t_anchor.header.frame_id = config_.boat_base_frame;
        t_anchor.child_frame_id = "boat/tether_anchor";
        t_anchor.transform.translation.z = config_.boat_tether_z_offset;
        t_anchor.transform.rotation.w = 1.0;
        transforms.push_back(t_anchor);
    }

    // 2. Process Drone
    process_vehicle(drone_odom, config_.drone_base_frame, config_.drone_z_offset, stamp, transforms);

    return transforms;
}

void FrameManagerPipeline::process_vehicle(
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    const std::string& base_frame_name,
    double z_offset,
    const rclcpp::Time& stamp,
    std::vector<geometry_msgs::msg::TransformStamped>& out_transforms)
{
    if (!odom) return;

    // 1. Frame prefix (ex: "boat" or "drone")
    std::string prefix = base_frame_name.substr(0, base_frame_name.find("/"));
    std::string gt_frame = prefix + "/ground_truth";

    // 2. Transform: world -> vehicle/ground_truth (PURE GAZEBO)
    auto t_gt = geometry::odometry_to_transform(odom, config_.world_frame, gt_frame);
    t_gt.header.stamp = stamp;
    out_transforms.push_back(t_gt);

    // 3. Transform: vehicle/ground_truth -> vehicle/base_link (OFFSET)
    geometry_msgs::msg::TransformStamped t_base;
    t_base.header.stamp = stamp;
    t_base.header.frame_id = gt_frame;
    t_base.child_frame_id = base_frame_name;
    
    t_base.transform.translation.x = 0.0;
    t_base.transform.translation.y = 0.0;
    t_base.transform.translation.z = z_offset;
    
    t_base.transform.rotation.x = 0.0;
    t_base.transform.rotation.y = 0.0;
    t_base.transform.rotation.z = 0.0;
    t_base.transform.rotation.w = 1.0;

    out_transforms.push_back(t_base);
}

} // namespace frame_manager
