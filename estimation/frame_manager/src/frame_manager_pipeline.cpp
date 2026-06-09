#include "frame_manager/frame_manager_pipeline.hpp"

namespace frame_manager {

FrameManagerPipeline::FrameManagerPipeline(const PipelineConfig& config) 
    : config_(config) {}

std::vector<TransformData> FrameManagerPipeline::run(
    const OdometryData* boat_odom,
    const OdometryData* drone_odom,
    double stamp_sec)
{
    std::vector<TransformData> transforms;

    if (boat_odom) {
        process_vehicle(*boat_odom, config_.boat_base_frame, config_.boat_z_offset, stamp_sec, transforms);
        
        // Tether Anchor
        TransformData t_anchor;
        t_anchor.stamp_sec = stamp_sec;
        t_anchor.frame_id = config_.boat_base_frame;
        t_anchor.child_frame_id = "boat/tether_anchor";
        t_anchor.translation = {0.0, 0.0, config_.boat_tether_z_offset};
        t_anchor.rotation = {0.0, 0.0, 0.0, 1.0};
        transforms.push_back(t_anchor);
    }

    if (drone_odom) {
        process_vehicle(*drone_odom, config_.drone_base_frame, config_.drone_z_offset, stamp_sec, transforms);
    }

    return transforms;
}

void FrameManagerPipeline::process_vehicle(
    const OdometryData& odom,
    const std::string& base_frame_name,
    double z_offset,
    double stamp_sec,
    std::vector<TransformData>& out_transforms)
{
    std::string prefix = base_frame_name.substr(0, base_frame_name.find("/"));
    std::string gt_frame = prefix + "/ground_truth";

    // world -> vehicle/ground_truth
    TransformData t_gt;
    t_gt.stamp_sec = stamp_sec;
    t_gt.frame_id = config_.world_frame;
    t_gt.child_frame_id = gt_frame;
    t_gt.translation = {odom.position.x, odom.position.y, odom.position.z};
    t_gt.rotation = {odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w};
    out_transforms.push_back(t_gt);

    // vehicle/ground_truth -> vehicle/base_link
    TransformData t_base;
    t_base.stamp_sec = stamp_sec;
    t_base.frame_id = gt_frame;
    t_base.child_frame_id = base_frame_name;
    t_base.translation = {0.0, 0.0, z_offset};
    t_base.rotation = {0.0, 0.0, 0.0, 1.0};
    out_transforms.push_back(t_base);
}

} // namespace frame_manager
