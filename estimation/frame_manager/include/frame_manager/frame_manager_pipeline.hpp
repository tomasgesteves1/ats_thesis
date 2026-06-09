#pragma once

#include <string>
#include <vector>
#include <array>

namespace frame_manager {

/**
 * @brief Pure C++ representation of Odometry data to decouple Pipeline from ROS messages.
 */
struct OdometryData {
    struct {
        double x;
        double y;
        double z;
    } position;

    struct {
        double x;
        double y;
        double z;
        double w;
    } orientation;
};

/**
 * @brief Pure C++ representation of a Transform.
 */
struct TransformData {
    std::string frame_id;
    std::string child_frame_id;
    double stamp_sec;
    
    struct {
        double x, y, z;
    } translation;
    
    struct {
        double x, y, z, w;
    } rotation;
};

/**
 * @brief Configuration parameters for the Frame Manager Pipeline.
 */
struct PipelineConfig {
    std::string world_frame;
    std::string boat_base_frame;
    std::string drone_base_frame;
    
    double boat_z_offset;
    double drone_z_offset;
    double boat_tether_z_offset;
};

class FrameManagerPipeline {
public:
    explicit FrameManagerPipeline(const PipelineConfig& config);

    /**
     * @brief Main processing logic. Pure C++ with no ROS dependencies.
     */
    std::vector<TransformData> run(
        const OdometryData* boat_odom,
        const OdometryData* drone_odom,
        double stamp_sec);

private:
    void process_vehicle(
        const OdometryData& odom,
        const std::string& base_frame_name,
        double z_offset,
        double stamp_sec,
        std::vector<TransformData>& out_transforms);

    PipelineConfig config_;
};

} // namespace frame_manager
