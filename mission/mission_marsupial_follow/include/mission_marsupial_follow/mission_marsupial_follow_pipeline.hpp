#pragma once

#include <vector>

namespace mission_marsupial_follow {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class MissionMarsupialFollowPipeline {
public:
    MissionMarsupialFollowPipeline() = default;
    ~MissionMarsupialFollowPipeline() = default;

    // Generate boat follower trajectory points based on predicted future positions of the boat
    std::vector<TrajectoryPoint> generateBoatFollower(
        double start_time,
        double boat_x,
        double boat_y,
        double boat_z,
        double boat_vx,
        double boat_vy,
        double boat_vz,
        double offset_x,
        double offset_y,
        double offset_z,
        int steps,
        double dt,
        bool predict_movement = true) const;
};

} // namespace mission_marsupial_follow
