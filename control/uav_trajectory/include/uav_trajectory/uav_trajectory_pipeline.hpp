#pragma once

#include <vector>

namespace uav_trajectory {

struct UavTrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class UavTrajectoryPipeline {
public:
    UavTrajectoryPipeline() = default;
    ~UavTrajectoryPipeline() = default;

    // Generate circular trajectory points starting from `start_time`
    std::vector<UavTrajectoryPoint> generateCircle(
        double start_time,
        double radius,
        double omega,
        double height,
        double center_x,
        double center_y,
        int steps,
        double dt) const;

    // Generate boat follower trajectory points based on predicted future positions of the boat
    std::vector<UavTrajectoryPoint> generateBoatFollower(
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

} // namespace uav_trajectory
