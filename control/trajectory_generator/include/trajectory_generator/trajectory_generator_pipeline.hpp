#pragma once

#include <vector>

namespace trajectory_generator {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class TrajectoryPipeline {
public:
    TrajectoryPipeline() = default;
    ~TrajectoryPipeline() = default;

    // Generate circular trajectory points starting from `start_time`
    std::vector<TrajectoryPoint> generateCircle(
        double start_time,
        double radius,
        double omega,
        double height,
        double center_x,
        double center_y,
        int steps,
        double dt) const;

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

} // namespace trajectory_generator
