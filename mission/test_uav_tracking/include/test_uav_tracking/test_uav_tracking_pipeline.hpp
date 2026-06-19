#pragma once

#include <vector>

namespace test_uav_tracking {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class TestUavTrackingPipeline {
public:
    TestUavTrackingPipeline() = default;
    ~TestUavTrackingPipeline() = default;

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

} // namespace test_uav_tracking
