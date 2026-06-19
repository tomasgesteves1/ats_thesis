#pragma once

#include <vector>

namespace mission_usv_circle {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class MissionUsvCirclePipeline {
public:
    MissionUsvCirclePipeline() = default;
    ~MissionUsvCirclePipeline() = default;

    // Generate circular trajectory points starting from start_time
    std::vector<TrajectoryPoint> generateCircle(
        double start_time,
        double radius,
        double omega,
        double height,
        double center_x,
        double center_y,
        int steps,
        double dt) const;
};

} // namespace mission_usv_circle
