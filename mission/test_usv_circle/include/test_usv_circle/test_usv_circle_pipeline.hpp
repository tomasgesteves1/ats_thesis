#pragma once

#include <vector>

namespace test_usv_circle {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class TestUsvCirclePipeline {
public:
    TestUsvCirclePipeline() = default;
    ~TestUsvCirclePipeline() = default;

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

} // namespace test_usv_circle
