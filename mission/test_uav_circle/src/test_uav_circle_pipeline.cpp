#include "test_uav_circle/test_uav_circle_pipeline.hpp"
#include <cmath>

namespace test_uav_circle {

std::vector<TrajectoryPoint> TestUavCirclePipeline::generateCircle(
    double start_time,
    double radius,
    double omega,
    double height,
    double center_x,
    double center_y,
    int steps,
    double dt) const
{
    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        double t = start_time + i * dt;
        TrajectoryPoint pt;
        pt.px = center_x + radius * std::cos(omega * t);
        pt.py = center_y + radius * std::sin(omega * t);
        pt.pz = height;

        pt.vx = -radius * omega * std::sin(omega * t);
        pt.vy = radius * omega * std::cos(omega * t);
        pt.vz = 0.0;

        points.push_back(pt);
    }

    return points;
}

} // namespace test_uav_circle
