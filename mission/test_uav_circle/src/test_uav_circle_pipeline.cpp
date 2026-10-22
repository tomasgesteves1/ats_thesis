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
    double z_amplitude,
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
        pt.pz = height + z_amplitude * std::cos(omega * t);  // tilted circle when z_amplitude != 0

        pt.vx = -radius * omega * std::sin(omega * t);
        pt.vy = radius * omega * std::cos(omega * t);
        pt.vz = -z_amplitude * omega * std::sin(omega * t);

        points.push_back(pt);
    }

    return points;
}

} // namespace test_uav_circle
