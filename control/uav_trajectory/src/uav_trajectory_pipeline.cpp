#include "uav_trajectory/uav_trajectory_pipeline.hpp"
#include <cmath>

namespace uav_trajectory {

std::vector<UavTrajectoryPoint> UavTrajectoryPipeline::generateCircle(
    double start_time,
    double radius,
    double omega,
    double height,
    double center_x,
    double center_y,
    int steps,
    double dt) const
{
    std::vector<UavTrajectoryPoint> points;
    points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        double t = start_time + i * dt;
        UavTrajectoryPoint pt;
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

std::vector<UavTrajectoryPoint> UavTrajectoryPipeline::generateBoatFollower(
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
    double dt) const
{
    (void)start_time; // Unused for relative predictions
    std::vector<UavTrajectoryPoint> points;
    points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        double elapsed = i * dt;
        UavTrajectoryPoint pt;
        
        // Predict future boat position
        double pred_boat_x = boat_x + boat_vx * elapsed;
        double pred_boat_y = boat_y + boat_vy * elapsed;
        double pred_boat_z = boat_z + boat_vz * elapsed;

        // UAV target position is boat position + offset
        pt.px = pred_boat_x + offset_x;
        pt.py = pred_boat_y + offset_y;
        pt.pz = pred_boat_z + offset_z;

        // UAV target velocity matches boat velocity (feedforward)
        pt.vx = boat_vx;
        pt.vy = boat_vy;
        pt.vz = boat_vz;

        points.push_back(pt);
    }

    return points;
}

} // namespace uav_trajectory
