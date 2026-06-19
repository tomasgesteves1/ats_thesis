#include "test_uav_tracking/test_uav_tracking_pipeline.hpp"
#include <cmath>

namespace test_uav_tracking {

std::vector<TrajectoryPoint> TestUavTrackingPipeline::generateBoatFollower(
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
    bool predict_movement) const
{
    (void)start_time; // Unused for relative predictions
    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        TrajectoryPoint pt;
        
        if (predict_movement) {
            double elapsed = i * dt;
            // Predict future boat position
            double pred_boat_x = boat_x + boat_vx * elapsed;
            double pred_boat_y = boat_y + boat_vy * elapsed;
            double pred_boat_z = boat_z + boat_vz * elapsed;

            // UAV target position is predicted boat position + offset
            pt.px = pred_boat_x + offset_x;
            pt.py = pred_boat_y + offset_y;
            pt.pz = pred_boat_z + offset_z;

            // UAV target velocity matches boat velocity (feedforward prediction)
            pt.vx = boat_vx;
            pt.vy = boat_vy;
            pt.vz = boat_vz;
        } else {
            // Static setpoint repeated throughout the horizon
            pt.px = boat_x + offset_x;
            pt.py = boat_y + offset_y;
            pt.pz = boat_z + offset_z;

            // Zero velocity reference to encourage faster dynamic catch-up behavior
            pt.vx = 0.0;
            pt.vy = 0.0;
            pt.vz = 0.0;
        }

        points.push_back(pt);
    }

    return points;
}

} // namespace test_uav_tracking
