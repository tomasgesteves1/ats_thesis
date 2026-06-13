#pragma once

#include <vector>

namespace uav_mpc {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class UavMpcTrajectory {
public:
    UavMpcTrajectory();

    // Configure circular trajectory parameters
    void configureCircle(double radius, double omega, double height, double center_x = 0.0, double center_y = 0.0);

    // Get trajectory reference point at time t
    TrajectoryPoint getPoint(double t) const;

    // Get full reference path for visualization
    std::vector<TrajectoryPoint> getReferencePath() const;

private:
    double radius_;
    double omega_;
    double height_;
    double center_x_;
    double center_y_;
};

} // namespace uav_mpc
