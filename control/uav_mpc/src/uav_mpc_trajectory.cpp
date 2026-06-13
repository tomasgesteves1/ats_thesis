#include "uav_mpc/uav_mpc_trajectory.hpp"
#include <cmath>

namespace uav_mpc {

UavMpcTrajectory::UavMpcTrajectory()
    : radius_(3.0), omega_(0.2), height_(10.0), center_x_(0.0), center_y_(0.0) {}

void UavMpcTrajectory::configureCircle(double radius, double omega, double height, double center_x, double center_y) {
    radius_ = radius;
    omega_ = omega;
    height_ = height;
    center_x_ = center_x;
    center_y_ = center_y;
}

TrajectoryPoint UavMpcTrajectory::getPoint(double t) const {
    TrajectoryPoint pt;
    pt.px = center_x_ + radius_ * std::cos(omega_ * t);
    pt.py = center_y_ + radius_ * std::sin(omega_ * t);
    pt.pz = height_;

    pt.vx = -radius_ * omega_ * std::sin(omega_ * t);
    pt.vy = radius_ * omega_ * std::cos(omega_ * t);
    pt.vz = 0.0;

    return pt;
}

std::vector<TrajectoryPoint> UavMpcTrajectory::getReferencePath() const {
    std::vector<TrajectoryPoint> path;
    int num_points = 100;
    double pi = 3.14159265358979323846;
    for (int i = 0; i <= num_points; ++i) {
        double theta = (2.0 * pi * i) / num_points;
        TrajectoryPoint pt;
        pt.px = center_x_ + radius_ * std::cos(theta);
        pt.py = center_y_ + radius_ * std::sin(theta);
        pt.pz = height_;
        pt.vx = 0.0;
        pt.vy = 0.0;
        pt.vz = 0.0;
        path.push_back(pt);
    }
    return path;
}

} // namespace uav_mpc
