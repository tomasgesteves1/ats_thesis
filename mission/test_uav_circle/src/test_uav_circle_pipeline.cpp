#include "test_uav_circle/test_uav_circle_pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

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

bool TestUavCirclePipeline::loadTable(const std::string& path)
{
    std::ifstream file(path);
    if (!file) return false;
    std::string line;
    std::getline(file, line);  // header
    std::vector<double> times;
    table_.clear();
    while (std::getline(file, line)) {
        std::stringstream cols(line);
        std::string col;
        double v[7];
        int k = 0;
        while (k < 7 && std::getline(cols, col, ',')) v[k++] = std::stod(col);
        if (k < 7) continue;
        times.push_back(v[0]);
        table_.push_back(TrajectoryPoint{v[1], v[2], v[3], v[4], v[5], v[6]});
    }
    if (table_.size() < 2) {
        table_.clear();
        return false;
    }
    table_dt_ = times[1] - times[0];
    return true;
}

TrajectoryPoint TestUavCirclePipeline::interpolate(double tau) const
{
    const double period = table_dt_ * table_.size();
    if (!loop_ && tau >= table_dt_ * (table_.size() - 1)) {
        const auto& last = table_.back();
        return TrajectoryPoint{last.px, last.py, last.pz, 0.0, 0.0, 0.0};
    }
    double s = std::fmod(tau, period);
    if (s < 0.0) s += period;
    const size_t i = static_cast<size_t>(s / table_dt_) % table_.size();
    const size_t j = (i + 1) % table_.size();
    const double w = s / table_dt_ - std::floor(s / table_dt_);
    const auto& a = table_[i];
    const auto& b = table_[j];
    auto lerp = [w](double x, double y) { return x + w * (y - x); };
    return TrajectoryPoint{lerp(a.px, b.px), lerp(a.py, b.py), lerp(a.pz, b.pz),
                           lerp(a.vx, b.vx), lerp(a.vy, b.vy), lerp(a.vz, b.vz)};
}

std::vector<TrajectoryPoint> TestUavCirclePipeline::sampleTable(double t, double ramp_time, int steps, double dt) const
{
    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        const double ti = std::max(0.0, t + i * dt);
        double tau = ti - 0.5 * ramp_time;  // after the ramp
        double tau_dot = 1.0;
        if (ramp_time > 0.0 && ti < ramp_time) {
            tau = 0.5 * ti - ramp_time / (2.0 * M_PI) * std::sin(M_PI * ti / ramp_time);
            tau_dot = 0.5 * (1.0 - std::cos(M_PI * ti / ramp_time));
        }
        TrajectoryPoint pt = interpolate(tau);
        pt.vx *= tau_dot;
        pt.vy *= tau_dot;
        pt.vz *= tau_dot;
        points.push_back(pt);
    }
    return points;
}

} // namespace test_uav_circle
