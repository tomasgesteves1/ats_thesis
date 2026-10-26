#include "test_usv_circle/test_usv_circle_pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace test_usv_circle {

std::vector<TrajectoryPoint> TestUsvCirclePipeline::generateCircle(
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

bool TestUsvCirclePipeline::loadTable(const std::string& path, bool loop)
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
        double v[4];
        int k = 0;
        while (k < 4 && std::getline(cols, col, ',')) v[k++] = std::stod(col);
        if (k < 4) continue;
        times.push_back(v[0]);
        table_.push_back({v[1], v[2], v[3]});
    }
    if (table_.size() < 2) {
        table_.clear();
        return false;
    }
    table_dt_ = times[1] - times[0];
    loop_ = loop;
    return true;
}

std::array<double, 3> TestUsvCirclePipeline::interpolate(double tau) const
{
    const double last = table_dt_ * (table_.size() - 1);
    if (tau <= 0.0) return table_.front();
    if (!loop_ && tau >= last) return table_.back();
    const double period = table_dt_ * table_.size();
    const double s = std::fmod(tau, period);
    const size_t i = static_cast<size_t>(s / table_dt_) % table_.size();
    const size_t j = (i + 1) % table_.size();
    const double w = s / table_dt_ - std::floor(s / table_dt_);
    const auto& a = table_[i];
    const auto& b = table_[j];
    return {a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2])};
}

std::vector<std::array<double, 3>> TestUsvCirclePipeline::sampleTable(double t, int steps, double dt) const
{
    std::vector<std::array<double, 3>> poses;
    poses.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) poses.push_back(interpolate(t + i * dt));
    return poses;
}

} // namespace test_usv_circle
