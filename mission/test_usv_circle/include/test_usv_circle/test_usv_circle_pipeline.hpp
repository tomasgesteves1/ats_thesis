#pragma once

#include <array>
#include <string>
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

    // Trajectory tabulated in a CSV file (t,x,y,psi,...; uniform time step, psi unwrapped). A periodic table repeats;
    // otherwise the reference holds the last pose after the end of the table. Returns false if the file cannot be read.
    bool loadTable(const std::string& path, bool loop);
    bool hasTable() const { return !table_.empty(); }
    // Poses [x, y, psi] over the horizon from the trajectory time t (held at the first pose for t < 0)
    std::vector<std::array<double, 3>> sampleTable(double t, int steps, double dt) const;

private:
    std::array<double, 3> interpolate(double tau) const;

    std::vector<std::array<double, 3>> table_;
    double table_dt_{0.1};
    bool loop_{true};
};

} // namespace test_usv_circle
