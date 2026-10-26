#pragma once

#include <string>
#include <vector>

namespace test_uav_circle {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

class TestUavCirclePipeline {
public:
    TestUavCirclePipeline() = default;
    ~TestUavCirclePipeline() = default;

    // Generate circular trajectory points starting from start_time
    std::vector<TrajectoryPoint> generateCircle(
        double start_time,
        double radius,
        double omega,
        double height,
        double center_x,
        double center_y,
        double z_amplitude,
        int steps,
        double dt) const;

    // Periodic trajectory tabulated in a CSV file (t,x,y,z,vx,vy,vz,...; one lap, uniform time step). Returns false if
    // the file cannot be read.
    bool loadTable(const std::string& path);
    // A periodic table repeats; otherwise the reference holds the last point, at rest, after the end of the table
    void setTableLoop(bool loop) { loop_ = loop; }
    bool hasTable() const { return !table_.empty(); }
    TrajectoryPoint firstTablePoint() const { return table_.front(); }

    // Samples the table over the horizon from the trajectory time t. The entry from hover follows a time warp tau(t)
    // whose rate rises smoothly from 0 to 1 over ramp_time, tau_dot = (1 - cos(pi t / ramp_time)) / 2, so the reference
    // starts at rest on the first point and reaches the nominal speed with a continuous acceleration.
    std::vector<TrajectoryPoint> sampleTable(double t, double ramp_time, int steps, double dt) const;

private:
    TrajectoryPoint interpolate(double tau) const;  // periodic, linear between samples

    std::vector<TrajectoryPoint> table_;
    double table_dt_{0.01};
    bool loop_{true};
};

} // namespace test_uav_circle
