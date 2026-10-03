#pragma once

#include <vector>

namespace test_uav_line {

struct TrajectoryPoint {
    double px{0.0};
    double py{0.0};
    double pz{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
};

struct MotionSegment {
    double t_start{0.0};
    double t_end{0.0};
    double s_start{0.0};
    double s_end{0.0};
    double v_target{0.0};
    double a{2.0};
    double t_acc{0.0};
    double t_cruise{0.0};
    double d_acc{0.0};
    double dir{1.0};
    bool is_pause{false};
};

struct LineTrajectoryConfig {
    double line_length{20.0};          ///< Total horizontal length of the line [m]
    double line_yaw{0.0};              ///< Heading of line in radians (0.0 = along world X)
    double center_x{0.0};              ///< Center X coordinate [m]
    double center_y{0.0};              ///< Center Y coordinate [m]
    double min_altitude{3.0};          ///< Minimum altitude (at s = -half_length) [m]
    double max_altitude{10.0};         ///< Maximum altitude (at s = +half_length) [m]
    double acceleration{2.0};          ///< Acceleration limit [m/s^2]
    double hover_time{12.0};           ///< Initial hover time for takeoff [s]
    double pause_time{4.0};            ///< Pause at waypoints [s]
    std::vector<double> speed_levels{2.0, 3.0}; ///< Target speeds [m/s]
};

class TestUavLinePipeline {
public:
    TestUavLinePipeline() = default;
    ~TestUavLinePipeline() = default;

    /// Initialize the motion profile and compute segments
    void init(const LineTrajectoryConfig& config);

    /// Generate horizon points starting from time t_query
    std::vector<TrajectoryPoint> generateHorizon(
        double t_query,
        int steps,
        double dt) const;

    /// Check if the mission is completed
    bool isFinished(double t_query) const;

    /// Get total planned mission duration
    double getTotalDuration() const { return total_duration_; }

    /// Check if pipeline has been initialized
    bool isInitialized() const { return initialized_; }

private:
    void addSegment(double s_from, double s_to, double v_max, double accel);
    void addPause(double s_pos, double duration);

    // Evaluate 1D position and velocity along line at elapsed time t
    void evaluate1D(double t, double& out_s, double& out_s_dot) const;

    LineTrajectoryConfig config_;
    std::vector<MotionSegment> segments_;
    double total_duration_{0.0};
    double total_length_3d_{0.0};
    double u_x_{1.0};
    double u_y_{0.0};
    double u_z_{0.0};
    double center_z_{6.5};
    bool initialized_{false};
};

} // namespace test_uav_line
