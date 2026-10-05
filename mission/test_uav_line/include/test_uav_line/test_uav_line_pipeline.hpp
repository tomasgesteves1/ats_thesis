#pragma once

#include <array>
#include <vector>

namespace test_uav_line {

struct TrajectoryPoint {
    double px{0.0};
    double py{0.0};
    double pz{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double yaw{0.0};
};

/// Trapezoidal (or triangular) speed profile along a path parameter s in [0, length]
struct SpeedProfile {
    double length{0.0};
    double a{2.0};
    double v_target{0.0};
    double t_acc{0.0};
    double t_cruise{0.0};
    double d_acc{0.0};
};

enum class PhaseType { PAUSE, MOVE, CIRCLE };

struct Phase {
    PhaseType type{PhaseType::PAUSE};
    double t_start{0.0};
    double t_end{0.0};
    SpeedProfile profile;
    // PAUSE / MOVE: start point and unit direction of the 3D straight segment
    std::array<double, 3> p0{0.0, 0.0, 0.0};
    std::array<double, 3> u{0.0, 0.0, 0.0};
    // CIRCLE: centre, radius (arc length s maps to angle s / radius)
    std::array<double, 3> centre{0.0, 0.0, 0.0};
    double radius{1.0};
};

struct LineTrajectoryConfig {
    double line_length{20.0};          ///< Horizontal length of each line [m]
    double line_yaw{0.0};              ///< Heading of the first line, 0.0 = world X [rad]
    double center_x{0.0};              ///< Centre X coordinate [m]
    double center_y{0.0};              ///< Centre Y coordinate [m]
    double min_altitude{3.0};          ///< Altitude at the start of each line [m]
    double max_altitude{10.0};         ///< Altitude at the end of each line [m]
    double acceleration{2.0};          ///< Acceleration limit [m/s^2]
    double hover_time{12.0};           ///< Initial hover time for takeoff [s]
    double pause_time{4.0};            ///< Pause at waypoints [s]
    std::vector<double> speed_levels{2.0, 3.0}; ///< One inclined line per speed [m/s]
    bool perpendicular_lines{true};    ///< Rotate each successive line by 90 deg
    double circle_radius{5.0};         ///< Final circle radius [m] (<= 0 disables the circle)
    double circle_speed{2.5};          ///< Circle tangential speed [m/s]
    int circle_laps{2};                ///< Number of laps
    double yaw_amplitude{1.2};         ///< Yaw oscillation amplitude during the circle [rad]
    int yaw_cycles{3};                 ///< Full yaw oscillations over the whole circle
};

class TestUavLinePipeline {
public:
    TestUavLinePipeline() = default;
    ~TestUavLinePipeline() = default;

    /// Build the phase sequence (hover, inclined lines, circle with yaw sweep)
    void init(const LineTrajectoryConfig& config);

    /// Reference points (position, velocity, yaw) from t_query over `steps` steps of dt
    std::vector<TrajectoryPoint> generateHorizon(double t_query, int steps, double dt) const;

    bool isFinished(double t_query) const;
    double getTotalDuration() const { return total_duration_; }
    bool isInitialized() const { return initialized_; }

private:
    static SpeedProfile makeProfile(double length, double v_max, double accel);
    static void evalProfile(const SpeedProfile& p, double tau, double& s, double& s_dot);

    void addPause(const std::array<double, 3>& p, double duration);
    void addMove(const std::array<double, 3>& from, const std::array<double, 3>& to,
                 double v_max, double accel);
    void addCircle(const std::array<double, 3>& centre, double radius, double v_max,
                   double accel, int laps);

    TrajectoryPoint evaluate(double t) const;

    LineTrajectoryConfig config_;
    std::vector<Phase> phases_;
    std::array<double, 3> last_point_{0.0, 0.0, 0.0};
    double total_duration_{0.0};
    double circle_t_start_{-1.0};
    double circle_t_end_{-1.0};
    bool initialized_{false};
};

} // namespace test_uav_line
