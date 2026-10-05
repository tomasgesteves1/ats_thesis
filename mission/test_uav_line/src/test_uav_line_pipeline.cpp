#include "test_uav_line/test_uav_line_pipeline.hpp"
#include <algorithm>
#include <cmath>

namespace test_uav_line {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

SpeedProfile TestUavLinePipeline::makeProfile(double length, double v_max, double accel) {
    SpeedProfile p;
    p.length = length;
    p.a = std::max(0.1, accel);
    const double v = std::max(0.1, v_max);

    const double dist_to_reach_v = (v * v) / (2.0 * p.a);
    if (2.0 * dist_to_reach_v > length) {
        // Triangular profile
        p.v_target = std::sqrt(length * p.a);
        p.t_acc = p.v_target / p.a;
        p.t_cruise = 0.0;
        p.d_acc = 0.5 * length;
    } else {
        // Trapezoidal profile
        p.v_target = v;
        p.t_acc = v / p.a;
        p.d_acc = dist_to_reach_v;
        p.t_cruise = (length - 2.0 * p.d_acc) / v;
    }
    return p;
}

void TestUavLinePipeline::evalProfile(const SpeedProfile& p, double tau, double& s, double& s_dot) {
    if (tau <= 0.0) {
        s = 0.0;
        s_dot = 0.0;
    } else if (tau <= p.t_acc) {
        s = 0.5 * p.a * tau * tau;
        s_dot = p.a * tau;
    } else if (tau <= p.t_acc + p.t_cruise) {
        s = p.d_acc + p.v_target * (tau - p.t_acc);
        s_dot = p.v_target;
    } else {
        const double dt_dec = tau - (p.t_acc + p.t_cruise);
        const double rem = std::max(0.0, p.t_acc - dt_dec);
        s = p.length - 0.5 * p.a * rem * rem;
        s_dot = std::max(0.0, p.v_target - p.a * dt_dec);
    }
}

void TestUavLinePipeline::addPause(const std::array<double, 3>& p, double duration) {
    if (duration <= 0.0) {
        return;
    }
    Phase ph;
    ph.type = PhaseType::PAUSE;
    ph.t_start = total_duration_;
    ph.t_end = total_duration_ + duration;
    ph.p0 = p;
    total_duration_ = ph.t_end;
    last_point_ = p;
    phases_.push_back(ph);
}

void TestUavLinePipeline::addMove(const std::array<double, 3>& from, const std::array<double, 3>& to,
                                  double v_max, double accel) {
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double dz = to[2] - from[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1e-6) {
        return;
    }
    Phase ph;
    ph.type = PhaseType::MOVE;
    ph.t_start = total_duration_;
    ph.p0 = from;
    ph.u = {dx / dist, dy / dist, dz / dist};
    ph.profile = makeProfile(dist, v_max, accel);
    ph.t_end = ph.t_start + 2.0 * ph.profile.t_acc + ph.profile.t_cruise;
    total_duration_ = ph.t_end;
    last_point_ = to;
    phases_.push_back(ph);
}

void TestUavLinePipeline::addCircle(const std::array<double, 3>& centre, double radius,
                                    double v_max, double accel, int laps) {
    Phase ph;
    ph.type = PhaseType::CIRCLE;
    ph.t_start = total_duration_;
    ph.centre = centre;
    ph.radius = radius;
    ph.profile = makeProfile(2.0 * kPi * radius * std::max(1, laps), v_max, accel);
    ph.t_end = ph.t_start + 2.0 * ph.profile.t_acc + ph.profile.t_cruise;
    circle_t_start_ = ph.t_start;
    circle_t_end_ = ph.t_end;
    total_duration_ = ph.t_end;
    last_point_ = {centre[0] + radius, centre[1], centre[2]};
    phases_.push_back(ph);
}

void TestUavLinePipeline::init(const LineTrajectoryConfig& config) {
    config_ = config;
    phases_.clear();
    total_duration_ = 0.0;
    circle_t_start_ = circle_t_end_ = -1.0;

    const double delta_z = config_.max_altitude - config_.min_altitude;
    const double centre_z = 0.5 * (config_.min_altitude + config_.max_altitude);
    const std::array<double, 3> centre{config_.center_x, config_.center_y, centre_z};

    double v_prep = 1.5;
    if (!config_.speed_levels.empty() && config_.speed_levels.front() < 1.5) {
        v_prep = config_.speed_levels.front();
    }

    // 1. Initial hover at the centre
    addPause(centre, config_.hover_time);

    // 2. One inclined line per speed level, each rotated 90 deg from the previous one
    for (size_t k = 0; k < config_.speed_levels.size(); ++k) {
        const double yaw_line = config_.line_yaw +
            (config_.perpendicular_lines ? static_cast<double>(k) * 0.5 * kPi : 0.0);
        const double hx = 0.5 * config_.line_length * std::cos(yaw_line);
        const double hy = 0.5 * config_.line_length * std::sin(yaw_line);
        const std::array<double, 3> lower{centre[0] - hx, centre[1] - hy, centre[2] - 0.5 * delta_z};
        const std::array<double, 3> upper{centre[0] + hx, centre[1] + hy, centre[2] + 0.5 * delta_z};
        const double v = config_.speed_levels[k];

        addMove(last_point_, lower, v_prep, config_.acceleration);
        addPause(lower, config_.pause_time);
        addMove(lower, upper, v, config_.acceleration);   // climb
        addPause(upper, config_.pause_time);
        addMove(upper, lower, v, config_.acceleration);   // descent
        addPause(lower, config_.pause_time);
    }

    // 3. Final circle at constant altitude, with the yaw sweeping independently of the motion
    if (config_.circle_radius > 0.0) {
        const std::array<double, 3> start{centre[0] + config_.circle_radius, centre[1], centre[2]};
        addMove(last_point_, start, v_prep, config_.acceleration);
        addPause(start, config_.pause_time);
        addCircle(centre, config_.circle_radius, config_.circle_speed,
                  config_.acceleration, config_.circle_laps);
        addPause(last_point_, config_.pause_time);
    }

    // 4. Return to the centre
    addMove(last_point_, centre, v_prep, config_.acceleration);
    addPause(centre, config_.pause_time);

    initialized_ = true;
}

TrajectoryPoint TestUavLinePipeline::evaluate(double t) const {
    TrajectoryPoint pt;
    if (phases_.empty()) {
        return pt;
    }
    const Phase* ph = &phases_.back();
    double tau = ph->t_end - ph->t_start;
    if (t <= 0.0) {
        ph = &phases_.front();
        tau = 0.0;
    } else if (t < total_duration_) {
        for (const auto& candidate : phases_) {
            if (t >= candidate.t_start && t <= candidate.t_end) {
                ph = &candidate;
                tau = t - candidate.t_start;
                break;
            }
        }
    }

    double s = 0.0;
    double s_dot = 0.0;
    switch (ph->type) {
    case PhaseType::PAUSE:
        pt.px = ph->p0[0];
        pt.py = ph->p0[1];
        pt.pz = ph->p0[2];
        break;
    case PhaseType::MOVE:
        evalProfile(ph->profile, tau, s, s_dot);
        pt.px = ph->p0[0] + s * ph->u[0];
        pt.py = ph->p0[1] + s * ph->u[1];
        pt.pz = ph->p0[2] + s * ph->u[2];
        pt.vx = s_dot * ph->u[0];
        pt.vy = s_dot * ph->u[1];
        pt.vz = s_dot * ph->u[2];
        break;
    case PhaseType::CIRCLE: {
        evalProfile(ph->profile, tau, s, s_dot);
        const double theta = s / ph->radius;
        pt.px = ph->centre[0] + ph->radius * std::cos(theta);
        pt.py = ph->centre[1] + ph->radius * std::sin(theta);
        pt.pz = ph->centre[2];
        pt.vx = -s_dot * std::sin(theta);
        pt.vy = s_dot * std::cos(theta);
        // Yaw: yaw_cycles full oscillations over the whole arc, starting and ending at zero
        pt.yaw = config_.yaw_amplitude *
                 std::sin(2.0 * kPi * config_.yaw_cycles * s / ph->profile.length);
        break;
    }
    }
    return pt;
}

std::vector<TrajectoryPoint> TestUavLinePipeline::generateHorizon(
    double t_query, int steps, double dt) const
{
    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        points.push_back(evaluate(t_query + i * dt));
    }
    return points;
}

bool TestUavLinePipeline::isFinished(double t_query) const {
    return t_query >= total_duration_;
}

} // namespace test_uav_line
