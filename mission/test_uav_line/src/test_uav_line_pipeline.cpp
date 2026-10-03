#include "test_uav_line/test_uav_line_pipeline.hpp"
#include <cmath>
#include <algorithm>

namespace test_uav_line {

void TestUavLinePipeline::init(const LineTrajectoryConfig& config) {
    config_ = config;
    segments_.clear();
    total_duration_ = 0.0;

    const double delta_z = config_.max_altitude - config_.min_altitude;
    total_length_3d_ = std::sqrt(config_.line_length * config_.line_length + delta_z * delta_z);
    if (total_length_3d_ < 1e-6) {
        total_length_3d_ = 1.0;
    }

    const double cos_yaw = std::cos(config_.line_yaw);
    const double sin_yaw = std::sin(config_.line_yaw);

    // Unit vector components along the 3D straight line
    u_x_ = cos_yaw * (config_.line_length / total_length_3d_);
    u_y_ = sin_yaw * (config_.line_length / total_length_3d_);
    u_z_ = delta_z / total_length_3d_;

    center_z_ = (config_.min_altitude + config_.max_altitude) / 2.0;

    const double half_len_3d = total_length_3d_ / 2.0;

    // 1. Initial hover for takeoff & stabilization above boat at center altitude
    addPause(0.0, config_.hover_time);

    // 2. Initial gentle repositioning to start of line (-half_len_3d, altitude = min_altitude)
    double v_prep = 1.5;
    if (!config_.speed_levels.empty() && config_.speed_levels.front() < 1.5) {
        v_prep = config_.speed_levels.front();
    }
    addSegment(0.0, -half_len_3d, v_prep, config_.acceleration);
    addPause(-half_len_3d, config_.pause_time);

    // 3. Sequential sweeps for each speed level (e.g. 4.0 m/s and 5.0 m/s)
    for (double v : config_.speed_levels) {
        // Forward climb along 3D line from min_altitude to max_altitude
        addSegment(-half_len_3d, +half_len_3d, v, config_.acceleration);
        addPause(+half_len_3d, config_.pause_time);

        // Backward descent along 3D line from max_altitude to min_altitude
        addSegment(+half_len_3d, -half_len_3d, v, config_.acceleration);
        addPause(-half_len_3d, config_.pause_time);
    }

    // 4. Return gently to center (0.0) above boat at center_z_
    addSegment(-half_len_3d, 0.0, v_prep, config_.acceleration);
    addPause(0.0, config_.pause_time);

    initialized_ = true;
}

void TestUavLinePipeline::addSegment(double s_from, double s_to, double v_max, double accel) {
    const double delta_s = s_to - s_from;
    const double dist = std::abs(delta_s);
    if (dist < 1e-6) {
        return;
    }

    const double dir = (delta_s > 0.0) ? 1.0 : -1.0;
    const double a = std::max(0.1, accel);
    const double v = std::max(0.1, v_max);

    MotionSegment seg;
    seg.t_start = total_duration_;
    seg.s_start = s_from;
    seg.s_end = s_to;
    seg.a = a;
    seg.dir = dir;
    seg.is_pause = false;

    // Check if profile reaches target velocity or is triangular
    const double dist_to_reach_v = (v * v) / (2.0 * a);
    if (2.0 * dist_to_reach_v > dist) {
        // Triangular profile
        seg.v_target = std::sqrt(dist * a);
        seg.t_acc = seg.v_target / a;
        seg.t_cruise = 0.0;
        seg.d_acc = 0.5 * dist;
    } else {
        // Trapezoidal profile
        seg.v_target = v;
        seg.t_acc = v / a;
        seg.d_acc = dist_to_reach_v;
        const double d_cruise = dist - 2.0 * seg.d_acc;
        seg.t_cruise = d_cruise / v;
    }

    const double seg_duration = 2.0 * seg.t_acc + seg.t_cruise;
    seg.t_end = seg.t_start + seg_duration;
    total_duration_ = seg.t_end;

    segments_.push_back(seg);
}

void TestUavLinePipeline::addPause(double s_pos, double duration) {
    if (duration <= 0.0) {
        return;
    }

    MotionSegment seg;
    seg.t_start = total_duration_;
    seg.t_end = total_duration_ + duration;
    seg.s_start = s_pos;
    seg.s_end = s_pos;
    seg.v_target = 0.0;
    seg.is_pause = true;

    total_duration_ = seg.t_end;
    segments_.push_back(seg);
}

void TestUavLinePipeline::evaluate1D(double t, double& out_s, double& out_s_dot) const {
    if (segments_.empty()) {
        out_s = 0.0;
        out_s_dot = 0.0;
        return;
    }

    if (t <= 0.0) {
        out_s = segments_.front().s_start;
        out_s_dot = 0.0;
        return;
    }

    if (t >= total_duration_) {
        out_s = segments_.back().s_end;
        out_s_dot = 0.0;
        return;
    }

    for (const auto& seg : segments_) {
        if (t >= seg.t_start && t <= seg.t_end) {
            if (seg.is_pause) {
                out_s = seg.s_start;
                out_s_dot = 0.0;
                return;
            }

            const double tau = t - seg.t_start;
            if (tau <= seg.t_acc) {
                // Acceleration phase
                out_s = seg.s_start + seg.dir * 0.5 * seg.a * tau * tau;
                out_s_dot = seg.dir * seg.a * tau;
            } else if (tau <= seg.t_acc + seg.t_cruise) {
                // Cruise phase
                const double dt_cruise = tau - seg.t_acc;
                out_s = seg.s_start + seg.dir * (seg.d_acc + seg.v_target * dt_cruise);
                out_s_dot = seg.dir * seg.v_target;
            } else {
                // Deceleration phase
                const double dt_dec = tau - (seg.t_acc + seg.t_cruise);
                const double rem = std::max(0.0, seg.t_acc - dt_dec);
                out_s = seg.s_end - seg.dir * 0.5 * seg.a * rem * rem;
                out_s_dot = seg.dir * (seg.v_target - seg.a * dt_dec);
            }
            return;
        }
    }

    out_s = segments_.back().s_end;
    out_s_dot = 0.0;
}

std::vector<TrajectoryPoint> TestUavLinePipeline::generateHorizon(
    double t_query,
    int steps,
    double dt) const
{
    std::vector<TrajectoryPoint> points;
    points.reserve(steps + 1);

    for (int i = 0; i <= steps; ++i) {
        const double t = t_query + i * dt;
        double s = 0.0;
        double s_dot = 0.0;
        evaluate1D(t, s, s_dot);

        TrajectoryPoint pt;
        pt.px = config_.center_x + s * u_x_;
        pt.py = config_.center_y + s * u_y_;
        pt.pz = center_z_ + s * u_z_;

        pt.vx = s_dot * u_x_;
        pt.vy = s_dot * u_y_;
        pt.vz = s_dot * u_z_;

        points.push_back(pt);
    }

    return points;
}

bool TestUavLinePipeline::isFinished(double t_query) const {
    return (t_query >= total_duration_);
}

} // namespace test_uav_line
