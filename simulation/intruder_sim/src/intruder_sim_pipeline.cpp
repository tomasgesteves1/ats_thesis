#include "intruder_sim/intruder_sim_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace intruder_sim
{

namespace
{
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double dist(double ax, double ay, const std::array<double, 2> & b) { return std::hypot(ax - b[0], ay - b[1]); }
}  // namespace

IntruderSimPipeline::IntruderSimPipeline(const IntruderConfig & config) : cfg_(config)
{
    if (cfg_.patrol.empty()) {
        cfg_.patrol.push_back({0.0, 0.0});
    }
    state_.x = cfg_.patrol[0][0];
    state_.y = cfg_.patrol[0][1];
    waypoint_ = cfg_.patrol.size() > 1 ? 1 : 0;
    const auto & w = cfg_.patrol[waypoint_];
    state_.yaw = std::atan2(w[1] - state_.y, w[0] - state_.x);
}

double IntruderSimPipeline::fleeHeading(const std::array<double, 2> & usv) const
{
    // away from the USV ...
    double dx = state_.x - usv[0];
    double dy = state_.y - usv[1];
    const double n = std::hypot(dx, dy);
    dx = n > 1e-6 ? dx / n : std::cos(state_.yaw);
    dy = n > 1e-6 ? dy / n : std::sin(state_.yaw);
    // ... turned inwards near the edges of the zone (the push grows linearly within the margin)
    const auto & z = cfg_.zone;
    const double m = cfg_.zone_margin;
    dx += std::max(0.0, (z[0] + m - state_.x) / m) - std::max(0.0, (state_.x - (z[1] - m)) / m);
    dy += std::max(0.0, (z[2] + m - state_.y) / m) - std::max(0.0, (state_.y - (z[3] - m)) / m);
    if (std::hypot(dx, dy) < 1e-3) {
        return state_.yaw;
    }
    return std::atan2(dy, dx);
}

std::string IntruderSimPipeline::step(double dt, const std::array<double, 2> & uav, const std::array<double, 2> & usv)
{
    std::string event;
    double heading = state_.yaw;
    double speed = 0.0;

    if (state_.mode == Mode::kPatrol) {
        if (dist(state_.x, state_.y, uav) < cfg_.alert_radius || dist(state_.x, state_.y, usv) < cfg_.alert_radius) {
            state_.mode = Mode::kFlee;
            event = "pursuers within the alert radius: fleeing";
        } else {
            const auto & w = cfg_.patrol[waypoint_];
            if (std::hypot(w[0] - state_.x, w[1] - state_.y) < cfg_.waypoint_radius) {
                waypoint_ = (waypoint_ + 1) % cfg_.patrol.size();
            }
            const auto & t = cfg_.patrol[waypoint_];
            heading = std::atan2(t[1] - state_.y, t[0] - state_.x);
            speed = cfg_.patrol_speed;
        }
    }
    if (state_.mode == Mode::kFlee) {
        capture_timer_ = dist(state_.x, state_.y, usv) < cfg_.capture_radius ? capture_timer_ + dt : 0.0;
        if (capture_timer_ >= cfg_.capture_time) {
            state_.mode = Mode::kCaptured;
            event = "intercepted by the USV: stopping";
        } else {
            // away from the USV, zigzagging about that direction to evade it
            flee_time_ += dt;
            const double phase = std::sin(2.0 * M_PI * flee_time_ / std::max(cfg_.weave_period, 1e-3));
            heading = fleeHeading(usv) + (phase >= 0.0 ? cfg_.weave_angle : -cfg_.weave_angle);
            speed = cfg_.flee_speed;
        }
    }

    state_.yaw = wrap(state_.yaw + std::clamp(wrap(heading - state_.yaw), -cfg_.turn_rate * dt, cfg_.turn_rate * dt));
    state_.speed += std::clamp(speed - state_.speed, -cfg_.acceleration * dt, cfg_.acceleration * dt);
    state_.x += state_.speed * std::cos(state_.yaw) * dt;
    state_.y += state_.speed * std::sin(state_.yaw) * dt;
    return event;
}

}  // namespace intruder_sim
