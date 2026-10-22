#include "mission_maneuvers/mission_maneuvers_usv.hpp"

#include <algorithm>
#include <cmath>


namespace mission_maneuvers
{

// ---------------------------------------------------------------------------------------------------------------
// UsvHold
// ---------------------------------------------------------------------------------------------------------------

void UsvHold::start(const Context & ctx)
{
    t0_ = ctx.t;
    pose_ = {ctx.boat.p[0], ctx.boat.p[1], ctx.boat.yaw};
}

Status UsvHold::update(const Context & ctx, UsvCommand & cmd)
{
    cmd.path.assign(static_cast<size_t>(ctx.usv_horizon.steps), pose_);
    return ctx.t - t0_ >= params_.get("duration", 0.0) ? Status::kDone : Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// UsvWaypoints
// ---------------------------------------------------------------------------------------------------------------

void UsvWaypoints::start(const Context & ctx)
{
    t0_ = ctx.t;
    points_ = {{ctx.boat.p[0], ctx.boat.p[1]}};
    const auto flat = params_.getVector("points", {});
    for (size_t i = 0; i + 1 < flat.size(); i += 2) {
        points_.push_back({flat[i], flat[i + 1]});
    }
    cumulative_ = {0.0};
    for (size_t i = 1; i < points_.size(); ++i) {
        cumulative_.push_back(cumulative_.back() +
                              norm2(points_[i][0] - points_[i - 1][0], points_[i][1] - points_[i - 1][1]));
    }
    travel_ = TravelProfile(cumulative_.back(), params_.get("speed", 1.0), params_.get("ramp_time", 5.0));
    final_yaw_ = ctx.boat.yaw;
    if (points_.size() > 1) {
        const auto & a = points_[points_.size() - 2];
        const auto & b = points_.back();
        final_yaw_ = std::atan2(b[1] - a[1], b[0] - a[0]);
    }
}

std::array<double, 2> UsvWaypoints::pointAt(double s) const
{
    if (points_.size() < 2 || s <= 0.0) {
        return points_.front();
    }
    if (s >= cumulative_.back()) {
        return points_.back();
    }
    const auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), s);
    const auto i = static_cast<size_t>(std::distance(cumulative_.begin(), it));   // segment (i-1, i)
    const double f = (s - cumulative_[i - 1]) / (cumulative_[i] - cumulative_[i - 1]);
    return {points_[i - 1][0] + f * (points_[i][0] - points_[i - 1][0]),
            points_[i - 1][1] + f * (points_[i][1] - points_[i - 1][1])};
}

Status UsvWaypoints::update(const Context & ctx, UsvCommand & cmd)
{
    const double lookahead = params_.get("lookahead", 3.0);
    const double length = cumulative_.back();
    cmd.path.resize(static_cast<size_t>(ctx.usv_horizon.steps));
    for (size_t k = 0; k < cmd.path.size(); ++k) {
        const double s = travel_.distanceAt(ctx.t - t0_ + static_cast<double>(k) * ctx.usv_horizon.dt);
        const auto p = pointAt(s);
        double yaw = final_yaw_;
        if (s + 1e-3 < length) {
            const auto q = pointAt(std::min(s + lookahead, length));
            yaw = std::atan2(q[1] - p[1], q[0] - p[0]);
        }
        cmd.path[k] = {p[0], p[1], yaw};
    }
    const bool finished = ctx.t - t0_ >= travel_.duration() &&
                          norm2(ctx.boat.p[0] - points_.back()[0], ctx.boat.p[1] - points_.back()[1]) <
                              params_.get("tolerance", 1.5);
    return finished ? Status::kDone : Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// UsvFollow
// ---------------------------------------------------------------------------------------------------------------

void UsvFollow::start(const Context & ctx)
{
    t0_ = ctx.t;
    v0_ = norm2(ctx.boat.v[0], ctx.boat.v[1]);
    settle_since_ = -1.0;
    hold_ = {ctx.boat.p[0], ctx.boat.p[1], ctx.boat.yaw};
}

Status UsvFollow::update(const Context & ctx, UsvCommand & cmd)
{
    cmd.path.resize(static_cast<size_t>(ctx.usv_horizon.steps));
    if (!ctx.target.valid) {
        cmd.path.assign(cmd.path.size(), hold_);
        return Status::kRunning;
    }
    const double speed = params_.get("speed", 1.5);
    const double ramp = params_.get("ramp_time", 4.0);
    const double standoff = params_.get("standoff", 6.0);
    const double turn_rate = params_.get("turn_rate", 0.3);
    const double dt = ctx.usv_horizon.dt;
    // The reference is a unicycle started at the current pose of the boat that turns towards the target (predicted
    // with constant velocity) with a limited turn rate and a speed ramped to the pursuit speed, and stops at the
    // standoff distance: a reference the boat can follow bow first (a jump of the reference heading towards the
    // target makes the NMPC back towards it instead)
    double x = ctx.boat.p[0], y = ctx.boat.p[1], yaw = ctx.boat.yaw;
    for (size_t k = 0; k < cmd.path.size(); ++k) {
        cmd.path[k] = {x, y, yaw};
        const double tau = static_cast<double>(k) * dt;
        const Vec3 target = add(ctx.target.p, scale(ctx.target.v, tau));
        const double bearing = std::atan2(target[1] - y, target[0] - x);
        yaw += std::clamp(wrapAngle(bearing - yaw), -turn_rate * dt, turn_rate * dt);
        const double d = norm2(target[0] - x, target[1] - y);
        const double v = d > standoff ? v0_ + (speed - v0_) * minJerk((ctx.t + tau - t0_) / ramp) : 0.0;
        const double step = std::min(v * dt, std::max(0.0, d - standoff));
        x += step * std::cos(yaw);
        y += step * std::sin(yaw);
    }
    hold_ = cmd.path.front();
    const double d = norm2(ctx.target.p[0] - ctx.boat.p[0], ctx.target.p[1] - ctx.boat.p[1]);
    if (d < params_.get("intercept_distance", 10.0)) {
        if (settle_since_ < 0.0) {
            settle_since_ = ctx.t;
        }
        if (ctx.t - settle_since_ >= params_.get("settle", 3.0)) {
            cmd.message = "target intercepted";
            return Status::kDone;
        }
    } else {
        settle_since_ = -1.0;
    }
    return Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// UsvSupport
// ---------------------------------------------------------------------------------------------------------------

void UsvSupport::start(const Context & ctx)
{
    direction_ = 1.0;
    (void)ctx;
}

Status UsvSupport::update(const Context & ctx, UsvCommand & cmd)
{
    const auto c = params_.getVector("center", {0.0, 0.0});
    const double standoff = params_.get("standoff", 6.0);
    const double rx = ctx.uav.p[0] - c[0], ry = ctx.uav.p[1] - c[1];
    const double r = std::max(norm2(rx, ry), 1e-3);
    const double theta = std::atan2(ry, rx);
    // angular speed of the UAV about the structure; the sense of turn is kept while the UAV is (almost) still
    const double omega = (rx * ctx.uav.v[1] - ry * ctx.uav.v[0]) / (r * r);
    if (std::abs(omega) > 0.01) {
        direction_ = omega > 0.0 ? 1.0 : -1.0;
    }
    cmd.path.resize(static_cast<size_t>(ctx.usv_horizon.steps));
    for (size_t k = 0; k < cmd.path.size(); ++k) {
        const double th = theta + omega * static_cast<double>(k) * ctx.usv_horizon.dt;
        cmd.path[k] = {c[0] + (r + standoff) * std::cos(th), c[1] + (r + standoff) * std::sin(th),
                       th + direction_ * M_PI_2};
    }
    return Status::kRunning;   // ended by the mission ('until')
}

}  // namespace mission_maneuvers
