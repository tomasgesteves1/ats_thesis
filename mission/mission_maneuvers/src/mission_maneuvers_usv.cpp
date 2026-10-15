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

}  // namespace mission_maneuvers
