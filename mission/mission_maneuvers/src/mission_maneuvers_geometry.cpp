#include "mission_maneuvers/mission_maneuvers_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace mission_maneuvers
{

double minJerk(double s)
{
    s = std::clamp(s, 0.0, 1.0);
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

double minJerkIntegral(double s)
{
    s = std::clamp(s, 0.0, 1.0);
    const double s4 = s * s * s * s;
    return s4 * (2.5 - 3.0 * s + s * s);
}

TravelProfile::TravelProfile(double length, double speed, double ramp_time)
    : length_(std::max(length, 0.0)), speed_(std::max(std::abs(speed), 1e-6)), ramp_(std::max(ramp_time, 0.0))
{
    // each ramp covers speed * ramp / 2
    cruise_ = length_ / speed_ - ramp_;
    if (cruise_ < 0.0) {
        ramp_ = length_ / speed_;
        cruise_ = 0.0;
    }
}

double TravelProfile::distanceAt(double tau) const
{
    if (tau <= 0.0) {
        return 0.0;
    }
    if (ramp_ <= 0.0) {
        return std::min(speed_ * tau, length_);
    }
    if (tau < ramp_) {
        return speed_ * ramp_ * minJerkIntegral(tau / ramp_);
    }
    if (tau < ramp_ + cruise_) {
        return speed_ * (0.5 * ramp_ + (tau - ramp_));
    }
    if (tau < duration()) {
        return length_ - speed_ * ramp_ * minJerkIntegral((duration() - tau) / ramp_);
    }
    return length_;
}

double norm(const Vec3 & a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
double norm2(double x, double y) { return std::sqrt(x * x + y * y); }
Vec3 add(const Vec3 & a, const Vec3 & b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3 sub(const Vec3 & a, const Vec3 & b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec3 scale(const Vec3 & a, double k) { return {a[0] * k, a[1] * k, a[2] * k}; }
Vec3 lerp(const Vec3 & a, const Vec3 & b, double s) { return add(a, scale(sub(b, a), s)); }

double wrapAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

namespace
{
/// Planar position of the predicted horizon at time tq (linear interpolation, constant velocity beyond the end).
std::array<double, 2> horizonAt(const BoatHorizon & h, double tq)
{
    const auto & pts = h.x_y_yaw;
    const double u = (tq - h.stamp) / h.dt;
    if (u <= 0.0) {
        return {pts.front()[0], pts.front()[1]};
    }
    const auto last = static_cast<double>(pts.size() - 1);
    if (u >= last) {
        const auto & a = pts[pts.size() - 2];
        const auto & b = pts.back();
        const double extra = (u - last) * h.dt;
        return {b[0] + (b[0] - a[0]) / h.dt * extra, b[1] + (b[1] - a[1]) / h.dt * extra};
    }
    const auto i = static_cast<size_t>(u);
    const double f = u - static_cast<double>(i);
    return {pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), pts[i][1] + f * (pts[i + 1][1] - pts[i][1])};
}
}  // namespace

Vec3 boatDisplacement(const Context & ctx, double tau)
{
    const auto & h = ctx.boat.horizon;
    if (h.x_y_yaw.size() > 2 && ctx.t - h.stamp < ctx.horizon_timeout) {
        const auto a = horizonAt(h, ctx.t);
        const auto b = horizonAt(h, ctx.t + tau);
        return {b[0] - a[0], b[1] - a[1], 0.0};
    }
    return {ctx.boat.v[0] * tau, ctx.boat.v[1] * tau, 0.0};
}

double boatYawAt(const Context & ctx, double tau)
{
    const auto & h = ctx.boat.horizon;
    if (h.x_y_yaw.size() > 2 && ctx.t - h.stamp < ctx.horizon_timeout) {
        // heading change along the horizon, accumulated with wrapped increments
        const double u_now = std::clamp((ctx.t - h.stamp) / h.dt, 0.0, static_cast<double>(h.x_y_yaw.size() - 1));
        const double u_end = std::clamp((ctx.t + tau - h.stamp) / h.dt, 0.0, static_cast<double>(h.x_y_yaw.size() - 1));
        auto yawAt = [&h](double u) {
            const auto i = std::min(static_cast<size_t>(u), h.x_y_yaw.size() - 2);
            const double f = u - static_cast<double>(i);
            return h.x_y_yaw[i][2] + f * wrapAngle(h.x_y_yaw[i + 1][2] - h.x_y_yaw[i][2]);
        };
        return ctx.boat.yaw + wrapAngle(yawAt(u_end) - yawAt(u_now));
    }
    return ctx.boat.yaw + ctx.boat.yaw_rate * tau;
}

Vec3 rotateZ(const Vec3 & v, double yaw)
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]};
}

Vec3 toBoatFrame(const Context & ctx, const Vec3 & v)
{
    return rotateZ(v, -ctx.boat.yaw);
}

Vec3 anchorReference(const Context & ctx)
{
    return {ctx.boat.anchor[0], ctx.boat.anchor[1], ctx.boat.anchor_z_mean};
}

std::vector<UavPose> boatRelativePath(const Context & ctx, const std::function<Vec3(double)> & rel, bool boat_frame)
{
    std::vector<UavPose> path(static_cast<size_t>(ctx.uav_horizon.steps));
    const Vec3 origin = anchorReference(ctx);
    for (size_t k = 0; k < path.size(); ++k) {
        const double tau = static_cast<double>(k) * ctx.uav_horizon.dt;
        const Vec3 r = rel(ctx.t + tau);
        path[k].p = add(add(origin, boatDisplacement(ctx, tau)), boat_frame ? rotateZ(r, boatYawAt(ctx, tau)) : r);
        path[k].yaw = ctx.boat.yaw;
    }
    return path;
}

Vec3 uavRelative(const Context & ctx)
{
    return sub(ctx.uav.p, anchorReference(ctx));
}

Vec3 uavRelativeVelocity(const Context & ctx)
{
    return sub(ctx.uav.v, ctx.boat.v);
}

double Params::get(const std::string & key, double fallback) const
{
    const auto it = values_.find(key);
    return (it == values_.end() || it->second.empty()) ? fallback : it->second.front();
}

std::vector<double> Params::getVector(const std::string & key, const std::vector<double> & fallback) const
{
    const auto it = values_.find(key);
    return it == values_.end() ? fallback : it->second;
}

Vec3 Params::getVec3(const std::string & key, const Vec3 & fallback) const
{
    const auto it = values_.find(key);
    if (it == values_.end() || it->second.size() != 3) {
        return fallback;
    }
    return {it->second[0], it->second[1], it->second[2]};
}

}  // namespace mission_maneuvers
