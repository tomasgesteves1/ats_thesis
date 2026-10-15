#include "mission_maneuvers/mission_maneuvers_uav.hpp"

#include <cmath>
#include <sstream>

namespace mission_maneuvers
{

namespace
{
std::string fmt(const char * text, double value)
{
    std::ostringstream os;
    os.precision(2);
    os << std::fixed << text << value;
    return os.str();
}
}  // namespace

bool SettleTimer::update(bool condition, double t, double hold_time)
{
    if (!condition) {
        since_ = -1.0;
        return false;
    }
    if (since_ < 0.0) {
        since_ = t;
    }
    return t - since_ >= hold_time;
}

// ---------------------------------------------------------------------------------------------------------------
// Takeoff
// ---------------------------------------------------------------------------------------------------------------

void Takeoff::start(const Context & ctx)
{
    climbing_ = false;
    pull_released_ = false;
    t_start_ = ctx.t;
    rel0_ = uavRelative(ctx);
}

Vec3 Takeoff::relAt(double t) const
{
    if (!climbing_) {
        return rel0_;
    }
    // vertical leg off the pad (the horizontal position on the pad is kept), then transfer to the offset
    const double clearance_time = params_.get("clearance_duration", 4.0);
    const Vec3 cleared = add(rel0_, {0.0, 0.0, params_.get("vertical_clearance", 1.0)});
    const double tau = t - t0_;
    if (tau < clearance_time) {
        return lerp(rel0_, cleared, minJerk(tau / clearance_time));
    }
    return lerp(cleared, params_.getVec3("offset", {0.0, 0.0, 3.0}),
                minJerk((tau - clearance_time) / params_.get("duration", 8.0)));
}

Status Takeoff::update(const Context & ctx, UavCommand & cmd)
{
    cmd.yaw_from_reference = true;
    cmd.low_altitude = true;
    cmd.request_offboard = true;
    cmd.request_arm = !climbing_;
    // the winch keeps pulling the UAV towards the pad until it is clear of the deck (height above the current,
    // heaving winch position)
    if (ctx.uav.p[2] - ctx.boat.anchor[2] > params_.get("pull_release_height", 1.6)) {
        pull_released_ = true;
    }
    cmd.tether_pull = !pull_released_;

    if (!climbing_) {
        // on the pad: the reference stays where the UAV is until it is armed in offboard mode
        if (ctx.uav.armed && ctx.uav.offboard) {
            climbing_ = true;
            t0_ = ctx.t;
            rel0_ = uavRelative(ctx);
            cmd.message = "armed in offboard mode, climbing";
        } else if (ctx.t - t_start_ > params_.get("arm_timeout", 30.0)) {
            cmd.message = "not armed in offboard mode within the timeout";
            cmd.path = boatRelativePath(ctx, [this](double t) { return relAt(t); });
            return Status::kFailed;
        }
    }
    cmd.path = boatRelativePath(ctx, [this](double t) { return relAt(t); });
    const double total = params_.get("clearance_duration", 4.0) + params_.get("duration", 8.0);
    if (climbing_ && ctx.t - t0_ >= total &&
        norm(sub(uavRelative(ctx), params_.getVec3("offset", {0.0, 0.0, 3.0}))) < params_.get("tolerance", 0.3)) {
        return Status::kDone;
    }
    return Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// GotoBoat
// ---------------------------------------------------------------------------------------------------------------

void GotoBoat::start(const Context & ctx)
{
    t0_ = ctx.t;
    boat_frame_ = params_.get("boat_frame", 0.0) > 0.5;
    rel0_ = boat_frame_ ? toBoatFrame(ctx, uavRelative(ctx)) : uavRelative(ctx);
    settle_.reset();
}

Status GotoBoat::update(const Context & ctx, UavCommand & cmd)
{
    const Vec3 offset = params_.getVec3("offset", {0.0, 0.0, 3.0});
    const double duration = params_.get("duration", 8.0);
    cmd.yaw_from_reference = params_.get("yaw_from_reference", 0.0) > 0.5;
    cmd.path = boatRelativePath(ctx, [&](double t) {
        return lerp(rel0_, offset, minJerk((t - t0_) / duration));
    }, boat_frame_);
    if (ctx.t - t0_ < duration) {
        return Status::kRunning;
    }
    const Vec3 target = boat_frame_ ? rotateZ(offset, ctx.boat.yaw) : offset;
    const Vec3 vrel = uavRelativeVelocity(ctx);
    const bool ok = norm(sub(uavRelative(ctx), target)) < params_.get("tolerance", 0.15) &&
                    norm2(vrel[0], vrel[1]) < params_.get("velocity_tolerance", 0.15);
    if (settle_.update(ok, ctx.t, params_.get("settle", 3.0))) {
        return Status::kDone;
    }
    if (ctx.t - t0_ > duration + params_.get("timeout", 40.0)) {
        if (params_.get("fail_on_timeout", 0.0) > 0.5) {
            cmd.message = "tolerance not reached within the timeout";
            return Status::kFailed;
        }
        cmd.message = "tolerance not reached within the timeout, continuing";
        return Status::kDone;
    }
    return Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// UavHold
// ---------------------------------------------------------------------------------------------------------------

void UavHold::start(const Context & ctx)
{
    t0_ = ctx.t;
    boat_frame_ = params_.get("boat_frame", 0.0) > 0.5;
    const Vec3 here = boat_frame_ ? toBoatFrame(ctx, uavRelative(ctx)) : uavRelative(ctx);
    offset_ = params_.getVec3("offset", here);
}

Status UavHold::update(const Context & ctx, UavCommand & cmd)
{
    cmd.yaw_from_reference = params_.get("yaw_from_reference", 0.0) > 0.5;
    cmd.path = boatRelativePath(ctx, [this](double) { return offset_; }, boat_frame_);
    const double duration = params_.get("duration", 0.0);
    return duration > 0.0 && ctx.t - t0_ >= duration ? Status::kDone : Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// Orbit
// ---------------------------------------------------------------------------------------------------------------

void Orbit::start(const Context & ctx)
{
    t0_ = ctx.t;
    rel0_ = uavRelative(ctx);
    // the orbit starts at the current bearing from the winch (behind the boat if the UAV is above it)
    theta0_ = norm2(rel0_[0], rel0_[1]) > 0.5 ? std::atan2(rel0_[1], rel0_[0]) : ctx.boat.yaw + M_PI;
    const double omega = params_.get("omega", 0.3);
    direction_ = omega < 0.0 ? -1.0 : 1.0;
    angle_ = TravelProfile(2.0 * M_PI * params_.get("laps", 1.0), omega, params_.get("ramp_time", 4.0));
}

Vec3 Orbit::relAt(double t) const
{
    const double r = params_.get("radius", 5.0);
    const double h = params_.get("height", 4.0);
    const double entry = params_.get("entry_duration", 8.0);
    const double tau = t - t0_;
    if (tau < entry) {
        const Vec3 start_point{r * std::cos(theta0_), r * std::sin(theta0_), h};
        return lerp(rel0_, start_point, minJerk(tau / entry));
    }
    const double theta = theta0_ + direction_ * angle_.distanceAt(tau - entry);
    return {r * std::cos(theta), r * std::sin(theta), h};
}

Status Orbit::update(const Context & ctx, UavCommand & cmd)
{
    cmd.yaw_from_reference = false;   // the UAV looks at the winch
    cmd.path = boatRelativePath(ctx, [this](double t) { return relAt(t); });
    return ctx.t - t0_ >= params_.get("entry_duration", 8.0) + angle_.duration() ? Status::kDone : Status::kRunning;
}

// ---------------------------------------------------------------------------------------------------------------
// Land
// ---------------------------------------------------------------------------------------------------------------

void Land::start(const Context & ctx)
{
    phase_ = Phase::kApproach;
    t_phase_ = ctx.t;
    rel0_ = uavRelative(ctx);
    settle_.reset();
    rest_height_ = ctx.uav_rest_height;
    final_approach_ = TravelProfile(params_.get("final_height", 0.5) + params_.get("depth", 0.2),
                                    params_.get("final_speed", 0.15), params_.get("final_ramp", 1.0));
}

Vec3 Land::relAt(double t) const
{
    const Vec3 above{0.0, 0.0, params_.get("height", 3.0)};
    switch (phase_) {
        case Phase::kApproach:
            return lerp(rel0_, above, minJerk((t - t_phase_) / params_.get("approach_duration", 8.0)));
        case Phase::kTrack:
            return above;
        default: {
            // minimum-jerk descent to final_height above the pad, then slow final approach at final_speed down to
            // depth below it (the deck heave then sets the worst touchdown speed, not the descent)
            const double t_descent = params_.get("descent_duration", 12.0);
            const double z_final = rest_height_ + params_.get("final_height", 0.5);
            const double tau = t - t_phase_;
            if (tau < t_descent) {
                return {0.0, 0.0, above[2] + (z_final - above[2]) * minJerk(tau / t_descent)};
            }
            return {0.0, 0.0, z_final - final_approach_.distanceAt(tau - t_descent)};
        }
    }
}

Status Land::update(const Context & ctx, UavCommand & cmd)
{
    cmd.yaw_from_reference = true;
    const Vec3 rel = uavRelative(ctx);
    const Vec3 vrel = uavRelativeVelocity(ctx);

    switch (phase_) {
        case Phase::kApproach:
            if (ctx.t - t_phase_ >= params_.get("approach_duration", 8.0)) {
                phase_ = Phase::kTrack;
                t_phase_ = ctx.t;
                settle_.reset();
            }
            break;
        case Phase::kTrack: {
            // horizontal tolerance only: the height reference is fixed in the world and ignores the heave
            const bool ok = norm2(rel[0], rel[1]) < params_.get("tolerance", 0.15) &&
                            norm2(vrel[0], vrel[1]) < params_.get("velocity_tolerance", 0.15);
            const bool settled = settle_.update(ok, ctx.t, params_.get("settle", 3.0));
            const bool timeout = ctx.t - t_phase_ > params_.get("track_timeout", 40.0);
            if (settled || timeout) {
                cmd.message = settled ? "centred above the pad, descending with the landing pull"
                                      : "centring tolerance not reached within the timeout, descending anyway";
                phase_ = Phase::kDescend;
                t_phase_ = ctx.t;
                settle_.reset();
            }
            break;
        }
        case Phase::kDescend: {
            // contact: height above the current (heaving) winch position back to the resting height
            const double height = ctx.uav.p[2] - ctx.boat.anchor[2];
            const bool contact = height < rest_height_ + params_.get("touch_margin", 0.05) &&
                                 std::abs(vrel[2]) < params_.get("touch_velocity", 0.1);
            if (settle_.update(contact, ctx.t, params_.get("touch_time", 0.5))) {
                cmd.message = fmt("touchdown, disarming; horizontal offset from the pad centre [m]: ",
                                  norm2(ctx.uav.p[0] - ctx.boat.anchor[0], ctx.uav.p[1] - ctx.boat.anchor[1]));
                phase_ = Phase::kTouch;
                t_phase_ = ctx.t;
            }
            break;
        }
        case Phase::kTouch:
            if (!ctx.uav.armed) {
                cmd.message = "disarmed on the pad";
                phase_ = Phase::kLanded;
            } else if (ctx.t - t_phase_ > params_.get("disarm_timeout", 5.0)) {
                cmd.message = "not disarmed within the timeout";
                cmd.tether_pull = true;
                cmd.path = boatRelativePath(ctx, [this](double t) { return relAt(t); });
                return Status::kFailed;
            }
            break;
        case Phase::kLanded:
            break;
    }

    cmd.tether_pull = phase_ == Phase::kDescend || phase_ == Phase::kTouch || phase_ == Phase::kLanded;
    cmd.low_altitude = cmd.tether_pull;
    cmd.request_disarm = phase_ == Phase::kTouch;
    cmd.path = boatRelativePath(ctx, [this](double t) { return relAt(t); });
    return phase_ == Phase::kLanded ? Status::kDone : Status::kRunning;
}

}  // namespace mission_maneuvers
