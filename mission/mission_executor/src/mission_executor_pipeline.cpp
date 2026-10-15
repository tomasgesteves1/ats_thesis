#include "mission_executor/mission_executor_pipeline.hpp"

namespace mission_executor
{

using mission_maneuvers::Params;

MissionExecutorPipeline::MissionExecutorPipeline(const PipelineConfig & config) : config_(config) {}

bool MissionExecutorPipeline::load(const MissionSpec & mission, std::string & error)
{
    uav_ = std::make_unique<Sequence<mission_maneuvers::UavManeuver, mission_maneuvers::UavCommand>>("uav");
    usv_ = std::make_unique<Sequence<mission_maneuvers::UsvManeuver, mission_maneuvers::UsvCommand>>("usv");
    for (const auto & step : mission.uav) {
        auto maneuver = mission_maneuvers::createUavManeuver(step.type, Params(step.params));
        if (!maneuver) {
            error = "unknown UAV maneuver '" + step.type + "' (step " + step.id + ")";
            return false;
        }
        uav_->add(step, std::move(maneuver));
    }
    for (const auto & step : mission.usv) {
        auto maneuver = mission_maneuvers::createUsvManeuver(step.type, Params(step.params));
        if (!maneuver) {
            error = "unknown USV maneuver '" + step.type + "' (step " + step.id + ")";
            return false;
        }
        usv_->add(step, std::move(maneuver));
    }
    abort_spec_ = mission.uav_abort;
    if (abort_spec_.empty()) {
        StepSpec land;
        land.id = "land";
        land.type = "land";
        abort_spec_.push_back(land);
    }
    for (const auto & step : abort_spec_) {
        if (!mission_maneuvers::createUavManeuver(step.type, Params(step.params))) {
            error = "unknown UAV maneuver '" + step.type + "' in uav_abort (step " + step.id + ")";
            return false;
        }
    }
    uav_abort_.reset();
    usv_abort_.reset();
    name_ = mission.name;
    done_.clear();
    state_ = MissionState::kRunning;
    return true;
}

mission_maneuvers::Context MissionExecutorPipeline::buildContext(const Measurements & m)
{
    // The references are given relative to the winch at its mean height: the boat heave is not followed
    anchor_z_sum_ += m.boat.anchor[2];
    ++anchor_z_count_;
    // Height of the UAV above the winch when it rests on the pad, taken before the first takeoff
    if (!rest_height_set_) {
        rest_height_ = m.uav.armed ? config_.default_rest_height : m.uav.p[2] - m.boat.anchor[2];
        rest_height_set_ = true;
    }
    mission_maneuvers::Context ctx;
    ctx.t = m.t;
    ctx.uav = m.uav;
    ctx.boat = m.boat;
    ctx.boat.anchor_z_mean = anchor_z_sum_ / static_cast<double>(anchor_z_count_);
    ctx.uav_rest_height = rest_height_;
    ctx.uav_horizon = config_.uav_horizon;
    ctx.usv_horizon = config_.usv_horizon;
    ctx.horizon_timeout = config_.horizon_timeout;
    return ctx;
}

void MissionExecutorPipeline::startAbort(const Measurements & m, Output & out)
{
    state_ = MissionState::kAborted;
    out.events.push_back("mission aborted: the boat holds its position");
    usv_abort_ = std::make_unique<Sequence<mission_maneuvers::UsvManeuver, mission_maneuvers::UsvCommand>>(
        "usv_abort");
    StepSpec hold;
    hold.id = "hold";
    hold.type = "hold";
    usv_abort_->add(hold, mission_maneuvers::createUsvManeuver("hold", Params()));
    if (m.uav.armed) {
        out.events.push_back("UAV armed: running the abort sequence");
        uav_abort_ = std::make_unique<Sequence<mission_maneuvers::UavManeuver, mission_maneuvers::UavCommand>>(
            "uav_abort");
        for (const auto & step : abort_spec_) {
            uav_abort_->add(step, mission_maneuvers::createUavManeuver(step.type, Params(step.params)));
        }
    }
}

Output MissionExecutorPipeline::tick(const Measurements & m)
{
    Output out;
    if (state_ == MissionState::kNotLoaded) {
        return out;
    }
    const auto ctx = buildContext(m);

    auto & uav_seq = uav_abort_ ? *uav_abort_ : *uav_;
    auto & usv_seq = usv_abort_ ? *usv_abort_ : *usv_;
    if (!uav_seq.tick(ctx, done_, out.uav, out.events)) {
        // no UAV maneuver yet: the reference stays at the current position
        mission_maneuvers::UavPose here{m.uav.p, m.uav.yaw};
        out.uav.path.assign(static_cast<size_t>(config_.uav_horizon.steps), here);
    }
    if (!usv_seq.tick(ctx, done_, out.usv, out.events)) {
        if (!usv_idle_set_) {
            usv_idle_ = {m.boat.p[0], m.boat.p[1], m.boat.yaw};
            usv_idle_set_ = true;
        }
        out.usv.path.assign(static_cast<size_t>(config_.usv_horizon.steps), usv_idle_);
    }

    if (state_ == MissionState::kRunning) {
        if (uav_->failed() || usv_->failed()) {
            startAbort(m, out);
        } else if (uav_->finished() && usv_->finished()) {
            state_ = MissionState::kDone;
            out.events.push_back("mission complete");
        }
    } else if (state_ == MissionState::kAborted && uav_abort_ && uav_abort_->finished() && !abort_reported_) {
        abort_reported_ = true;
        out.events.push_back("abort sequence complete");
    }
    return out;
}

}  // namespace mission_executor
