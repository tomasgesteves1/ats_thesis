#ifndef MISSION_EXECUTOR__MISSION_EXECUTOR_SEQUENCE_HPP_
#define MISSION_EXECUTOR__MISSION_EXECUTOR_SEQUENCE_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "mission_executor/mission_executor_loader.hpp"
#include "mission_maneuvers/mission_maneuvers_types.hpp"

namespace mission_executor
{

/// Runs the maneuvers of one vehicle in order. A step whose 'after' names a step not yet done waits, and the
/// previous maneuver keeps producing the reference meanwhile; the last maneuver keeps producing it once the
/// sequence is finished. A failed maneuver stops the sequence (its last command is kept).
template <class ManeuverT, class CommandT>
class Sequence
{
public:
    explicit Sequence(std::string vehicle) : vehicle_(std::move(vehicle)) {}

    void add(const StepSpec & spec, std::unique_ptr<ManeuverT> maneuver)
    {
        steps_.push_back({spec, std::move(maneuver)});
    }

    /// Advances the sequence; returns false when no maneuver is active yet (the caller provides an idle command).
    bool tick(const mission_maneuvers::Context & ctx, std::set<std::string> & done, CommandT & cmd,
              std::vector<std::string> & events)
    {
        if (!finished_ && !failed_ && !started_ && index_ < steps_.size()) {
            const auto & spec = steps_[index_].spec;
            if (spec.after.empty() || done.count(spec.after) > 0) {
                steps_[index_].maneuver->start(ctx);
                started_ = true;
                active_ = static_cast<int>(index_);
                events.push_back(vehicle_ + "." + spec.id + ": started (" + spec.type + ")");
            }
        }
        if (active_ < 0) {
            return false;
        }
        auto & step = steps_[static_cast<size_t>(active_)];
        const auto status = step.maneuver->update(ctx, cmd);
        if (!cmd.message.empty()) {
            events.push_back(vehicle_ + "." + step.spec.id + ": " + cmd.message);
        }
        const bool current = started_ && static_cast<size_t>(active_) == index_ && !finished_ && !failed_;
        if (current && status == mission_maneuvers::Status::kDone) {
            done.insert(vehicle_ + "." + step.spec.id);
            events.push_back(vehicle_ + "." + step.spec.id + ": done");
            if (index_ + 1 < steps_.size()) {
                ++index_;
                started_ = false;
            } else {
                finished_ = true;
            }
        } else if (current && status == mission_maneuvers::Status::kFailed) {
            failed_ = true;
            events.push_back(vehicle_ + "." + step.spec.id + ": FAILED, sequence stopped");
        }
        return true;
    }

    bool finished() const { return finished_ || steps_.empty(); }
    bool failed() const { return failed_; }

private:
    struct Step
    {
        StepSpec spec;
        std::unique_ptr<ManeuverT> maneuver;
    };
    std::string vehicle_;
    std::vector<Step> steps_;
    size_t index_{0};
    int active_{-1};
    bool started_{false};
    bool finished_{false};
    bool failed_{false};
};

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__MISSION_EXECUTOR_SEQUENCE_HPP_
