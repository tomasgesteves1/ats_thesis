#ifndef MOORDYN_TETHER__MOORDYN_TETHER_PIPELINE_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_PIPELINE_HPP_

#include <array>
#include <functional>
#include <string>
#include <vector>
#include "moordyn/MoorDyn2.h"

#include "moordyn_tether/moordyn_tether_kinematics.hpp"
#include "moordyn_tether/moordyn_tether_winch.hpp"
#include "moordyn_tether/moordyn_tether_filter.hpp"

namespace moordyn_tether
{

/// Log severity levels for the pipeline callback.
enum class LogLevel { kInfo, kWarn, kError };

/// Callback type for logging from the pure C++ pipeline.
using LogCallback = std::function<void(LogLevel, const std::string &)>;

/// @brief Pure C++ wrapper around MoorDyn for tether physics simulation.
///
/// This class has NO dependency on ROS. Funciona estritamente como orquestrador.
class MoordynTetherPipeline
{
public:
    explicit MoordynTetherPipeline(const std::string & config_file_path,
                                   LogCallback log_cb = {});
    ~MoordynTetherPipeline();

    // Non-copyable
    MoordynTetherPipeline(const MoordynTetherPipeline &) = delete;
    MoordynTetherPipeline & operator=(const MoordynTetherPipeline &) = delete;

    bool initialize(const std::array<BodyState, 2> & body_states);

    void setWinchConfig(const WinchConfig & config);

    void setFilterAlpha(double alpha);

    bool step(const std::array<BodyState, 2> & body_states,
              double dt,
              std::vector<double> & out_forces,
              std::vector<std::vector<double>> & out_cable_nodes);

    bool isValid() const { return system_ != nullptr; }

    double getTetherLength() const;

private:
    void log(LogLevel level, const std::string & msg) const;

    MoorDyn      system_{nullptr};
    MoorDynLine  line_{nullptr};     ///< Cached handle to Line 1.
    VirtualWinch winch_;             ///< Winch logic delegator.
    ForceFilter  force_filter_;      ///< Noise filter for forces.
    std::vector<double> last_forces_;///< Forces from the previous physics step.
    LogCallback  log_cb_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_PIPELINE_HPP_
