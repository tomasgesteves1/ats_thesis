#ifndef MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_
#define MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_

#include <functional>
#include <string>
#include <vector>
#include "moordyn/MoorDyn2.h"

namespace moordyn_tether
{

/// Log severity levels for the pipeline callback.
enum class LogLevel { kInfo, kWarn, kError };

/// Callback type for logging from the pure C++ pipeline.
using LogCallback = std::function<void(LogLevel, const std::string &)>;

/// @brief Pure C++ wrapper around MoorDyn for tether physics simulation.
///
/// This class has NO dependency on ROS.  All logging is routed through
/// an optional callback so that the ROS wrapper can bridge it to RCLCPP macros.
class TetherMoorDynPipeline
{
public:
    /// @brief Construct the MoorDyn physics pipeline.
    /// @param config_file_path Absolute path to the MoorDyn lines.txt file.
    /// @param log_cb Optional logging callback (defaults to no-op).
    explicit TetherMoorDynPipeline(const std::string & config_file_path,
                                   LogCallback log_cb = {});
    ~TetherMoorDynPipeline();

    // Non-copyable
    TetherMoorDynPipeline(const TetherMoorDynPipeline &) = delete;
    TetherMoorDynPipeline & operator=(const TetherMoorDynPipeline &) = delete;

    /// @brief Initialize MoorDyn with the given coupled point positions.
    /// @param initial_positions Flat vector [x0,y0,z0, x1,y1,z1, ...].
    /// @return true on success.
    bool initialize(const std::vector<double> & initial_positions);

    /// @brief Advance the physics simulation by dt seconds.
    /// @param current_positions Current coupled-point positions.
    /// @param dt Time step in seconds.
    /// @param[out] out_forces Resulting forces per coupled DOF.
    /// @param[out] out_cable_nodes 3-D positions of internal cable nodes.
    /// @return true on success.
    bool step(const std::vector<double> & current_positions,
              double dt,
              std::vector<double> & out_forces,
              std::vector<std::vector<double>> & out_cable_nodes);

    /// @return true if the MoorDyn system handle was created successfully.
    bool isValid() const { return system_ != nullptr; }

private:
    void log(LogLevel level, const std::string & msg) const;

    MoorDyn system_{nullptr};
    LogCallback log_cb_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_
