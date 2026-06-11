#ifndef MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_
#define MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_

#include <array>
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

/// @brief Full kinematic state of one tether attachment body.
///
/// All positions are in the world frame.
/// Velocities and orientation are in the body frame (as reported by odometry).
/// The pipeline is responsible for computing the world-frame velocity
/// at the anchor point from this raw data.
struct BodyState
{
    double anchor_pos[3]{};           // attachment point position  (world frame, from TF)
    double body_center_pos[3]{};      // body CoM / base_link pos   (world frame, from TF)
    double quat[4]{0.0, 0.0, 0.0, 1.0}; // orientation [qx, qy, qz, qw]
    double lin_vel_body[3]{};         // linear  velocity            (body frame, from odometry)
    double ang_vel_body[3]{};         // angular velocity            (body frame, from odometry)
};

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

    /// @brief Initialize MoorDyn from the initial body states.
    ///        Initial velocities are assumed to be zero at startup.
    /// @param body_states Array of [boat, drone] kinematic states.
    /// @return true on success.
    bool initialize(const std::array<BodyState, 2> & body_states);

    /// @brief Advance the physics simulation by dt seconds.
    /// @param body_states Current [boat, drone] kinematic states.
    /// @param dt Time step in seconds.
    /// @param[out] out_forces Resulting forces per coupled DOF (size 6: boat xyz, drone xyz).
    /// @param[out] out_cable_nodes 3-D positions of internal cable nodes.
    /// @return true on success.
    bool step(const std::array<BodyState, 2> & body_states,
              double dt,
              std::vector<double> & out_forces,
              std::vector<std::vector<double>> & out_cable_nodes);

    /// @return true if the MoorDyn system handle was created successfully.
    bool isValid() const { return system_ != nullptr; }

private:
    // --- Math helpers (pure C++, no ROS, no Eigen) ---

    /// Rotate vector v from body to world frame using quaternion q = [qx,qy,qz,qw].
    static void rotateByQuat(const double q[4], const double v[3], double out[3]);

    /// Cross product: out = a × b.
    static void cross3(const double a[3], const double b[3], double out[3]);

    /// Compute the world-frame velocity of the anchor point from a full BodyState.
    /// vel_anchor = R(q)*v_lin + (R(q)*ω) × (anchor_pos - body_center_pos)
    static void computeAnchorVelocity(const BodyState & state, double vel_world[3]);

    /// Build a flat position vector [boat_anchor xyz, drone_anchor xyz] for MoorDyn.
    static void buildPositionVector(const std::array<BodyState, 2> & states, double pos[6]);

    /// Build a flat velocity vector [boat_anchor xyz, drone_anchor xyz] for MoorDyn.
    static void buildVelocityVector(const std::array<BodyState, 2> & states, double vel[6]);

    // --- Logging ---
    void log(LogLevel level, const std::string & msg) const;

    MoorDyn    system_{nullptr};
    LogCallback log_cb_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__TETHER_MOORDYN_PIPELINE_HPP_
