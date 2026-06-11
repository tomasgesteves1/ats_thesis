#include "moordyn_tether/tether_moordyn_pipeline.hpp"

#include <cmath>
#include <filesystem>

namespace moordyn_tether
{

// ===========================================================================
// Constructor / Destructor
// ===========================================================================

TetherMoorDynPipeline::TetherMoorDynPipeline(const std::string & config_file_path,
                                             LogCallback log_cb)
    : log_cb_(std::move(log_cb))
{
    log(LogLevel::kInfo, "Loading MoorDyn config from: " + config_file_path);

    if (!std::filesystem::exists(config_file_path))
    {
        log(LogLevel::kError, "MoorDyn config file not found: " + config_file_path);
        return;
    }

    system_ = MoorDyn_Create(config_file_path.c_str());
    if (!system_)
    {
        log(LogLevel::kError, "MoorDyn_Create FAILED. Check MoorDyn installation.");
    }
    else
    {
        unsigned int n_dof = 0;
        MoorDyn_NCoupledDOF(system_, &n_dof);
        log(LogLevel::kInfo,
            "MoorDyn system created successfully. Expecting " +
            std::to_string(n_dof) + " coupled DOF.");
    }
}

TetherMoorDynPipeline::~TetherMoorDynPipeline()
{
    if (system_)
    {
        MoorDyn_Close(system_);
    }
}

// ===========================================================================
// Public API
// ===========================================================================

bool TetherMoorDynPipeline::initialize(const std::array<BodyState, 2> & body_states)
{
    if (!system_)
    {
        log(LogLevel::kError, "Cannot initialize: MoorDyn system is NULL.");
        return false;
    }

    unsigned int n_dof = 0;
    MoorDyn_NCoupledDOF(system_, &n_dof);
    if (n_dof != 6)
    {
        log(LogLevel::kError,
            "Expected 6 coupled DOF (2 bodies × 3 axes), got " +
            std::to_string(n_dof) + ". Check lines.txt.");
        return false;
    }

    double pos[6];
    double vel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // zero velocity at init
    buildPositionVector(body_states, pos);

    int result = MoorDyn_Init(system_, pos, vel);
    if (result != MOORDYN_SUCCESS)
    {
        log(LogLevel::kError,
            "MoorDyn_Init FAILED with code " + std::to_string(result) +
            ". Invalid initial geometry?");
        return false;
    }

    log(LogLevel::kInfo, "MoorDyn initialized successfully.");
    return true;
}

bool TetherMoorDynPipeline::step(const std::array<BodyState, 2> & body_states,
                                 double dt,
                                 std::vector<double> & out_forces,
                                 std::vector<std::vector<double>> & out_cable_nodes)
{
    if (!system_)
    {
        return false;
    }

    double pos[6];
    double vel[6];
    buildPositionVector(body_states, pos);
    buildVelocityVector(body_states, vel);

    out_forces.resize(6);
    double time    = 0.0;
    double step_dt = dt;

    int result = MoorDyn_Step(system_,
                              pos,
                              vel,
                              out_forces.data(),
                              &time,
                              &step_dt);

    if (result != MOORDYN_SUCCESS)
    {
        log(LogLevel::kError,
            "MoorDyn_Step failed with code " + std::to_string(result));
        return false;
    }

    // Extract cable geometry (Line 1, 1-indexed in MoorDyn)
    out_cable_nodes.clear();
    MoorDynLine line = MoorDyn_GetLine(system_, 1);
    if (line)
    {
        unsigned int n_nodes = 0;
        MoorDyn_GetLineNumberNodes(line, &n_nodes);
        for (unsigned int i = 0; i < n_nodes; ++i)
        {
            double node_pos[3];
            MoorDyn_GetLineNodePos(line, i, node_pos);
            out_cable_nodes.push_back({node_pos[0], node_pos[1], node_pos[2]});
        }
    }

    return true;
}

// ===========================================================================
// Private math helpers (pure C++, no ROS, no Eigen)
// ===========================================================================

void TetherMoorDynPipeline::rotateByQuat(const double q[4],
                                         const double v[3],
                                         double out[3])
{
    // Active rotation of v by unit quaternion q = [qx, qy, qz, qw].
    // Uses the Rodrigues formula via cross products (avoids full matrix build):
    //   t      = 2 * (q_vec × v)
    //   v_rot  = v + qw * t + q_vec × t
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];

    const double tx = 2.0 * (qy * v[2] - qz * v[1]);
    const double ty = 2.0 * (qz * v[0] - qx * v[2]);
    const double tz = 2.0 * (qx * v[1] - qy * v[0]);

    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

void TetherMoorDynPipeline::cross3(const double a[3],
                                   const double b[3],
                                   double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void TetherMoorDynPipeline::computeAnchorVelocity(const BodyState & state,
                                                   double vel_world[3])
{
    // Step 1: Rotate body-frame linear velocity to world frame.
    //   v_lin_world = R(q) * v_lin_body
    double v_lin_world[3];
    rotateByQuat(state.quat, state.lin_vel_body, v_lin_world);

    // Step 2: Rotate body-frame angular velocity to world frame.
    //   ω_world = R(q) * ω_body
    double omega_world[3];
    rotateByQuat(state.quat, state.ang_vel_body, omega_world);

    // Step 3: Compute lever arm from body center to anchor point (world frame).
    //   r = anchor_pos - body_center_pos
    const double r[3] = {
        state.anchor_pos[0] - state.body_center_pos[0],
        state.anchor_pos[1] - state.body_center_pos[1],
        state.anchor_pos[2] - state.body_center_pos[2]
    };

    // Step 4: Angular velocity contribution: ω_world × r
    double ang_contrib[3];
    cross3(omega_world, r, ang_contrib);

    // Step 5: Total anchor velocity in world frame.
    //   v_anchor = v_lin_world + ω_world × r
    vel_world[0] = v_lin_world[0] + ang_contrib[0];
    vel_world[1] = v_lin_world[1] + ang_contrib[1];
    vel_world[2] = v_lin_world[2] + ang_contrib[2];
}

void TetherMoorDynPipeline::buildPositionVector(
    const std::array<BodyState, 2> & states, double pos[6])
{
    pos[0] = states[0].anchor_pos[0];
    pos[1] = states[0].anchor_pos[1];
    pos[2] = states[0].anchor_pos[2];
    pos[3] = states[1].anchor_pos[0];
    pos[4] = states[1].anchor_pos[1];
    pos[5] = states[1].anchor_pos[2];
}

void TetherMoorDynPipeline::buildVelocityVector(
    const std::array<BodyState, 2> & states, double vel[6])
{
    double v_boat[3];
    computeAnchorVelocity(states[0], v_boat);
    vel[0] = v_boat[0];
    vel[1] = v_boat[1];
    vel[2] = v_boat[2];

    double v_drone[3];
    computeAnchorVelocity(states[1], v_drone);
    vel[3] = v_drone[0];
    vel[4] = v_drone[1];
    vel[5] = v_drone[2];
}

// ===========================================================================
// Logging
// ===========================================================================

void TetherMoorDynPipeline::log(LogLevel level, const std::string & msg) const
{
    if (log_cb_)
    {
        log_cb_(level, msg);
    }
}

}  // namespace moordyn_tether
