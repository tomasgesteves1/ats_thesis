#include "moordyn_tether/moordyn_tether_pipeline.hpp"

#include <cmath>
#include <filesystem>

namespace moordyn_tether
{

MoordynTetherPipeline::MoordynTetherPipeline(const std::string & config_file_path,
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

MoordynTetherPipeline::~MoordynTetherPipeline()
{
    if (system_)
    {
        MoorDyn_Close(system_);
    }
}

bool MoordynTetherPipeline::initialize(const std::array<BodyState, 2> & body_states)
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
    Kinematics::buildPositionVector(body_states, pos);

    line_ = MoorDyn_GetLine(system_, 1);
    if (!line_)
    {
        log(LogLevel::kWarn, "Could not obtain Line 1 handle — virtual winch disabled.");
    }

    if (winch_.getConfig().enabled && line_)
    {
        const double span = Kinematics::distance3(body_states[0].anchor_pos,
                                                  body_states[1].anchor_pos);
        winch_.update(line_, body_states, 0.0);
        
        log(LogLevel::kInfo,
            "Winch: initial span = " + std::to_string(span) +
            " m, setting UnstrLen before init.");
    }

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

bool MoordynTetherPipeline::step(const std::array<BodyState, 2> & body_states,
                                 double dt,
                                 std::vector<double> & out_forces,
                                 std::vector<std::vector<double>> & out_cable_nodes)
{
    if (!system_)
    {
        return false;
    }

    // Apply virtual winch before the physics step.
    winch_.update(line_, body_states, dt);

    double pos[6];
    double vel[6];
    Kinematics::buildPositionVector(body_states, pos);
    Kinematics::buildVelocityVector(body_states, vel);

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

    // Apply exponential moving average filter to reduce noise/spikes.
    force_filter_.apply(out_forces, out_forces);

    // Extract cable geometry (Line 1, 1-indexed in MoorDyn)
    out_cable_nodes.clear();
    if (line_)
    {
        unsigned int n_nodes = 0;
        MoorDyn_GetLineNumberNodes(line_, &n_nodes);
        for (unsigned int i = 0; i < n_nodes; ++i)
        {
            double node_pos[3];
            MoorDyn_GetLineNodePos(line_, i, node_pos);
            // Add the 100.0 offset back for visualization
            out_cable_nodes.push_back({node_pos[0], node_pos[1], node_pos[2] + 100.0});
        }
    }

    return true;
}

void MoordynTetherPipeline::setWinchConfig(const WinchConfig & config)
{
    winch_.setConfig(config);
    log(LogLevel::kInfo,
        "Winch config updated — enabled: " + std::string(config.enabled ? "true" : "false") +
        ", slack_factor: " + std::to_string(config.slack_factor) +
        ", min: " + std::to_string(config.min_length) +
        ", max: " + std::to_string(config.max_length));
}

void MoordynTetherPipeline::setFilterAlpha(double alpha)
{
    force_filter_.init(alpha);
    log(LogLevel::kInfo, "Force filter alpha set to: " + std::to_string(alpha));
}

double MoordynTetherPipeline::getTetherLength() const
{
    if (!line_)
    {
        return 0.0;
    }
    double len = 0.0;
    MoorDyn_GetLineUnstretchedLength(line_, &len);
    return len;
}

void MoordynTetherPipeline::log(LogLevel level, const std::string & msg) const
{
    if (log_cb_)
    {
        log_cb_(level, msg);
    }
}

}  // namespace moordyn_tether
