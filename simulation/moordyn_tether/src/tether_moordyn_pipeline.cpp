#include "moordyn_tether/tether_moordyn_pipeline.hpp"

#include <filesystem>

namespace moordyn_tether
{

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

bool TetherMoorDynPipeline::initialize(const std::vector<double> & initial_positions)
{
    if (!system_)
    {
        log(LogLevel::kError, "Cannot initialize: MoorDyn system is NULL.");
        return false;
    }

    unsigned int n_dof = 0;
    MoorDyn_NCoupledDOF(system_, &n_dof);
    if (initial_positions.size() != n_dof)
    {
        log(LogLevel::kError,
            "Position vector size (" + std::to_string(initial_positions.size()) +
            ") does not match expected DOF (" + std::to_string(n_dof) + ").");
        return false;
    }

    std::vector<double> initial_velocities(initial_positions.size(), 0.0);
    int result = MoorDyn_Init(system_,
                              initial_positions.data(),
                              initial_velocities.data());

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

bool TetherMoorDynPipeline::step(const std::vector<double> & current_positions,
                                 double dt,
                                 std::vector<double> & out_forces,
                                 std::vector<std::vector<double>> & out_cable_nodes)
{
    if (!system_)
    {
        return false;
    }

    out_forces.resize(current_positions.size());

    // Zero velocities for now.
    // TODO(tomas): pass real body velocities for accurate hydrodynamic damping.
    std::vector<double> current_velocities(current_positions.size(), 0.0);

    double time = 0.0;
    double step_dt = dt;

    int result = MoorDyn_Step(system_,
                              current_positions.data(),
                              current_velocities.data(),
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
            double pos[3];
            MoorDyn_GetLineNodePos(line, i, pos);
            out_cable_nodes.push_back({pos[0], pos[1], pos[2]});
        }
    }

    return true;
}

void TetherMoorDynPipeline::log(LogLevel level, const std::string & msg) const
{
    if (log_cb_)
    {
        log_cb_(level, msg);
    }
}

}  // namespace moordyn_tether
