#pragma once

#include <array>
#include <vector>

namespace usv_mpc {

struct TrajectoryPoint {
    double x;
    double y;
    double psi;
    double v;
    double w;
};

/// Thruster commands [T_L, T_R, alpha_L, alpha_R] (N, N, rad, rad)
using ThrusterCommand = std::array<double, 4>;

class UsvMpcPipeline {
public:
    UsvMpcPipeline();
    ~UsvMpcPipeline();

    void updateState(const std::vector<double>& state);
    void setReference(const std::vector<double>& ref_pose);
    void setExternalReferencePath(const std::vector<TrajectoryPoint>& path);
    void setTrajectoryType(int type); // 0: HOLD, 1: EXTERNAL

    /// Tether coupling: UAV predicted positions resampled at the USV prediction nodes
    /// (index i = node i, size N_c + 1). An empty horizon disables the tether constraint.
    void setUavHorizon(const std::vector<std::array<double, 3>>& uav_horizon);
    void setTetherParameters(double d_max, double z_t);

    /// Solves the OCP and returns the thruster commands integrated over one sampling interval.
    ThrusterCommand computeControl();
    std::vector<std::vector<double>> getPredictedStates();

private:
    std::vector<double> current_state_;
    std::vector<double> current_reference_;
    std::vector<TrajectoryPoint> external_reference_path_;
    int trajectory_type_; // 0: HOLD, 1: EXTERNAL
    ThrusterCommand last_command_;
    std::vector<std::array<double, 3>> uav_horizon_;
    double d_max_;
    double z_t_;
    std::vector<std::vector<double>> last_trajectory_;
    bool has_valid_solution_;
    bool first_run_;
    void* acados_ocp_capsule_;
};

} // namespace usv_mpc
