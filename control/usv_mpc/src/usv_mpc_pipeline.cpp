#include "usv_mpc/usv_mpc_pipeline.hpp"
#include "acados_solver_usv_dynamic.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace usv_mpc {

namespace {
constexpr int NX = 10;       // [x, y, psi, u, v, r, T_L, T_R, alpha_L, alpha_R]
constexpr int NU = 4;        // [dT_L, dT_R, dalpha_L, dalpha_R]
constexpr int NY = 14;       // stage residuals
constexpr int NY_E = 10;     // terminal residuals
constexpr int NP = 7;        // [psi_ref, x_a, y_a, z_a, z_t, x_ref, y_ref]
constexpr double TS = 0.1;   // Sampling time (must match the generator config)
constexpr double NO_TETHER = 1.0e6;
}

UsvMpcPipeline::UsvMpcPipeline() {
    current_state_.resize(6, 0.0);
    current_reference_.resize(6, 0.0);
    trajectory_type_ = 0; // Default: HOLD
    last_command_ = {0.0, 0.0, 0.0, 0.0};
    d_max_ = 15.0;
    z_t_ = 0.0;
    has_valid_solution_ = false;
    first_run_ = true;

    acados_ocp_capsule_ = usv_dynamic_acados_create_capsule();
    int status = usv_dynamic_acados_create((usv_dynamic_solver_capsule*)acados_ocp_capsule_);
    if (status) {
        std::cerr << "UsvMpcPipeline: Failed to create the ACADOS solver!" << std::endl;
    }
}

UsvMpcPipeline::~UsvMpcPipeline() {
    if (acados_ocp_capsule_) {
        usv_dynamic_acados_free((usv_dynamic_solver_capsule*)acados_ocp_capsule_);
        usv_dynamic_acados_free_capsule((usv_dynamic_solver_capsule*)acados_ocp_capsule_);
    }
}

void UsvMpcPipeline::updateState(const std::vector<double>& state) {
    if (state.size() == 6) {
        current_state_ = state;
    }
}

void UsvMpcPipeline::setReference(const std::vector<double>& ref_pose) {
    if (ref_pose.size() >= 3) {
        current_reference_.resize(6, 0.0);
        current_reference_[0] = ref_pose[0];
        current_reference_[1] = ref_pose[1];
        current_reference_[2] = ref_pose[2];
        if (ref_pose.size() == 6) {
            current_reference_[3] = ref_pose[3];
            current_reference_[4] = ref_pose[4];
            current_reference_[5] = ref_pose[5];
        }
        trajectory_type_ = 0; // Override to HOLD/STATIC
    }
}

void UsvMpcPipeline::setExternalReferencePath(const std::vector<TrajectoryPoint>& path) {
    external_reference_path_ = path;
}

void UsvMpcPipeline::setTrajectoryType(int type) {
    trajectory_type_ = type;
}

void UsvMpcPipeline::setUavHorizon(const std::vector<std::array<double, 3>>& uav_horizon) {
    uav_horizon_ = uav_horizon;
}

void UsvMpcPipeline::setTetherParameters(double d_max, double z_t) {
    d_max_ = d_max;
    z_t_ = z_t;
}

ThrusterCommand UsvMpcPipeline::computeControl() {
    auto capsule = (usv_dynamic_solver_capsule*)acados_ocp_capsule_;
    const int N = USV_DYNAMIC_N;

    // 1. Initial state: measured motion + last applied thruster commands
    double x0[NX];
    for (int i = 0; i < 6; i++) x0[i] = current_state_[i];
    for (int i = 0; i < 4; i++) x0[6 + i] = last_command_[i];
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    if (first_run_) {
        for (int i = 0; i <= N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x0);
        }
        first_run_ = false;
    }

    // 2. Reference over the horizon: yref = [0 (position residual), 0 (heading residual), u, v, r, 0 (T, alpha),
    // 0 (rates)]; the position and heading references are parameters of each node
    const bool external = (trajectory_type_ == 1 && !external_reference_path_.empty());
    const int path_size = static_cast<int>(external_reference_path_.size());
    for (int i = 0; i <= N; i++) {
        double px, py, psi_ref, u_ref = 0.0, r_ref = 0.0;
        if (external) {
            const auto& pt = external_reference_path_[std::min(i, path_size - 1)];
            px = pt.x; py = pt.y; psi_ref = pt.psi; u_ref = pt.v; r_ref = pt.w;
        } else {
            px = current_reference_[0]; py = current_reference_[1]; psi_ref = current_reference_[2];
        }

        double yref[NY] = {0.0, 0.0, 0.0, u_ref, 0.0, r_ref, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);

        // 3. Parameters: heading reference, UAV predicted position, attachment height and position reference
        const bool tether_active = i < static_cast<int>(uav_horizon_.size());
        const auto& pa = tether_active ? uav_horizon_[i] : std::array<double, 3>{0.0, 0.0, z_t_};
        double p[NP] = {psi_ref, pa[0], pa[1], pa[2], z_t_, px, py};
        usv_dynamic_acados_update_params(capsule, i, p, NP);

        // Tether bound d^2 <= d_max^2 on the nodes covered by the UAV horizon (nodes 1..N_c); inactive elsewhere
        if (i >= 1) {
            double uh = tether_active ? d_max_ * d_max_ : NO_TETHER;
            ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "uh", &uh);
        }
    }

    // 4. Solve OCP. A failed QP leaves the iterate unusable for the next warm start, so on failure the solver
    // memory is reset, the guess is re-initialised at the current state and the problem is solved once more
    int status = usv_dynamic_acados_solve(capsule);
    if (status != 0 && status != 2) {
        usv_dynamic_acados_reset(capsule, 1);
        double u0[NU] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i <= N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x0);
            if (i < N) ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u0);
        }
        std::cerr << "UsvMpcPipeline: ACADOS solver failed with status " << status << ", solver reset" << std::endl;
        status = usv_dynamic_acados_solve(capsule);
    }

    if (status == 0 || status == 2) {
        // Integrate the first optimal rate over one sampling interval (Euler, rates held constant)
        double u_opt[NU] = {0.0, 0.0, 0.0, 0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", u_opt);
        for (int i = 0; i < 4; i++) last_command_[i] += u_opt[i] * TS;
        has_valid_solution_ = true;
    } else {
        std::cerr << "UsvMpcPipeline: ACADOS solver failed with status: " << status << std::endl;
        has_valid_solution_ = false; // keep the last applied command
    }

    return last_command_;
}

std::vector<std::vector<double>> UsvMpcPipeline::getPredictedStates() {
    auto capsule = (usv_dynamic_solver_capsule*)acados_ocp_capsule_;
    const int N = USV_DYNAMIC_N;

    if (has_valid_solution_) {
        std::vector<std::vector<double>> trajectory;
        trajectory.reserve(N + 1);
        for (int i = 0; i <= N; i++) {
            double x_pred[NX] = {0.0};
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_pred);
            trajectory.emplace_back(x_pred, x_pred + NX);
        }
        last_trajectory_ = trajectory;
        has_valid_solution_ = false;
    } else if (last_trajectory_.empty()) {
        return std::vector<std::vector<double>>(N + 1, std::vector<double>(NX, 0.0));
    }

    return last_trajectory_;
}

} // namespace usv_mpc
