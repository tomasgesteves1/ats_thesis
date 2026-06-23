#include "usv_mpc/usv_mpc_pipeline.hpp"
#include "acados_solver_usv_dynamic.h"
#include <iostream>
#include <cmath>

namespace usv_mpc {

UsvMpcPipeline::UsvMpcPipeline() {
    current_state_.resize(6, 0.0);
    current_reference_.resize(6, 0.0);
    trajectory_type_ = 0; // Default: HOLD
    
    acados_ocp_capsule_ = usv_dynamic_acados_create_capsule();
    int status = usv_dynamic_acados_create((usv_dynamic_solver_capsule*)acados_ocp_capsule_);
    if (status) {
        std::cerr << "UsvMpcPipeline: Falha a criar o solver ACADOS!" << std::endl;
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

std::vector<double> UsvMpcPipeline::computeControl() {
    auto capsule = (usv_dynamic_solver_capsule*)acados_ocp_capsule_;
    
    // 1. Set initial state bounds (x0)
    double x0[6];
    for (int i = 0; i < 6; i++) x0[i] = current_state_[i];
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // 2. Inject reference trajectory over horizon
    int N = USV_DYNAMIC_N;
    double current_yaw = current_state_[2];
    double prev_yaw = current_yaw;

    // Hydrodynamic damping coefficients for feedforward drag compensation
    const double X_U = 165.97;
    const double X_UU = 140.00;
    const double X_UU_BWD = 157.68;
    const double N_R = 400.76;
    const double N_RR = 1069.15;

    if (trajectory_type_ == 1 && !external_reference_path_.empty()) {
        // Track external dynamic trajectory
        int path_size = external_reference_path_.size();
        for (int i = 0; i < N; i++) {
            // Get corresponding point, pad with last point if horizon extends past path
            int idx = std::min(i, path_size - 1);
            const auto& pt = external_reference_path_[idx];
            
            // Continuous angle unwrapping to prevent 180-degree jumps on -pi/pi boundary
            double target_psi = pt.psi;
            double diff = target_psi - prev_yaw;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            target_psi = prev_yaw + diff;
            prev_yaw = target_psi;
            
            // Feedforward drag compensation
            double u_ref = pt.v;
            double r_ref = pt.w;
            double x_uu_eff = (u_ref < 0.0) ? X_UU_BWD : X_UU;
            double X_ff = (X_U + x_uu_eff * std::abs(u_ref)) * u_ref;
            double N_ff = (N_R + N_RR * std::abs(r_ref)) * r_ref;
            
            // intermediate yref (size 9): [x, y, psi, u, v, r, X, Y, N]
            double yref[9] = {pt.x, pt.y, target_psi, u_ref, 0.0, r_ref, X_ff, 0.0, N_ff};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        // Terminal stage yref_e (size 6): [x, y, psi, u, v, r]
        const auto& pt_e = external_reference_path_[std::min(N, path_size - 1)];
        double target_psi_e = pt_e.psi;
        double diff_e = target_psi_e - prev_yaw;
        while (diff_e > M_PI) diff_e -= 2.0 * M_PI;
        while (diff_e < -M_PI) diff_e += 2.0 * M_PI;
        target_psi_e = prev_yaw + diff_e;

        double yref_e[6] = {pt_e.x, pt_e.y, target_psi_e, pt_e.v, 0.0, pt_e.w};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);

    } else {
        // Hold static reference pose with angle unwrapping
        double target_psi = current_reference_[2];
        double diff = target_psi - current_yaw;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        target_psi = current_yaw + diff;

        double yref[9] = {current_reference_[0], current_reference_[1], target_psi, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double yref_e[6] = {current_reference_[0], current_reference_[1], target_psi, 0.0, 0.0, 0.0};
        
        for (int i = 0; i < N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);
    }

    // 3. Solve OCP
    int status = usv_dynamic_acados_solve(capsule);
    
    double u_opt[3] = {0.0, 0.0, 0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", &u_opt);
    } else {
        std::cerr << "UsvMpcPipeline: ACADOS solver falhou com status: " << status << std::endl;
    }

    return {u_opt[0], u_opt[1], u_opt[2]};
}

std::vector<std::vector<double>> UsvMpcPipeline::getPredictedStates() {
    auto capsule = (usv_dynamic_solver_capsule*)acados_ocp_capsule_;
    int N = USV_DYNAMIC_N;
    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(N + 1);
    
    for (int i = 0; i <= N; i++) {
        double x_pred[6] = {0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", &x_pred);
        trajectory.push_back({x_pred[0], x_pred[1], x_pred[2], x_pred[3], x_pred[4], x_pred[5]});
    }
    return trajectory;
}

} // namespace usv_mpc
