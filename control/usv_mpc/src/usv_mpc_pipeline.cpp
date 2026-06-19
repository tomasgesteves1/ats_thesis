#include "usv_mpc/usv_mpc_pipeline.hpp"
#include "acados_solver_usv_kinematic.h"
#include <iostream>
#include <cmath>

namespace usv_mpc {

UsvMpcPipeline::UsvMpcPipeline() {
    current_state_.resize(3, 0.0);
    current_reference_.resize(3, 0.0);
    trajectory_type_ = 0; // Default: HOLD
    
    acados_ocp_capsule_ = usv_kinematic_acados_create_capsule();
    int status = usv_kinematic_acados_create((usv_kinematic_solver_capsule*)acados_ocp_capsule_);
    if (status) {
        std::cerr << "UsvMpcPipeline: Falha a criar o solver ACADOS!" << std::endl;
    }
}

UsvMpcPipeline::~UsvMpcPipeline() {
    if (acados_ocp_capsule_) {
        usv_kinematic_acados_free((usv_kinematic_solver_capsule*)acados_ocp_capsule_);
        usv_kinematic_acados_free_capsule((usv_kinematic_solver_capsule*)acados_ocp_capsule_);
    }
}

void UsvMpcPipeline::updateState(const std::vector<double>& state) {
    if (state.size() == 3) {
        current_state_ = state;
    }
}

void UsvMpcPipeline::setReference(const std::vector<double>& ref_pose) {
    if (ref_pose.size() == 3) {
        current_reference_ = ref_pose;
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
    auto capsule = (usv_kinematic_solver_capsule*)acados_ocp_capsule_;
    
    // 1. Set initial state bounds (x0)
    double x0[3];
    for (int i = 0; i < 3; i++) x0[i] = current_state_[i];
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // 2. Inject reference trajectory over horizon
    int N = USV_KINEMATIC_N;
    if (trajectory_type_ == 1 && !external_reference_path_.empty()) {
        // Track external dynamic trajectory
        int path_size = external_reference_path_.size();
        for (int i = 0; i < N; i++) {
            // Get corresponding point, pad with last point if horizon extends past path
            int idx = std::min(i, path_size - 1);
            const auto& pt = external_reference_path_[idx];
            
            // intermediate yref: [x, y, psi, v, w]
            double yref[5] = {pt.x, pt.y, pt.psi, pt.v, pt.w};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        // Terminal stage yref_e: [x, y, psi]
        const auto& pt_e = external_reference_path_[std::min(N, path_size - 1)];
        double yref_e[3] = {pt_e.x, pt_e.y, pt_e.psi};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);

    } else {
        // Hold static reference pose
        double yref[5] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0};
        double yref_e[3] = {current_reference_[0], current_reference_[1], current_reference_[2]};
        
        for (int i = 0; i < N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);
    }

    // 3. Solve OCP
    int status = usv_kinematic_acados_solve(capsule);
    
    double u_opt[2] = {0.0, 0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", &u_opt);
    } else {
        std::cerr << "UsvMpcPipeline: ACADOS solver falhou com status: " << status << std::endl;
    }

    return {u_opt[0], u_opt[1]};
}

} // namespace usv_mpc
