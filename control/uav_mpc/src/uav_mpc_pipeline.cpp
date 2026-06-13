#include "uav_mpc/uav_mpc_pipeline.hpp"
#include "acados_solver_uav_tethered.h"
#include <iostream>

namespace uav_mpc {

UavMpcPipeline::UavMpcPipeline() {
    current_state_.resize(6, 0.0);
    
    acados_ocp_capsule_ = uav_tethered_acados_create_capsule();
    int status = uav_tethered_acados_create((uav_tethered_solver_capsule*)acados_ocp_capsule_);
    if (status) {
        std::cerr << "UavMpcPipeline: Falha a criar o solver ACADOS!" << std::endl;
    }
}

UavMpcPipeline::~UavMpcPipeline() {
    if (acados_ocp_capsule_) {
        uav_tethered_acados_free((uav_tethered_solver_capsule*)acados_ocp_capsule_);
        uav_tethered_acados_free_capsule((uav_tethered_solver_capsule*)acados_ocp_capsule_);
    }
}

void UavMpcPipeline::updateState(const std::vector<double>& state) {
    if (state.size() == 6) {
        current_state_ = state;
    }
}

void UavMpcPipeline::setReference(const std::vector<double>& ref) {
    if (ref.size() == 3) {
        auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
        // yref = [p_x, p_y, p_z, v_x, v_y, v_z, u_x, u_y, u_z]
        // Control z reference is 9.81 to counteract gravity in steady state!
        double yref[9] = {ref[0], ref[1], ref[2], 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
        double yref_e[6] = {ref[0], ref[1], ref[2], 0.0, 0.0, 0.0};
        
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);
    }
}

std::vector<double> UavMpcPipeline::computeControl() {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    
    // Injetar estado atual (x0) no ACADOS
    double x0[6];
    for(int i=0; i<6; i++) x0[i] = current_state_[i];
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // Resolver
    int status = uav_tethered_acados_solve(capsule);
    
    // Extrair controlo (u_opt)
    double u_opt[3] = {0.0, 0.0, 0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", &u_opt);
    } else {
        std::cerr << "UavMpcPipeline: ACADOS solver falhou com status: " << status << std::endl;
    }

    return {u_opt[0], u_opt[1], u_opt[2]};
}

} // namespace uav_mpc
