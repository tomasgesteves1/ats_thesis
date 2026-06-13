#include "usv_mpc/usv_mpc_pipeline.hpp"
#include "acados_solver_usv_kinematic.h"
#include <iostream>

namespace usv_mpc {

UsvMpcPipeline::UsvMpcPipeline() {
    current_state_.resize(3, 0.0);
    
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

std::vector<double> UsvMpcPipeline::computeControl() {
    auto capsule = (usv_kinematic_solver_capsule*)acados_ocp_capsule_;
    
    double x0[3];
    for(int i=0; i<3; i++) x0[i] = current_state_[i];
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

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
