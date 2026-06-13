#include "uav_mpc/uav_mpc_pipeline.hpp"
#include "acados_solver_uav_tethered.h"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace uav_mpc {

UavMpcPipeline::UavMpcPipeline() 
    : qx_(0.0), qy_(0.0), qz_(0.0), qw_(1.0),
      target_yaw_(0.0), target_initialized_(false) {
    current_state_.resize(6, 0.0);
    current_reference_.resize(3, 0.0);
    
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

void UavMpcPipeline::updateOrientation(double qx, double qy, double qz, double qw) {
    qx_ = qx;
    qy_ = qy;
    qz_ = qz;
    qw_ = qw;
}

void UavMpcPipeline::setReference(const std::vector<double>& ref) {
    if (ref.size() == 3) {
        current_reference_ = ref;
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

UavControlOutput UavMpcPipeline::computeControl() {
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

    // --- CÁLCULOS MATEMÁTICOS DE ATITUDE E FORÇA (Pure C++) ---
    UavControlOutput output;
    
    // Guardar yaw inicial se ainda não inicializado
    if (!target_initialized_) {
        double roll, pitch;
        quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, target_yaw_);
        target_initialized_ = true;
    }

    // Desired acceleration in world frame (ENU)
    double m = 2.06;
    double F_enu[3] = { u_opt[0] * m, u_opt[1] * m, u_opt[2] * m };

    double thrust_mag = std::sqrt(F_enu[0]*F_enu[0] + F_enu[1]*F_enu[1] + F_enu[2]*F_enu[2]);
    if (thrust_mag < 0.1) {
        thrust_mag = 0.1;
    }

    // 1. Converter vetor de força desejado de ENU para o referencial de mundo do PX4 (NED)
    double F_ned[3] = { F_enu[1], F_enu[0], -F_enu[2] };

    // 2. Extrair o eixo Z desejado do corpo (Z_body) no referencial NED.
    double z_body[3] = { -F_ned[0] / thrust_mag, -F_ned[1] / thrust_mag, -F_ned[2] / thrust_mag };

    // 3. Obter o vetor de rumo (heading/yaw) desejado em NED
    double yaw_ned = -target_yaw_ + M_PI_2;
    double x_yaw[3] = { std::cos(yaw_ned), std::sin(yaw_ned), 0.0 };

    // 4. Calcular o eixo Y do corpo (Y_body = Z_body x X_yaw)
    double y_body[3];
    y_body[0] = -z_body[2] * x_yaw[1];
    y_body[1] = z_body[2] * x_yaw[0];
    y_body[2] = z_body[0] * x_yaw[1] - z_body[1] * x_yaw[0];

    double y_norm = std::sqrt(y_body[0]*y_body[0] + y_body[1]*y_body[1] + y_body[2]*y_body[2]);
    if (y_norm < 1e-6) {
        y_body[0] = -std::sin(yaw_ned);
        y_body[1] = std::cos(yaw_ned);
        y_body[2] = 0.0;
    } else {
        y_body[0] /= y_norm;
        y_body[1] /= y_norm;
        y_body[2] /= y_norm;
    }

    // 5. Calcular o eixo X do corpo (X_body = Y_body x Z_body)
    double x_body[3];
    x_body[0] = y_body[1] * z_body[2] - y_body[2] * z_body[1];
    x_body[1] = y_body[2] * z_body[0] - y_body[0] * z_body[2];
    x_body[2] = y_body[0] * z_body[1] - y_body[1] * z_body[0];

    // 6. Formar a Matriz de Rotação Desejada R_d = [x_body, y_body, z_body]
    double R_d[3][3];
    R_d[0][0] = x_body[0]; R_d[0][1] = y_body[0]; R_d[0][2] = z_body[0];
    R_d[1][0] = x_body[1]; R_d[1][1] = y_body[1]; R_d[1][2] = z_body[1];
    R_d[2][0] = x_body[2]; R_d[2][1] = y_body[2]; R_d[2][2] = z_body[2];

    // 7. Converter R_d para quaternion q_d (ordem Hamiltoniana [w, x, y, z] para PX4)
    float q_d[4];
    rotationMatrixToQuaternion(R_d, q_d);

    output.q_d[0] = q_d[0];
    output.q_d[1] = q_d[1];
    output.q_d[2] = q_d[2];
    output.q_d[3] = q_d[3];

    // 8. Normalizar o thrust (força de empuxo)
    double hover_thrust = 2.06 * 9.81;
    double hover_throttle = 0.52;
    double thrust_normalized = (thrust_mag / hover_thrust) * hover_throttle;
    output.thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

    // 9. Extrair a trajetória prevista (predicted positions)
    double x_step[6];
    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        output.predicted_positions.push_back({x_step[0], x_step[1], x_step[2]});
    }

    output.current_reference = current_reference_;

    return output;
}

void UavMpcPipeline::quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) const {
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void UavMpcPipeline::rotationMatrixToQuaternion(double R[3][3], float q[4]) const {
    double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        double s = 2.0 * std::sqrt(tr + 1.0);
        q[0] = 0.25 * s;                // w
        q[1] = (R[2][1] - R[1][2]) / s; // x
        q[2] = (R[0][2] - R[2][0]) / s; // y
        q[3] = (R[1][0] - R[0][1]) / s; // z
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q[0] = (R[2][1] - R[1][2]) / s; // w
        q[1] = 0.25 * s;                // x
        q[2] = (R[0][1] + R[1][0]) / s; // y
        q[3] = (R[0][2] + R[2][0]) / s; // z
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q[0] = (R[0][2] - R[2][0]) / s; // w
        q[1] = (R[0][1] + R[1][0]) / s; // x
        q[2] = 0.25 * s;                // y
        q[3] = (R[1][2] + R[2][1]) / s; // z
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q[0] = (R[1][0] - R[0][1]) / s; // w
        q[1] = (R[0][2] + R[2][0]) / s; // x
        q[2] = (R[1][2] + R[2][1]) / s; // y
        q[3] = 0.25 * s;                // z
    }
}

} // namespace uav_mpc
