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
      target_yaw_(0.0), target_initialized_(false),
      anchor_x_(0.0), anchor_y_(0.0), anchor_z_(0.0),
      L_tether_(3.0), use_tether_(true), v_max_(10.0), u_max_(19.62), tau_(0.15),
      trajectory_type_(TrajectoryType::HOLD), time_(0.0) {
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
    }
}

void UavMpcPipeline::setTrajectoryType(TrajectoryType type) {
    if (trajectory_type_ != type) {
        trajectory_type_ = type;
        if (type == TrajectoryType::CIRCLE) {
            time_ = 0.0;
        }
    }
}

void UavMpcPipeline::configureCircle(double radius, double omega, double height, double center_x, double center_y) {
    trajectory_gen_.configureCircle(radius, omega, height, center_x, center_y);
}

void UavMpcPipeline::setUseTether(bool use_tether) {
    use_tether_ = use_tether;
}

void UavMpcPipeline::setVelocityLimit(double v_max) {
    v_max_ = v_max;
}

void UavMpcPipeline::setInputLimit(double u_max) {
    u_max_ = u_max;
}

void UavMpcPipeline::setAttitudeTimeConstant(double tau) {
    tau_ = tau;
}

UavControlOutput UavMpcPipeline::computeControl() {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    
    // Obter atitude e yaw atuais do drone
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, yaw);

    // Guardar yaw inicial para fins de visualização ou controle se necessário
    if (!target_initialized_) {
        target_yaw_ = yaw;
        target_initialized_ = true;
    }

    // Injetar estado atual (x0) no ACADOS: [x, y, z, vx, vy, vz, phi, theta]
    double x0[8];
    x0[0] = current_state_[0];
    x0[1] = current_state_[1];
    x0[2] = current_state_[2];

    // Rotate velocities from body frame to world frame (ENU)
    double v_body[3] = {current_state_[3], current_state_[4], current_state_[5]};
    double v_world[3] = {0.0, 0.0, 0.0};
    rotateVectorByQuaternion(qx_, qy_, qz_, qw_, v_body, v_world);

    x0[3] = v_world[0];
    x0[4] = v_world[1];
    x0[5] = v_world[2];

    x0[6] = roll;
    x0[7] = pitch;
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // Atualizar limites de velocidade e atitude real (lbx, ubx para etapas 1...N)
    // idxbx = [3, 4, 5, 6, 7], que corresponde a [vx, vy, vz, phi, theta]
    double lbx[5] = { -v_max_, -v_max_, -v_max_, -0.4, -0.4 };
    double ubx[5] = {  v_max_,  v_max_,  v_max_,  0.4,  0.4 };
    for (int i = 1; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubx", ubx);
    }

    // Atualizar limites de controlo (lbu, ubu para etapas 0...N-1)
    // idxbu = [0, 1, 2], que corresponde a [phi_cmd, theta_cmd, a_T]
    double lbu[3] = { -0.4, -0.4, 0.1 * 9.81 };
    double ubu[3] = {  0.4,  0.4, u_max_ };
    for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubu", ubu);
    }

    // Injetar referências no horizonte de predição
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        time_ += 0.05; // Incrementa 50ms (Ts)
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            double t_stage = time_ + i * 0.05;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            // yref = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, phi_cmd, theta_cmd, a_T]
            double yref[11] = {pt.px, pt.py, pt.pz, pt.vx, pt.vy, pt.vz, 0.0, 0.0, 0.0, 0.0, 9.81};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        double t_terminal = time_ + capsule->nlp_solver_plan->N * 0.05;
        TrajectoryPoint pt_e = trajectory_gen_.getPoint(t_terminal);
        // yref_e = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta]
        double yref_e[8] = {pt_e.px, pt_e.py, pt_e.pz, pt_e.vx, pt_e.vy, pt_e.vz, 0.0, 0.0};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);

        // Atualizar current_reference_ para visualização no RViz
        current_reference_ = {pt_e.px, pt_e.py, pt_e.pz};
    } else {
        // HOLD mode
        double yref[11] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
        double yref_e[8] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);
    }

    // Dynamic tension including cable weight: T_0 = T_winch + rho * L * g (or 0 if disabled)
    double T0_val = 0.0;
    if (use_tether_) {
        T0_val = 0.1 + 0.020 * L_tether_ * 9.81;
    }

    // Parâmetros a passar ao ACADOS: [p_anchor_x, p_anchor_y, p_anchor_z, T0, wc, eps, psi, tau]
    double p_params[8] = {
        anchor_x_,
        anchor_y_,
        anchor_z_,
        T0_val,  // T0: updated dynamically
        0.0,     // wc: 0.0 stiffness
        0.02,    // eps: small offset
        yaw,     // psi: current yaw of the drone
        tau_     // tau: attitude response time constant
    };

    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        uav_tethered_acados_update_params(capsule, i, p_params, 8);
    }

    // Configurar o limite superior da restrição não linear do cabo (uh) dinamicamente
    double uh_val = 900.0; // Default L_max^2 (30^2)
    if (!use_tether_) {
        uh_val = 1e8; // Limite muito grande para simular drone livre
    } else {
        uh_val = (L_tether_ + 1.0) * (L_tether_ + 1.0); // Comprimento atual + margem
    }
    double uh_array[1] = { uh_val };
    for (int i = 1; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "uh", uh_array);
    }

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
    
    // Construir matriz de rotação desejada em ENU: R_enu = R_z(yaw) * R_y(theta_cmd) * R_x(phi_cmd)
    double c_psi = std::cos(yaw);
    double s_psi = std::sin(yaw);
    double c_theta = std::cos(u_opt[1]);
    double s_theta = std::sin(u_opt[1]);
    double c_phi = std::cos(u_opt[0]);
    double s_phi = std::sin(u_opt[0]);

    double R_enu[3][3];
    R_enu[0][0] = c_psi * c_theta;
    R_enu[0][1] = c_psi * s_theta * s_phi - s_psi * c_phi;
    R_enu[0][2] = c_psi * s_theta * c_phi + s_psi * s_phi;
    R_enu[1][0] = s_psi * c_theta;
    R_enu[1][1] = s_psi * s_theta * s_phi + c_psi * c_phi;
    R_enu[1][2] = s_psi * s_theta * c_phi - c_psi * s_phi;
    R_enu[2][0] = -s_theta;
    R_enu[2][1] = c_theta * s_phi;
    R_enu[2][2] = c_theta * c_phi;

    // Converter R_enu para R_d (NED/FRD) usando R_d = M * R_enu * M
    double R_d[3][3];
    R_d[0][0] = R_enu[1][1];
    R_d[0][1] = R_enu[1][0];
    R_d[0][2] = -R_enu[1][2];

    R_d[1][0] = R_enu[0][1];
    R_d[1][1] = R_enu[0][0];
    R_d[1][2] = -R_enu[0][2];

    R_d[2][0] = -R_enu[2][1];
    R_d[2][1] = -R_enu[2][0];
    R_d[2][2] = R_enu[2][2];

    // Converter R_d para quaternion q_d (ordem Hamiltoniana [w, x, y, z] para PX4)
    float q_d[4];
    rotationMatrixToQuaternion(R_d, q_d);

    output.q_d[0] = q_d[0];
    output.q_d[1] = q_d[1];
    output.q_d[2] = q_d[2];
    output.q_d[3] = q_d[3];

    // Normalizar o thrust (força de empuxo)
    double hover_throttle = 0.52;
    double thrust_normalized = (u_opt[2] / 9.81) * hover_throttle;
    output.thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

    // Extrair a trajetória prevista (predicted positions)
    double x_step[8];
    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        output.predicted_positions.push_back({x_step[0], x_step[1], x_step[2]});
    }

    output.current_reference = current_reference_;

    // Calcular a magnitude da força que o MPC assume que o tether está a fazer
    double dx = anchor_x_ - current_state_[0];
    double dy = anchor_y_ - current_state_[1];
    double dz = anchor_z_ - current_state_[2];
    double s_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double eps = 0.02; // do generate_acados_uav.py
    double s_norm_eps = std::sqrt(s_dist*s_dist + eps*eps);
    output.mpc_tether_force_mag = T0_val * (s_dist / s_norm_eps);

    // Popular o caminho de referência para visualização no RViz
    output.reference_path.clear();
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        std::vector<TrajectoryPoint> ref_path = trajectory_gen_.getReferencePath();
        for (const auto& pt : ref_path) {
            output.reference_path.push_back({pt.px, pt.py, pt.pz});
        }
    } else {
        // HOLD mode: o caminho é apenas o ponto estático atual
        output.reference_path.push_back({current_reference_[0], current_reference_[1], current_reference_[2]});
    }

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

void UavMpcPipeline::rotateVectorByQuaternion(double qx, double qy, double qz, double qw, const double v_in[3], double v_out[3]) const {
    // v_out = v_in + 2 * q_xyz x (q_xyz x v_in + w * v_in)
    double tx = 2.0 * (qy * v_in[2] - qz * v_in[1]);
    double ty = 2.0 * (qz * v_in[0] - qx * v_in[2]);
    double tz = 2.0 * (qx * v_in[1] - qy * v_in[0]);

    v_out[0] = v_in[0] + qw * tx + qy * tz - qz * ty;
    v_out[1] = v_in[1] + qw * ty + qz * tx - qx * tz;
    v_out[2] = v_in[2] + qw * tz + qx * ty - qy * tx;
}

void UavMpcPipeline::updateAnchorPosition(double x, double y, double z) {
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_z_ = z;
}

void UavMpcPipeline::updateTetherLength(double length) {
    L_tether_ = length;
}

} // namespace uav_mpc
