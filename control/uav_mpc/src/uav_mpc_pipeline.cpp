#include "uav_mpc/uav_mpc_pipeline.hpp"
#include "uav_mpc/uav_mpc_kinematics.hpp"
#include "uav_mpc/uav_mpc_tether.hpp"
#include "acados_solver_uav_tethered.h"
#include <iostream>
#include <cmath>

namespace uav_mpc {

UavMpcPipeline::UavMpcPipeline() 
    : qx_(0.0), qy_(0.0), qz_(0.0), qw_(1.0),
      target_yaw_(0.0), target_initialized_(false),
      anchor_x_(0.0), anchor_y_(0.0), anchor_z_(0.0),
      L_tether_(3.0), use_tether_(true), v_max_(2.0), u_max_(15.0),
      hover_throttle_(0.52), tilt_max_(0.2),
      trajectory_type_(TrajectoryType::HOLD), circle_start_time_(-1.0),
      open_loop_mode_enabled_(false), open_loop_active_(false),
      open_loop_step_(0), N_horizon_(0),
      last_phi_cmd_(0.0), last_theta_cmd_(0.0), first_run_(true) {
    current_state_.resize(6, 0.0);
    current_reference_.resize(3, 0.0);
    current_reference_velocity_.resize(3, 0.0);
    
    acados_ocp_capsule_ = uav_tethered_acados_create_capsule();
    int status = uav_tethered_acados_create((uav_tethered_solver_capsule*)acados_ocp_capsule_);
    if (status) {
        std::cerr << "UavMpcPipeline: Failed to create ACADOS solver!" << std::endl;
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
            circle_start_time_ = -1.0;  // Will be set on first computeControl call with valid time
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

void UavMpcPipeline::setHoverThrottle(double hover_throttle) {
    hover_throttle_ = hover_throttle;
}

void UavMpcPipeline::setTiltMax(double tilt_max) {
    tilt_max_ = tilt_max;
}

UavControlOutput UavMpcPipeline::computeControl(double current_time) {
    if (open_loop_mode_enabled_ && open_loop_active_) {
        UavControlOutput output;
        
        int idx = open_loop_step_;
        if (idx >= N_horizon_) {
            idx = N_horizon_ - 1; // Hold the last control input
        }
        
        const auto& step = open_loop_steps_[idx];
        
        output.q_d[0] = step.q_d[0];
        output.q_d[1] = step.q_d[1];
        output.q_d[2] = step.q_d[2];
        output.q_d[3] = step.q_d[3];
        
        output.thrust_normalized = step.thrust_normalized;
        output.u_opt[0] = step.u_opt[0];
        output.u_opt[1] = step.u_opt[1];
        output.u_opt[2] = step.u_opt[2];
        
        output.predicted_positions = open_loop_predicted_positions_;
        output.current_reference = {step.reference[0], step.reference[1], step.reference[2]};
        output.current_reference_velocity = {step.reference_velocity[0], step.reference_velocity[1], step.reference_velocity[2]};
        
        output.reference_path.clear();
        if (trajectory_type_ == TrajectoryType::CIRCLE) {
            double elapsed_vis = (circle_start_time_ >= 0.0) ? (current_time - circle_start_time_) : 0.0;
            for (int i = 0; i <= N_horizon_; i++) {
                double t_stage = elapsed_vis + i * Ts_;
                TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
                output.reference_path.push_back({pt.px, pt.py, pt.pz});
            }
        } else {
            output.reference_path.push_back({current_reference_[0], current_reference_[1], current_reference_[2]});
        }
        
        output.mpc_tether_force_mag = step.mpc_tether_force_mag;
        
        if (open_loop_step_ < N_horizon_) {
            open_loop_step_++;
        }
        
        return output;
    }

    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    
    // Get current drone attitude angles in Euler representation
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    kinematics::quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, yaw);

    // Set current initial state (x0) in ACADOS: [x, y, z, vx, vy, vz, phi, theta, phi_cmd, theta_cmd]
    double x0[10];
    x0[0] = current_state_[0];
    x0[1] = current_state_[1];
    x0[2] = current_state_[2];

    // Rotate velocities from body frame to world frame (ENU)
    double v_body[3] = {current_state_[3], current_state_[4], current_state_[5]};
    double v_world[3] = {0.0, 0.0, 0.0};
    kinematics::rotateVectorByQuaternion(qx_, qy_, qz_, qw_, v_body, v_world);

    x0[3] = v_world[0];
    x0[4] = v_world[1];
    x0[5] = v_world[2];
    x0[6] = roll;
    x0[7] = pitch;

    if (first_run_) {
        last_phi_cmd_ = roll;
        last_theta_cmd_ = pitch;
        first_run_ = false;
    }
    x0[8] = last_phi_cmd_;
    x0[9] = last_theta_cmd_;

    // Save initial yaw for control reference purposes if needed and warm start solver guesses once
    if (!target_initialized_) {
        target_yaw_ = yaw;
        
        double x_guess[10] = {x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7], x0[8], x0[9]};
        double u_guess[3] = {0.0, 0.0, 9.81};
        for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x_guess);
        }
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_guess);
        }
        target_initialized_ = true;
    }

    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // Set state velocity and tilt bounds (lbx, ubx for stages 1...N)
    // idxbx = [3, 4, 5, 6, 7, 8, 9] corresponding to [vx, vy, vz, phi, theta, phi_cmd, theta_cmd]
    double lbx[7] = { -v_max_, -v_max_, -v_max_, -tilt_max_, -tilt_max_, -tilt_max_, -tilt_max_ };
    double ubx[7] = {  v_max_,  v_max_,  v_max_,  tilt_max_,  tilt_max_,  tilt_max_,  tilt_max_ };
    for (int i = 1; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubx", ubx);
    }

    // Set control inputs bounds (lbu, ubu for stages 0...N-1)
    // idxbu = [0, 1, 2], corresponding to [phi_dot_cmd, theta_dot_cmd, a_T]
    // Slew rate limits: angular rate speed bounded at 2.0 rad/s (~115 deg/s)
    double lbu[3] = { -0.8, -0.8, 0.1 * 9.81 };
    double ubu[3] = {  0.8,  0.8, u_max_ };
    for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubu", ubu);
    }

    // Inject reference trajectory into the prediction horizon
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        // Initialize circle start time on first call
        if (circle_start_time_ < 0.0) {
            circle_start_time_ = current_time;
        }
        double elapsed = current_time - circle_start_time_;

        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            double t_stage = elapsed + i * Ts_;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            // yref = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, phi_cmd, theta_cmd, phi_dot_cmd, theta_dot_cmd, a_T]
            double yref[13] = {pt.px, pt.py, pt.pz, pt.vx, pt.vy, pt.vz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        double t_terminal = elapsed + capsule->nlp_solver_plan->N * Ts_;
        TrajectoryPoint pt_e = trajectory_gen_.getPoint(t_terminal);
        // yref_e = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, phi_cmd, theta_cmd]
        double yref_e[10] = {pt_e.px, pt_e.py, pt_e.pz, pt_e.vx, pt_e.vy, pt_e.vz, 0.0, 0.0, 0.0, 0.0};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);

        // Update current reference for RViz visualization (at start of prediction horizon)
        TrajectoryPoint pt_start = trajectory_gen_.getPoint(elapsed);
        current_reference_ = {pt_start.px, pt_start.py, pt_start.pz};
        current_reference_velocity_ = {pt_start.vx, pt_start.vy, pt_start.vz};
    } else {
        // HOLD mode
        double yref[13] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
        double yref_e[10] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);
        current_reference_velocity_ = {0.0, 0.0, 0.0};
    }

    // Dynamic tension calculation
    double T0_val = 0.0;
    if (use_tether_) {
        T0_val = tether::calculateWinchTension(L_tether_);
    }

    // Set parameters in ACADOS: [psi]
    double p_params[1] = {
        yaw
    };

    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        uav_tethered_acados_update_params(capsule, i, p_params, 1);
    }

    // Se já foi inicializado, fazemos shifting da solução anterior para o warm start do novo ciclo
    if (target_initialized_) {
        int N = capsule->nlp_solver_plan->N;

        // 1. Shift dos controlos (u_i = u_{i+1})
        double u_temp[3];
        for (int i = 0; i < N - 1; i++) {
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "u", u_temp);
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_temp);
        }
        // Duplicar o NOVO último controlo (copiar de N-2 para N-1)
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, N - 2, "u", u_temp);
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, N - 1, "u", u_temp);

        // 2. Shift dos estados (x_i = x_{i+1})
        double x_temp[10];
        for (int i = 0; i < N; i++) {
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "x", x_temp);
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x_temp);
        }
        // Duplicar o NOVO último estado (copiar de N-1 para N)
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, N - 1, "x", x_temp);
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, N, "x", x_temp);
    }

    // Solve OCP using wrapper function
    int status = uav_tethered_acados_solve(capsule);
    
    // Extract optimal control inputs (u_opt)
    double u_opt[3] = {0.0, 0.0, 0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", &u_opt);
    } else {
        std::cerr << "UavMpcPipeline: ACADOS solver failed with status: " << status << std::endl;
    }

    // Extract predicted state at step 1 to read target attitude commands [phi_cmd, theta_cmd] (indices 8, 9)
    double x_step1[10] = {0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 1, "x", &x_step1);
    }
    double phi_cmd = x_step1[8];
    double theta_cmd = x_step1[9];

    // Feedback update for the next iteration
    if (status == 0) {
        last_phi_cmd_ = phi_cmd;
        last_theta_cmd_ = theta_cmd;
    }

    UavControlOutput output;

    // Populate optimal control command outputs
    // (Map target angles in u_opt for logger and telemetry compatibility)
    output.u_opt[0] = phi_cmd;
    output.u_opt[1] = theta_cmd;
    output.u_opt[2] = u_opt[2]; // thrust acceleration command

    // Calculate desired attitude outputs using target angles from step 1
    kinematics::computeDesiredQuaternion(phi_cmd, theta_cmd, yaw, output.q_d);

    // Normalize thrust (hover throttle calculation)
    double thrust_normalized = (u_opt[2] / 9.81) * hover_throttle_;
    output.thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

    // Extract predicted trajectory positions
    double x_step[10];
    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        output.predicted_positions.push_back({x_step[0], x_step[1], x_step[2]});
    }

    output.current_reference = current_reference_;
    output.current_reference_velocity = current_reference_velocity_;

    // Calculate estimated tether force magnitude
    output.mpc_tether_force_mag = tether::estimateTetherForce(
        current_state_[0], current_state_[1], current_state_[2],
        anchor_x_, anchor_y_, anchor_z_,
        T0_val
    );

    // Populate reference path for visualization (N points along the prediction horizon)
    output.reference_path.clear();
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        double elapsed_vis = (circle_start_time_ >= 0.0) ? (current_time - circle_start_time_) : 0.0;
        for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
            double t_stage = elapsed_vis + i * Ts_;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            output.reference_path.push_back({pt.px, pt.py, pt.pz});
        }
    } else {
        output.reference_path.push_back({current_reference_[0], current_reference_[1], current_reference_[2]});
    }

    return output;
}

void UavMpcPipeline::updateAnchorPosition(double x, double y, double z) {
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_z_ = z;
}

void UavMpcPipeline::updateTetherLength(double length) {
    L_tether_ = length;
}

void UavMpcPipeline::setOpenLoopMode(bool enabled) {
    open_loop_mode_enabled_ = enabled;
}

void UavMpcPipeline::captureOpenLoopHorizon(double current_time) {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    int N = capsule->nlp_solver_plan->N;
    N_horizon_ = N;

    // Get current drone attitude angles in Euler representation
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    kinematics::quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, yaw);

    // Set current initial state (x0) in ACADOS: [x, y, z, vx, vy, vz, phi, theta, phi_cmd, theta_cmd]
    double x0[10];
    x0[0] = current_state_[0];
    x0[1] = current_state_[1];
    x0[2] = current_state_[2];

    // Rotate velocities from body frame to world frame (ENU)
    double v_body[3] = {current_state_[3], current_state_[4], current_state_[5]};
    double v_world[3] = {0.0, 0.0, 0.0};
    kinematics::rotateVectorByQuaternion(qx_, qy_, qz_, qw_, v_body, v_world);

    x0[3] = v_world[0];
    x0[4] = v_world[1];
    x0[5] = v_world[2];
    x0[6] = roll;
    x0[7] = pitch;
    x0[8] = last_phi_cmd_;
    x0[9] = last_theta_cmd_;

    // Warm start solver guesses once if needed
    if (!target_initialized_) {
        target_yaw_ = yaw;
        double x_guess[10] = {x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7], x0[8], x0[9]};
        double u_guess[3] = {0.0, 0.0, 9.81};
        for (int i = 0; i <= N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x_guess);
        }
        for (int i = 0; i < N; i++) {
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_guess);
        }
        target_initialized_ = true;
    }

    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // Set state velocity and tilt bounds
    double lbx[7] = { -v_max_, -v_max_, -v_max_, -tilt_max_, -tilt_max_, -tilt_max_, -tilt_max_ };
    double ubx[7] = {  v_max_,  v_max_,  v_max_,  tilt_max_,  tilt_max_,  tilt_max_,  tilt_max_ };
    for (int i = 1; i <= N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubx", ubx);
    }

    // Set control inputs bounds
    double lbu[3] = { -0.8, -0.8, 0.1 * 9.81 };
    double ubu[3] = {  0.8,  0.8, u_max_ };
    for (int i = 0; i < N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubu", ubu);
    }

    // Inject reference trajectory into the prediction horizon
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        if (circle_start_time_ < 0.0) {
            circle_start_time_ = current_time;
        }
        double elapsed = current_time - circle_start_time_;

        for (int i = 0; i < N; i++) {
            double t_stage = elapsed + i * Ts_;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            double yref[13] = {pt.px, pt.py, pt.pz, pt.vx, pt.vy, pt.vz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        double t_terminal = elapsed + N * Ts_;
        TrajectoryPoint pt_e = trajectory_gen_.getPoint(t_terminal);
        double yref_e[10] = {pt_e.px, pt_e.py, pt_e.pz, pt_e.vx, pt_e.vy, pt_e.vz, 0.0, 0.0, 0.0, 0.0};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);
        
        TrajectoryPoint pt_start = trajectory_gen_.getPoint(elapsed);
        current_reference_ = {pt_start.px, pt_start.py, pt_start.pz};
        current_reference_velocity_ = {pt_start.vx, pt_start.vy, pt_start.vz};
    } else {
        double yref[13] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
        double yref_e[10] = {current_reference_[0], current_reference_[1], current_reference_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < N; i++) {
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, N, "yref", yref_e);
        current_reference_velocity_ = {0.0, 0.0, 0.0};
    }

    double T0_val = use_tether_ ? tether::calculateWinchTension(L_tether_) : 0.0;
    double p_params[1] = { yaw };
    for (int i = 0; i <= N; i++) {
        uav_tethered_acados_update_params(capsule, i, p_params, 1);
    }

    // Solve OCP
    int status = uav_tethered_acados_solve(capsule);
    if (status != 0) {
        std::cerr << "UavMpcPipeline: ACADOS solver failed during open-loop capture with status: " << status << std::endl;
    }

    open_loop_steps_.clear();
    open_loop_predicted_positions_.clear();

    // Retrieve predicted states
    for (int i = 0; i <= N; i++) {
        double x_step[10];
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        open_loop_predicted_positions_.push_back({x_step[0], x_step[1], x_step[2]});
    }

    // Retrieve control inputs and compute required step properties
    for (int i = 0; i < N; i++) {
        OpenLoopControlStep step_data;
        double u_val[3] = {0.0, 0.0, 0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "u", u_val);

        double x_next[10] = {0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "x", x_next);
        double phi_cmd = x_next[8];
        double theta_cmd = x_next[9];

        step_data.u_opt[0] = phi_cmd;
        step_data.u_opt[1] = theta_cmd;
        step_data.u_opt[2] = u_val[2];

        // Compute desired quaternion (using initial yaw)
        kinematics::computeDesiredQuaternion(phi_cmd, theta_cmd, yaw, step_data.q_d);

        // Compute normalized thrust
        double thrust_normalized = (u_val[2] / 9.81) * hover_throttle_;
        step_data.thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

        // Reference trajectory points
        if (trajectory_type_ == TrajectoryType::CIRCLE) {
            double elapsed = current_time - circle_start_time_;
            double t_stage = elapsed + i * Ts_;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            step_data.reference[0] = pt.px;
            step_data.reference[1] = pt.py;
            step_data.reference[2] = pt.pz;
            step_data.reference_velocity[0] = pt.vx;
            step_data.reference_velocity[1] = pt.vy;
            step_data.reference_velocity[2] = pt.vz;
        } else {
            step_data.reference[0] = current_reference_[0];
            step_data.reference[1] = current_reference_[1];
            step_data.reference[2] = current_reference_[2];
            step_data.reference_velocity[0] = 0.0;
            step_data.reference_velocity[1] = 0.0;
            step_data.reference_velocity[2] = 0.0;
        }

        // Calculate estimated tether force magnitude
        step_data.mpc_tether_force_mag = tether::estimateTetherForce(
            x_next[0], x_next[1], x_next[2],
            anchor_x_, anchor_y_, anchor_z_,
            T0_val
        );

        open_loop_steps_.push_back(step_data);
    }

    open_loop_step_ = 0;
    open_loop_active_ = true;
}

void UavMpcPipeline::resetOpenLoop() {
    open_loop_active_ = false;
    open_loop_step_ = 0;
}

bool UavMpcPipeline::isOpenLoopActive() const {
    return open_loop_active_;
}

const std::vector<std::vector<double>>& UavMpcPipeline::getCapturedPredictedPositions() const {
    return open_loop_predicted_positions_;
}

} // namespace uav_mpc
