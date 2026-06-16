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
      trajectory_type_(TrajectoryType::HOLD), time_(0.0) {
    current_state_.resize(6, 0.0);
    current_reference_.resize(3, 0.0);
    
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

void UavMpcPipeline::setHoverThrottle(double hover_throttle) {
    hover_throttle_ = hover_throttle;
}

void UavMpcPipeline::setTiltMax(double tilt_max) {
    tilt_max_ = tilt_max;
}

UavControlOutput UavMpcPipeline::computeControl() {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    
    // Get current drone attitude angles in Euler representation
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    kinematics::quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, yaw);

    // Save initial yaw for control reference purposes if needed
    if (!target_initialized_) {
        target_yaw_ = yaw;
        target_initialized_ = true;
    }

    // Set current initial state (x0) in ACADOS: [x, y, z, vx, vy, vz, phi, theta]
    double x0[8];
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

    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", x0);

    // Warm-start solver state and control guesses to avoid impossible transitions from zero initialization
    double x_guess[8] = {x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7]};
    double u_guess[3] = {0.0, 0.0, 9.81};
    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x_guess);
    }
    for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_guess);
    }

    // Set state velocity and tilt bounds (lbx, ubx for stages 1...N)
    // idxbx = [3, 4, 5, 6, 7] corresponding to [vx, vy, vz, phi, theta]
    double lbx[5] = { -v_max_, -v_max_, -v_max_, -tilt_max_, -tilt_max_ };
    double ubx[5] = {  v_max_,  v_max_,  v_max_,  tilt_max_,  tilt_max_ };
    for (int i = 1; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubx", ubx);
    }

    // Set control inputs bounds (lbu, ubu for stages 0...N-1)
    // idxbu = [0, 1, 2], corresponding to [phi_dot_cmd, theta_dot_cmd, a_T]
    // Slew rate limits: angular rate speed bounded at 2.0 rad/s (~115 deg/s)
    double lbu[3] = { -2.0, -2.0, 0.1 * 9.81 };
    double ubu[3] = {  2.0,  2.0, u_max_ };
    for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, i, "ubu", ubu);
    }

    // Inject reference trajectory into the prediction horizon
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        time_ += 0.05; // Increment by 50ms (Ts)
        for (int i = 0; i < capsule->nlp_solver_plan->N; i++) {
            double t_stage = time_ + i * 0.05;
            TrajectoryPoint pt = trajectory_gen_.getPoint(t_stage);
            // yref = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, phi_dot_cmd, theta_dot_cmd, a_T]
            double yref[11] = {pt.px, pt.py, pt.pz, pt.vx, pt.vy, pt.vz, 0.0, 0.0, 0.0, 0.0, 9.81};
            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, i, "yref", yref);
        }
        double t_terminal = time_ + capsule->nlp_solver_plan->N * 0.05;
        TrajectoryPoint pt_e = trajectory_gen_.getPoint(t_terminal);
        // yref_e = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta]
        double yref_e[8] = {pt_e.px, pt_e.py, pt_e.pz, pt_e.vx, pt_e.vy, pt_e.vz, 0.0, 0.0};
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_solver_plan->N, "yref", yref_e);

        // Update current reference for RViz visualization
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

    // Solve OCP using wrapper function
    int status = uav_tethered_acados_solve(capsule);
    
    // Extract optimal control inputs (u_opt)
    double u_opt[3] = {0.0, 0.0, 0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", &u_opt);
    } else {
        std::cerr << "UavMpcPipeline: ACADOS solver failed with status: " << status << std::endl;
    }

    // Extract predicted state at step 1 to read target attitude angles [phi_cmd, theta_cmd]
    double x_step1[8] = {0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 1, "x", &x_step1);
    }
    double phi_cmd = x_step1[6];
    double theta_cmd = x_step1[7];

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
    double x_step[8];
    for (int i = 0; i <= capsule->nlp_solver_plan->N; i++) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        output.predicted_positions.push_back({x_step[0], x_step[1], x_step[2]});
    }

    output.current_reference = current_reference_;

    // Calculate estimated tether force magnitude
    output.mpc_tether_force_mag = tether::estimateTetherForce(
        current_state_[0], current_state_[1], current_state_[2],
        anchor_x_, anchor_y_, anchor_z_,
        T0_val
    );

    // Populate reference path for visualization
    output.reference_path.clear();
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
        std::vector<TrajectoryPoint> ref_path = trajectory_gen_.getReferencePath();
        for (const auto& pt : ref_path) {
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

} // namespace uav_mpc
