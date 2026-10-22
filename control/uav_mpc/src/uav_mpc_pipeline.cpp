#include "uav_mpc/uav_mpc_pipeline.hpp"
#include "uav_mpc/uav_mpc_kinematics.hpp"
#include "uav_mpc/uav_mpc_tether_geometry.hpp"
#include "acados_solver_uav_tethered.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <utility>

namespace uav_mpc {

namespace {
constexpr int NX = 13;      // [x, y, z, vx, vy, vz, phi, theta, psi, phi_cmd, theta_cmd, psi_cmd, T]
constexpr int PREDICTED_ATTITUDE_NODE = 10;  // node of the horizon whose attitude is published (10 T_s = 0.2 s)
constexpr int NU = 4;       // [phi_dot_cmd, theta_dot_cmd, psi_dot_cmd, T_dot]
constexpr int NY = 15;      // [p, v, phi, theta, yaw residual (2), T - m g, u (4)]
constexpr int NY_E = 11;
constexpr int NP = 13;      // [anchor (3), anchor velocity (3), yaw reference point (2), tether switch, disturbance (3), tether model switch]
constexpr double NO_LIMIT = 1.0e6;

double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

UavMpcPipeline::UavMpcPipeline()
    : qx_(0.0), qy_(0.0), qz_(0.0), qw_(1.0),
      target_yaw_(0.0), target_initialized_(false),
      anchor_x_(0.0), anchor_y_(0.0), anchor_z_(0.0),
      has_boat_horizon_(false),
      L_tether_(3.0), use_tether_(true),
      trajectory_type_(TrajectoryType::HOLD),
      last_phi_cmd_(0.0), last_theta_cmd_(0.0), last_psi_cmd_(0.0), last_thrust_cmd_(0.0), yaw_cont_(0.0), first_run_(true) {
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

void UavMpcPipeline::setExternalReferencePath(const std::vector<TrajectoryPoint>& path) {
    external_reference_path_ = path;
}

void UavMpcPipeline::setTrajectoryType(TrajectoryType type) {
    trajectory_type_ = type;
}

void UavMpcPipeline::configureDisturbanceObserver(bool enabled, double gain, double max_force) {
    observer_enabled_ = enabled;
    observer_.configure(gain, max_force, params_.mass, Ts_);
    if (!enabled) {
        resetDisturbance();
    }
}

void UavMpcPipeline::setYawHeading(bool enabled, double heading) {
    yaw_heading_enabled_ = enabled;
    yaw_heading_ = heading;
}

void UavMpcPipeline::setUseTether(bool use_tether) {
    use_tether_ = use_tether;
}

void UavMpcPipeline::setTetherModel(bool tether_model) {
    tether_model_ = tether_model;
}

void UavMpcPipeline::setParameters(const UavMpcParams& params) {
    params_ = params;
}

void UavMpcPipeline::updateAnchorPosition(double x, double y, double z) {
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_z_ = z;
}

void UavMpcPipeline::updateBoatHorizon(const std::vector<std::vector<double>>& boat_horizon) {
    boat_horizon_ = boat_horizon;
    has_boat_horizon_ = !boat_horizon.empty();
}

void UavMpcPipeline::updateTetherLength(double length) {
    L_tether_ = length;
}

void UavMpcPipeline::buildInitialState(double x0[13]) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    kinematics::quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, yaw);

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

    // Yaw is tracked continuously (unwrapped) so that the state never jumps across +-pi
    if (first_run_) {
        yaw_cont_ = yaw;
        last_phi_cmd_ = roll;
        last_theta_cmd_ = pitch;
        last_psi_cmd_ = yaw;
        last_thrust_cmd_ = params_.mass * params_.gravity;
        first_run_ = false;
    } else {
        yaw_cont_ += wrapAngle(yaw - yaw_cont_);
    }

    // The tilt state is the thrust axis expressed in the frame rotated by the commanded yaw (the frame of the attitude
    // setpoint): the measured axis is rotated by the difference between the measured and the commanded yaw
    const double n_x = std::sin(pitch) * std::cos(roll);
    const double n_y = -std::sin(roll);
    const double n_z = std::cos(pitch) * std::cos(roll);
    const double dpsi = yaw - last_psi_cmd_;
    const double loc_x = std::cos(dpsi) * n_x - std::sin(dpsi) * n_y;
    const double loc_y = std::sin(dpsi) * n_x + std::cos(dpsi) * n_y;
    x0[6] = std::asin(-loc_y);
    x0[7] = std::atan2(loc_x, n_z);
    x0[8] = yaw_cont_;
    x0[9] = last_phi_cmd_;
    x0[10] = last_theta_cmd_;
    x0[11] = last_psi_cmd_;
    x0[12] = last_thrust_cmd_;
}

void UavMpcPipeline::warmStartOnce(const double x0[13]) {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    const int N = capsule->nlp_solver_plan->N;
    if (target_initialized_) return;

    target_yaw_ = x0[8];
    double u_guess[NU] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i <= N; i++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", const_cast<double*>(x0));
    }
    for (int i = 0; i < N; i++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_guess);
    }
    target_initialized_ = true;
}

UavControlOutput UavMpcPipeline::computeControl(double current_time) {
    (void)current_time;
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    const int N = capsule->nlp_solver_plan->N;

    double x0[NX];
    buildInitialState(x0);
    const bool warm = target_initialized_;

    // Disturbance observer: velocity predicted by the previous solution for this instant against the measured one
    if (observer_enabled_ && has_predicted_velocity_) {
        observer_.update({x0[3], x0[4], x0[5]}, predicted_velocity_, disturbance_frozen_);
    }
    warmStartOnce(x0);

    setupOcpSolver(x0);

    // Shift the previous solution by one node to warm start the new cycle
    if (warm) {
        double u_temp[NU];
        for (int i = 0; i < N - 1; i++) {
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "u", u_temp);
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "u", u_temp);
        }
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, N - 2, "u", u_temp);
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, N - 1, "u", u_temp);

        double x_temp[NX];
        for (int i = 0; i < N; i++) {
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "x", x_temp);
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, i, "x", x_temp);
        }
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, N - 1, "x", x_temp);
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, N, "x", x_temp);
    }

    int status = uav_tethered_acados_solve(capsule);

    double x_step1[NX] = {0.0};
    if (status == 0) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 1, "x", x_step1);
        // The commanded attitude at node 1 is the integral of the first optimal rates over one sampling interval
        last_phi_cmd_ = x_step1[9];
        last_theta_cmd_ = x_step1[10];
        last_psi_cmd_ = x_step1[11];
        last_thrust_cmd_ = x_step1[12];
        predicted_velocity_ = {x_step1[3], x_step1[4], x_step1[5]};
        has_predicted_velocity_ = true;
    } else {
        std::cerr << "UavMpcPipeline: ACADOS solver failed with status: " << status << std::endl;
        has_predicted_velocity_ = false;
    }

    UavControlOutput output;
    output.u_opt[0] = last_phi_cmd_;
    output.u_opt[1] = last_theta_cmd_;
    output.u_opt[2] = last_thrust_cmd_;   // thrust (N)
    output.psi_cmd = last_psi_cmd_;

    kinematics::computeDesiredQuaternion(last_phi_cmd_, last_theta_cmd_, last_psi_cmd_, output.q_d);

    output.thrust_normalized = kinematics::thrustToThrottle(last_thrust_cmd_, params_.mass, params_.gravity, params_.hover_throttle,
                                                            params_.rotor_speed_min, params_.rotor_speed_max);

    double x_step[NX];
    for (int i = 0; i <= N; i++) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        output.predicted_positions.push_back({x_step[0], x_step[1], x_step[2]});
        if (i == PREDICTED_ATTITUDE_NODE) {
            // Predicted roll and pitch re-expressed from the frame of the commanded yaw to that of the predicted yaw,
            // so that they compare directly with the measured Euler angles
            const double n_x = std::sin(x_step[7]) * std::cos(x_step[6]);
            const double n_y = -std::sin(x_step[6]);
            const double n_z = std::cos(x_step[7]) * std::cos(x_step[6]);
            const double a = x_step[11] - x_step[8];
            const double l_x = std::cos(a) * n_x - std::sin(a) * n_y;
            const double l_y = std::sin(a) * n_x + std::cos(a) * n_y;
            output.predicted_attitude[0] = std::asin(-l_y);
            output.predicted_attitude[1] = std::atan2(l_x, n_z);
            output.predicted_attitude[2] = x_step[8];
        }
    }

    output.current_reference = current_reference_;
    output.current_reference_velocity = current_reference_velocity_;

    output.reference_path.clear();
    if (trajectory_type_ == TrajectoryType::EXTERNAL) {
        for (const auto& pt : external_reference_path_) {
            output.reference_path.push_back({pt.px, pt.py, pt.pz});
        }
    } else {
        output.reference_path.push_back({current_reference_[0], current_reference_[1], current_reference_[2]});
    }

    return output;
}

std::vector<UavControlOutput> UavMpcPipeline::getOpenLoopHorizon(double current_time) {
    (void)current_time;
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    const int N = capsule->nlp_solver_plan->N;

    double x0[NX];
    buildInitialState(x0);
    warmStartOnce(x0);
    setupOcpSolver(x0);

    int status = uav_tethered_acados_solve(capsule);
    if (status != 0) {
        std::cerr << "UavMpcPipeline: ACADOS solver failed during open-loop capture with status: " << status << std::endl;
    }

    std::vector<std::vector<double>> pred_positions;
    pred_positions.reserve(N + 1);
    for (int i = 0; i <= N; i++) {
        double x_step[NX];
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "x", x_step);
        pred_positions.push_back({x_step[0], x_step[1], x_step[2]});
    }

    std::vector<UavControlOutput> horizon_outputs;
    horizon_outputs.reserve(N);
    for (int i = 0; i < N; i++) {
        UavControlOutput step_data;
        double u_val[NU] = {0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i, "u", u_val);

        double x_next[NX] = {0.0};
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, i + 1, "x", x_next);

        step_data.u_opt[0] = x_next[9];
        step_data.u_opt[1] = x_next[10];
        step_data.u_opt[2] = x_next[12];
        step_data.psi_cmd = x_next[11];

        kinematics::computeDesiredQuaternion(x_next[9], x_next[10], x_next[11], step_data.q_d);

        step_data.thrust_normalized = kinematics::thrustToThrottle(x_next[12], params_.mass, params_.gravity, params_.hover_throttle,
                                                                   params_.rotor_speed_min, params_.rotor_speed_max);

        step_data.predicted_positions = pred_positions;

        if (trajectory_type_ == TrajectoryType::EXTERNAL && external_reference_path_.size() >= static_cast<size_t>(N + 1)) {
            step_data.current_reference = {external_reference_path_[i].px, external_reference_path_[i].py, external_reference_path_[i].pz};
            step_data.current_reference_velocity = {external_reference_path_[i].vx, external_reference_path_[i].vy, external_reference_path_[i].vz};
        } else {
            step_data.current_reference = current_reference_;
            step_data.current_reference_velocity = {0.0, 0.0, 0.0};
        }

        step_data.reference_path.clear();
        if (trajectory_type_ == TrajectoryType::EXTERNAL) {
            for (const auto& pt : external_reference_path_) {
                step_data.reference_path.push_back({pt.px, pt.py, pt.pz});
            }
        } else {
            step_data.reference_path.push_back({current_reference_[0], current_reference_[1], current_reference_[2]});
        }

        horizon_outputs.push_back(step_data);
    }

    return horizon_outputs;
}

void UavMpcPipeline::setupOcpSolver(const double x0[13]) {
    auto capsule = (uav_tethered_solver_capsule*)acados_ocp_capsule_;
    const int N = capsule->nlp_solver_plan->N;
    auto cfg = capsule->nlp_config; auto dims = capsule->nlp_dims; auto in = capsule->nlp_in; auto out = capsule->nlp_out;

    ocp_nlp_constraints_model_set(cfg, dims, in, out, 0, "lbx", const_cast<double*>(x0));
    ocp_nlp_constraints_model_set(cfg, dims, in, out, 0, "ubx", const_cast<double*>(x0));

    // State bounds on [z, vz, phi, theta] (soft) and T (hard), nodes 1..N
    double lbx[5] = {params_.z_min, -params_.v_z_max, -params_.phi_max, -params_.theta_max, params_.thrust_min};
    double ubx[5] = {NO_LIMIT, params_.v_z_max, params_.phi_max, params_.theta_max, params_.thrust_max};
    for (int i = 1; i <= N; i++) {
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "ubx", ubx);
    }

    // Tether and speed bounds on h = [d^2, d_dot, |v_xy|]: d^2 <= d_max^2, d_dot <= d_dot_max, |v_xy| <= v_xy_max
    double lh_val[3] = {0.0, -20.0, 0.0};
    double uh_val[3] = {
        use_tether_ ? (L_tether_ * L_tether_) : NO_LIMIT,
        use_tether_ ? params_.d_dot_max : NO_LIMIT,
        params_.v_xy_max
    };
    for (int i = 1; i <= N; i++) {
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "lh", lh_val);
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "uh", uh_val);
    }

    // Reference trajectory over the horizon: yref = [p, v, 0 (tilt), 0, 0 (yaw residual), 0 (T - m g), 0 (inputs)]
    const bool external = (trajectory_type_ == TrajectoryType::EXTERNAL &&
                           external_reference_path_.size() >= static_cast<size_t>(N + 1));
    for (int i = 0; i <= N; i++) {
        double rx, ry, rz, rvx = 0.0, rvy = 0.0, rvz = 0.0;
        if (external) {
            const auto& pt = external_reference_path_[i];
            rx = pt.px; ry = pt.py; rz = pt.pz; rvx = pt.vx; rvy = pt.vy; rvz = pt.vz;
        } else {
            rx = current_reference_[0]; ry = current_reference_[1]; rz = current_reference_[2];
        }
        double proj_x, proj_y, proj_z;
        tether::projectReferenceToTetherLimit(
            i, Ts_, rx, ry, rz, anchor_x_, anchor_y_, has_boat_horizon_, boat_horizon_,
            L_tether_, use_tether_, proj_x, proj_y, proj_z);

        if (i < N) {
            double yref[NY] = {proj_x, proj_y, proj_z, rvx, rvy, rvz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            ocp_nlp_cost_model_set(cfg, dims, in, i, "yref", yref);
        } else {
            double yref_e[NY_E] = {proj_x, proj_y, proj_z, rvx, rvy, rvz, 0.0, 0.0, 0.0, 0.0, 0.0};
            ocp_nlp_cost_model_set(cfg, dims, in, N, "yref", yref_e);
        }
    }
    if (external) {
        current_reference_ = {external_reference_path_[0].px, external_reference_path_[0].py, external_reference_path_[0].pz};
        current_reference_velocity_ = {external_reference_path_[0].vx, external_reference_path_[0].vy, external_reference_path_[0].vz};
    } else {
        current_reference_velocity_ = {0.0, 0.0, 0.0};
    }

    // Parameters per node: tether attachment point (position, velocity), yaw reference point, tether switch.
    // The UAV looks at the attachment point; without tether the reference point is the UAV itself (yaw is free).
    // To align the yaw with a given heading, the reference point is moved far along that heading from the
    // attachment point, so that its bearing from the UAV is the heading (error below |p - anchor| / distance rad).
    constexpr double kHeadingPointDistance = 100.0;  // m
    const std::array<double, 3> disturbance = observer_enabled_ ? observer_.estimate() : std::array<double, 3>{0.0, 0.0, 0.0};
    for (int i = 0; i <= N; i++) {
        auto b_state = tether::getInterpolatedBoatState(i, Ts_, anchor_x_, anchor_y_, anchor_z_, has_boat_horizon_, boat_horizon_);
        double b_ref_x = use_tether_ ? b_state.px : x0[0];
        double b_ref_y = use_tether_ ? b_state.py : x0[1];
        if (use_tether_ && yaw_heading_enabled_) {
            b_ref_x += kHeadingPointDistance * std::cos(yaw_heading_);
            b_ref_y += kHeadingPointDistance * std::sin(yaw_heading_);
        }
        double p_params[NP] = {
            b_state.px, b_state.py, b_state.pz,
            b_state.vx, b_state.vy, b_state.vz,
            b_ref_x,
            b_ref_y,
            use_tether_ ? 1.0 : 0.0,
            disturbance[0], disturbance[1], disturbance[2],
            tether_model_ ? 1.0 : 0.0
        };
        uav_tethered_acados_update_params(capsule, i, p_params, NP);
    }
}

} // namespace uav_mpc
