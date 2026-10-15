#pragma once

#include <vector>

namespace uav_mpc {

struct TrajectoryPoint {
    double px, py, pz;
    double vx, vy, vz;
};

enum class TrajectoryType {
    HOLD = 0,
    EXTERNAL = 1
};

struct UavControlOutput {
    double q_d[4]; // w, x, y, z desired orientation
    double thrust_normalized;
    double u_opt[3]; // optimal command applied: [phi_cmd, theta_cmd, T] (rad, rad, N)
    double psi_cmd;  // commanded yaw (rad)
    double predicted_attitude[3]{0.0, 0.0, 0.0}; // [roll, pitch, yaw] predicted at node PREDICTED_ATTITUDE_NODE (rad)
    std::vector<std::vector<double>> predicted_positions; // N+1 points of [x, y, z]
    std::vector<double> current_reference; // [x, y, z]
    std::vector<double> current_reference_velocity; // [vx, vy, vz]
    std::vector<std::vector<double>> reference_path; // Reference trajectory points for visualization
};

/// Run-time parameters of the controller (Section 3.3, Table 4.12); read from uav_mpc.yaml by the node
struct UavMpcParams {
    double mass{2.06};            // m (kg)
    double gravity{9.81};         // g (m/s^2)
    double z_min{2.0};            // minimum altitude (m)
    double v_xy_max{10.0};        // horizontal speed limit (m/s)
    double v_z_max{10.0};         // vertical speed limit (m/s)
    double phi_max{0.5};          // roll limit (rad)
    double theta_max{0.5};        // pitch limit (rad)
    double d_dot_max{2.0};        // maximum tether pay-out rate (m/s)
    double thrust_min{2.02};      // minimum thrust (N)
    double thrust_max{32.96};     // maximum thrust (N)
    double hover_throttle{0.618}; // normalised hover throttle
    double rotor_speed_min{150.0};   // rotor speed at zero throttle (rad/s)
    double rotor_speed_max{1000.0};  // rotor speed at full throttle (rad/s)
};

class UavMpcPipeline {
public:
    UavMpcPipeline();
    ~UavMpcPipeline();

    void updateState(const std::vector<double>& state);
    void updateOrientation(double qx, double qy, double qz, double qw);
    void updateAnchorPosition(double x, double y, double z);
    void updateBoatHorizon(const std::vector<std::vector<double>>& boat_horizon);
    void updateTetherLength(double length);
    void setReference(const std::vector<double>& ref);
    void setExternalReferencePath(const std::vector<TrajectoryPoint>& path);
    void setTrajectoryType(TrajectoryType type);
    void setUseTether(bool use_tether);
    // Yaw aligned with a given heading instead of looking at the attachment point (e.g. the boat heading for landing)
    void setYawHeading(bool enabled, double heading);
    void setParameters(const UavMpcParams& params);
    UavControlOutput computeControl(double current_time);

    // Open-loop test support: returns the full planned control horizon
    std::vector<UavControlOutput> getOpenLoopHorizon(double current_time);

private:
    void setupOcpSolver(const double x0[13]);
    void buildInitialState(double x0[13]);
    void warmStartOnce(const double x0[13]);

    std::vector<double> current_state_;
    void* acados_ocp_capsule_;
    
    // Current drone attitude
    double qx_, qy_, qz_, qw_;
    double target_yaw_;
    bool target_initialized_;
    std::vector<double> current_reference_;
    std::vector<double> current_reference_velocity_;

    // Global tether anchor position and length
    double anchor_x_, anchor_y_, anchor_z_;
    std::vector<std::vector<double>> boat_horizon_;
    bool has_boat_horizon_;
    double L_tether_;
    bool use_tether_;
    bool yaw_heading_enabled_{false};
    double yaw_heading_{0.0};
    UavMpcParams params_;

    // Trajectory generator and time
    TrajectoryType trajectory_type_;
    std::vector<TrajectoryPoint> external_reference_path_;
    static constexpr double Ts_ = 0.02;  // Control period (s)

    // Last applied attitude commands (state feedback of the augmented model); yaw is kept continuous (unwrapped)
    double last_phi_cmd_;
    double last_theta_cmd_;
    double last_psi_cmd_;
    double last_thrust_cmd_;
    double yaw_cont_;
    bool first_run_;
};

} // namespace uav_mpc
