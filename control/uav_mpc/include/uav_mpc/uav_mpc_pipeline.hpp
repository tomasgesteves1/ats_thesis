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
    double u_opt[3]; // raw optimal control command from MPC: [phi_cmd, theta_cmd, a_T]
    std::vector<std::vector<double>> predicted_positions; // N+1 points of [x, y, z]
    std::vector<double> current_reference; // [x, y, z]
    std::vector<double> current_reference_velocity; // [vx, vy, vz]
    std::vector<std::vector<double>> reference_path; // Reference trajectory points for visualization
    double mpc_tether_force_mag; // Tether force magnitude assumed by the MPC model (N)
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
    void setCostWeights(double w_pos, double w_vel);
    void setTrajectoryType(TrajectoryType type);
    void setUseTether(bool use_tether);
    void setVelocityLimit(double v_max);
    void setInputLimit(double u_max);
    void setHoverThrottle(double hover_throttle);
    void setTiltMax(double tilt_max);
    UavControlOutput computeControl(double current_time);

    // Open-loop test support: returns the full planned control horizon
    std::vector<UavControlOutput> getOpenLoopHorizon(double current_time);

private:
    void setupOcpSolver(double current_time, double roll, double pitch, double yaw, const double x0[10]);

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
    double v_max_;
    double u_max_;
    double hover_throttle_;
    double tilt_max_;

    // Trajectory generator and time
    TrajectoryType trajectory_type_;
    std::vector<TrajectoryPoint> external_reference_path_;
    static constexpr double Ts_ = 0.02;  // Control period (s)

    // Variables for augmented state feedback (Option B with rate constraints)
    double last_phi_cmd_;
    double last_theta_cmd_;
    bool first_run_;
};

} // namespace uav_mpc
