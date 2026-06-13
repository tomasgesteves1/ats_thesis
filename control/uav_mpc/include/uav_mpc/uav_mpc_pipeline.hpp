#pragma once

#include <vector>
#include "uav_mpc/uav_mpc_trajectory.hpp"

namespace uav_mpc {

enum class TrajectoryType {
    HOLD = 0,
    CIRCLE = 1
};

struct UavControlOutput {
    double q_d[4]; // w, x, y, z
    double thrust_normalized;
    std::vector<std::vector<double>> predicted_positions; // N+1 points of [x, y, z]
    std::vector<double> current_reference; // [x, y, z]
    std::vector<std::vector<double>> reference_path; // Points of the reference trajectory for visualization
    double mpc_tether_force_mag; // Tether force magnitude assumed by the MPC model (N)
};

class UavMpcPipeline {
public:
    UavMpcPipeline();
    ~UavMpcPipeline();

    void updateState(const std::vector<double>& state);
    void updateOrientation(double qx, double qy, double qz, double qw);
    void updateAnchorPosition(double x, double y, double z);
    void updateTetherLength(double length);
    void setReference(const std::vector<double>& ref);
    void setTrajectoryType(TrajectoryType type);
    void configureCircle(double radius, double omega, double height, double center_x = 0.0, double center_y = 0.0);
    void setUseTether(bool use_tether);
    void setVelocityLimit(double v_max);
    void setInputLimit(double u_max);
    void setAttitudeTimeConstant(double tau);
    UavControlOutput computeControl();

private:
    std::vector<double> current_state_;
    void* acados_ocp_capsule_;
    
    // Atitude atual do drone
    double qx_, qy_, qz_, qw_;
    double target_yaw_;
    bool target_initialized_;
    std::vector<double> current_reference_;

    // Posição global da âncora do cabo e comprimento
    double anchor_x_, anchor_y_, anchor_z_;
    double L_tether_;
    bool use_tether_;
    double v_max_;
    double u_max_;
    double tau_;

    // Trajetória e tempo
    TrajectoryType trajectory_type_;
    UavMpcTrajectory trajectory_gen_;
    double time_;

    // Funções auxiliares de matemática (atitude)
    void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) const;
    void rotationMatrixToQuaternion(double R[3][3], float q[4]) const;
    void rotateVectorByQuaternion(double qx, double qy, double qz, double qw, const double v_in[3], double v_out[3]) const;
};

} // namespace uav_mpc
