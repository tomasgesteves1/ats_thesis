#pragma once

#include <vector>

namespace uav_mpc {

struct UavControlOutput {
    double q_d[4]; // w, x, y, z
    double thrust_normalized;
    std::vector<std::vector<double>> predicted_positions; // N+1 points of [x, y, z]
    std::vector<double> current_reference; // [x, y, z]
};

class UavMpcPipeline {
public:
    UavMpcPipeline();
    ~UavMpcPipeline();

    void updateState(const std::vector<double>& state);
    void updateOrientation(double qx, double qy, double qz, double qw);
    void setReference(const std::vector<double>& ref);
    UavControlOutput computeControl();

private:
    std::vector<double> current_state_;
    void* acados_ocp_capsule_;
    
    // Atitude atual do drone
    double qx_, qy_, qz_, qw_;
    double target_yaw_;
    bool target_initialized_;
    std::vector<double> current_reference_;

    // Funções auxiliares de matemática (atitude)
    void quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) const;
    void rotationMatrixToQuaternion(double R[3][3], float q[4]) const;
};

} // namespace uav_mpc
