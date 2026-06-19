#pragma once

#include <vector>

namespace usv_mpc {

struct TrajectoryPoint {
    double x;
    double y;
    double psi;
    double v;
    double w;
};

class UsvMpcPipeline {
public:
    UsvMpcPipeline();
    ~UsvMpcPipeline();

    void updateState(const std::vector<double>& state);
    void setReference(const std::vector<double>& ref_pose);
    void setExternalReferencePath(const std::vector<TrajectoryPoint>& path);
    void setTrajectoryType(int type); // 0: HOLD, 1: EXTERNAL
    std::vector<double> computeControl();

private:
    std::vector<double> current_state_;
    std::vector<double> current_reference_;
    std::vector<TrajectoryPoint> external_reference_path_;
    int trajectory_type_; // 0: HOLD, 1: EXTERNAL
    void* acados_ocp_capsule_;
};

} // namespace usv_mpc
