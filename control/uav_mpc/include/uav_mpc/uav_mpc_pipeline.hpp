#pragma once

#include <vector>

namespace uav_mpc {

class UavMpcPipeline {
public:
    UavMpcPipeline();
    ~UavMpcPipeline();

    void updateState(const std::vector<double>& state);
    void setReference(const std::vector<double>& ref);
    std::vector<double> computeControl();

private:
    std::vector<double> current_state_;
    void* acados_ocp_capsule_;
};

} // namespace uav_mpc
