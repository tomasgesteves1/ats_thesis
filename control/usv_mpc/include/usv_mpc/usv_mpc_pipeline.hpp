#pragma once

#include <vector>

namespace usv_mpc {

class UsvMpcPipeline {
public:
    UsvMpcPipeline();
    ~UsvMpcPipeline();

    void updateState(const std::vector<double>& state);
    std::vector<double> computeControl();

private:
    std::vector<double> current_state_;
    void* acados_ocp_capsule_;
};

} // namespace usv_mpc
