#pragma once

#include "mocap_bridge/mocap_bridge_kinematics.hpp"

namespace mocap_bridge {

struct BridgeConfig {
    float pos_variance{0.001f};
    float ang_variance{0.0001f};
    float vel_variance{0.001f};
};

class MocapBridgePipeline {
public:
    explicit MocapBridgePipeline(const BridgeConfig& config);
    kinematics::NedOdometry process(const kinematics::EnuOdometry& in) const;
    const BridgeConfig& getConfig() const { return config_; }

private:
    BridgeConfig config_;
};

} // namespace mocap_bridge
