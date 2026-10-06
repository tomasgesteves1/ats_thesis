#include "mocap_bridge/mocap_bridge_pipeline.hpp"

namespace mocap_bridge {

MocapBridgePipeline::MocapBridgePipeline(const BridgeConfig& config)
    : config_(config) {}

kinematics::NedOdometry MocapBridgePipeline::process(const kinematics::EnuOdometry& in) const {
    kinematics::NedOdometry out{};
    kinematics::convertEnuToNed(in, out);
    return out;
}

} // namespace mocap_bridge
