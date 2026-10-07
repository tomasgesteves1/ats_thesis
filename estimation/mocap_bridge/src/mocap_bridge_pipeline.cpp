#include "mocap_bridge/mocap_bridge_pipeline.hpp"

namespace mocap_bridge {

MocapBridgePipeline::MocapBridgePipeline(const BridgeConfig& config)
    : config_(config) {}

kinematics::NedOdometry MocapBridgePipeline::process(const kinematics::EnuOdometry& in) const {
    return processEnuToNed(in);
}

kinematics::NedOdometry MocapBridgePipeline::processEnuToNed(const kinematics::EnuOdometry& in) const {
    kinematics::NedOdometry out{};
    kinematics::convertEnuToNed(in, out);
    return out;
}

kinematics::EnuOdometry MocapBridgePipeline::processNedToEnu(const kinematics::NedOdometry& in) const {
    kinematics::EnuOdometry out{};
    kinematics::convertNedToEnu(in, out);
    return out;
}

kinematics::EulerAngles MocapBridgePipeline::computeEuler(const std::array<double, 4>& q_xyzw) const {
    return kinematics::quaternionToEuler(q_xyzw);
}

kinematics::OdometryError MocapBridgePipeline::computeError(
    const kinematics::EnuOdometry& gt, const kinematics::EnuOdometry& est) const {
    return kinematics::computeError(gt, est);
}

} // namespace mocap_bridge
