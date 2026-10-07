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
    kinematics::NedOdometry processEnuToNed(const kinematics::EnuOdometry& in) const;
    kinematics::EnuOdometry processNedToEnu(const kinematics::NedOdometry& in) const;
    kinematics::EulerAngles computeEuler(const std::array<double, 4>& q_xyzw) const;
    kinematics::OdometryError computeError(const kinematics::EnuOdometry& gt, const kinematics::EnuOdometry& est) const;

    const BridgeConfig& getConfig() const { return config_; }

private:
    BridgeConfig config_;
};

} // namespace mocap_bridge
