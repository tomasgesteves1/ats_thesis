#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "uav_trajectory/uav_trajectory_pipeline.hpp"

namespace uav_trajectory {

class UavTrajectoryCircleNode : public rclcpp::Node {
public:
    UavTrajectoryCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UavTrajectoryPipeline> pipeline_;
};

} // namespace uav_trajectory
