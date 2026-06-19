#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "trajectory_generator/trajectory_generator_pipeline.hpp"

namespace trajectory_generator {

class UavTrajectoryCircleNode : public rclcpp::Node {
public:
    UavTrajectoryCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TrajectoryPipeline> pipeline_;
};

} // namespace trajectory_generator
