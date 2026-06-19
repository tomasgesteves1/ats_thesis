#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "mission_drone_circle/mission_drone_circle_pipeline.hpp"

namespace mission_drone_circle {

class MissionDroneCircleNode : public rclcpp::Node {
public:
    MissionDroneCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<MissionDroneCirclePipeline> pipeline_;
};

} // namespace mission_drone_circle
