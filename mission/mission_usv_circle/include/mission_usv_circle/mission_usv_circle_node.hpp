#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "mission_usv_circle/mission_usv_circle_pipeline.hpp"

namespace mission_usv_circle {

class MissionUsvCircleNode : public rclcpp::Node {
public:
    MissionUsvCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<MissionUsvCirclePipeline> pipeline_;
};

} // namespace mission_usv_circle
