#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "test_uav_circle/test_uav_circle_pipeline.hpp"

namespace test_uav_circle {

class TestUavCircleNode : public rclcpp::Node {
public:
    TestUavCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUavCirclePipeline> pipeline_;
};

} // namespace test_uav_circle
