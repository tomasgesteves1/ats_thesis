#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include "test_usv_circle/test_usv_circle_pipeline.hpp"

namespace test_usv_circle {

class TestUsvCircleNode : public rclcpp::Node {
public:
    TestUsvCircleNode();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUsvCirclePipeline> pipeline_;
};

} // namespace test_usv_circle
