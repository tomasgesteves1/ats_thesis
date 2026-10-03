#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include "test_uav_line/test_uav_line_pipeline.hpp"

namespace test_uav_line {

class TestUavLineNode : public rclcpp::Node {
public:
    TestUavLineNode();

private:
    void timerCallback();

    std::unique_ptr<TestUavLinePipeline> pipeline_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double start_time_{-1.0};
    bool mission_finished_logged_{false};
};

} // namespace test_uav_line
