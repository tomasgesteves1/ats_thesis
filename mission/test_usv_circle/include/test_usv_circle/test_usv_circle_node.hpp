#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include "test_usv_circle/test_usv_circle_pipeline.hpp"

namespace test_usv_circle {

class TestUsvCircleNode : public rclcpp::Node {
public:
    TestUsvCircleNode();

private:
    void timerCallback();
    double get_double_param(const std::string& name);
    int get_int_param(const std::string& name);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<TestUsvCirclePipeline> pipeline_;
};

} // namespace test_usv_circle
