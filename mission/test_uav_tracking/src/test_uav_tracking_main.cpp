#include <rclcpp/rclcpp.hpp>
#include "test_uav_tracking/test_uav_tracking_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_uav_tracking::TestUavTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
