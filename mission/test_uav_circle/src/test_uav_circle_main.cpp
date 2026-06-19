#include <rclcpp/rclcpp.hpp>
#include "test_uav_circle/test_uav_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_uav_circle::TestUavCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
