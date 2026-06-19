#include <rclcpp/rclcpp.hpp>
#include "test_usv_circle/test_usv_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_usv_circle::TestUsvCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
