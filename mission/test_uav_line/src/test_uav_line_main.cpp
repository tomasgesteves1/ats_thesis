#include "test_uav_line/test_uav_line_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_uav_line::TestUavLineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
