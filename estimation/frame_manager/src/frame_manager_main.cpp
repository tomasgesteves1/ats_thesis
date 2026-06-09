#include <rclcpp/rclcpp.hpp>
#include "frame_manager/frame_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frame_manager::FrameManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
