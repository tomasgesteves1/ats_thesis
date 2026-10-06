#include <rclcpp/rclcpp.hpp>
#include "mocap_bridge/mocap_bridge_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mocap_bridge::MocapBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
