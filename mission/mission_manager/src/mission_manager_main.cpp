#include <rclcpp/rclcpp.hpp>
#include "mission_manager/mission_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_manager::MissionManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
