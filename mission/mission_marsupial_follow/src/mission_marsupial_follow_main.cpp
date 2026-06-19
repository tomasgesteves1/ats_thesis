#include <rclcpp/rclcpp.hpp>
#include "mission_marsupial_follow/mission_marsupial_follow_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_marsupial_follow::MissionMarsupialFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
