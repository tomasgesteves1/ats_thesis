#include <rclcpp/rclcpp.hpp>
#include "mission_usv_circle/mission_usv_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_usv_circle::MissionUsvCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
