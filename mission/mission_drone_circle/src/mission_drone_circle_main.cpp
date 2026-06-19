#include <rclcpp/rclcpp.hpp>
#include "mission_drone_circle/mission_drone_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_drone_circle::MissionDroneCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
