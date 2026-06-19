#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "uav_trajectory/uav_trajectory_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uav_trajectory::UavTrajectoryCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
