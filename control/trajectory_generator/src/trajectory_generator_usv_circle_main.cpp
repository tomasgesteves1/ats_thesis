#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "trajectory_generator/trajectory_generator_usv_circle_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trajectory_generator::UsvTrajectoryCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
