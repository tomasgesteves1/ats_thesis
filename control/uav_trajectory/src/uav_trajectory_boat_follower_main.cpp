#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "uav_trajectory/uav_trajectory_boat_follower_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uav_trajectory::UavTrajectoryBoatFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
