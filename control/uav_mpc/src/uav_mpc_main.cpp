#include "uav_mpc/uav_mpc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uav_mpc::UavMpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
