#include "usv_mpc/usv_mpc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<usv_mpc::UsvMpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
