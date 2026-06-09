#include <rclcpp/rclcpp.hpp>
#include "moordyn_tether/moordyn_tether_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<moordyn_tether::MoordynTetherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
