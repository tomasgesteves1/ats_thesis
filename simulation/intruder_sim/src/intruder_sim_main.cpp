#include <rclcpp/rclcpp.hpp>

#include "intruder_sim/intruder_sim_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<intruder_sim::IntruderSimNode>());
    rclcpp::shutdown();
    return 0;
}
