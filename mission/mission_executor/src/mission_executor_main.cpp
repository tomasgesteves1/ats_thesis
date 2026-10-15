#include <rclcpp/rclcpp.hpp>

#include "mission_executor/mission_executor_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mission_executor::MissionExecutorNode>());
    rclcpp::shutdown();
    return 0;
}
