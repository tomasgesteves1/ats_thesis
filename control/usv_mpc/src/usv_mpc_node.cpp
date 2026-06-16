#include "usv_mpc/usv_mpc_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace usv_mpc {

UsvMpcNode::UsvMpcNode() : Node("usv_mpc_node") {
    pipeline_ = std::make_unique<UsvMpcPipeline>();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UsvMpcNode::odomCallback, this, std::placeholders::_1));

    // O output será standard Twist (cmd_vel) lido pelo VRX
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer a 10Hz (0.1s) using node's clock (sim time)
    timer_ = this->create_timer(
        std::chrono::milliseconds(100), std::bind(&UsvMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "USV Kinematic Planner Node inicializado.");
}

void UsvMpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Zero matemática exaustiva. Apenas converter quaternião para yaw.
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::vector<double> state = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw
    };
    pipeline_->updateState(state);
}

void UsvMpcNode::controlLoop() {
    std::vector<double> control_cmd = pipeline_->computeControl();

    geometry_msgs::msg::Twist cmd_msg;
    if (control_cmd.size() == 2) {
        cmd_msg.linear.x = control_cmd[0];   // Surge velocity (v)
        cmd_msg.angular.z = control_cmd[1];  // Yaw rate (w)
        cmd_pub_->publish(cmd_msg);
    }
}

} // namespace usv_mpc
