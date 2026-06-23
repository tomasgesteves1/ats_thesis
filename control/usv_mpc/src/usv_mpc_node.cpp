#include "usv_mpc/usv_mpc_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace usv_mpc {

UsvMpcNode::UsvMpcNode() 
    : Node("usv_mpc_node"),
      target_initialized_(false) 
{
    pipeline_ = std::make_unique<UsvMpcPipeline>();

    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<std::string>("trajectory_type", "hold");
    this->declare_parameter<double>("control_period", 0.1);
    this->declare_parameter<bool>("mode_open_loop", false);

    // Relative subscriptions & publishers (Rule 4 of CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UsvMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UsvMpcNode::targetCallback, this, std::placeholders::_1));

    trajectory_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_path", 10, std::bind(&UsvMpcNode::trajectoryPathCallback, this, std::placeholders::_1));

    // Output is virtual forces and moment at CG
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("cmd_wrench", 10);

    // Publisher for visualization of MPC prediction horizon
    mpc_horizon_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_horizon", 10);

    // Timer at 10Hz (0.1s) matching control period of usv_mpc
    timer_ = this->create_timer(
        std::chrono::milliseconds(100), std::bind(&UsvMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "USV Dynamic MPC Planner Node inicializado.");
}

void UsvMpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
        yaw,
        msg->twist.twist.linear.x,  // u (surge velocity in body frame)
        msg->twist.twist.linear.y,  // v (sway velocity in body frame)
        msg->twist.twist.angular.z  // r (yaw rate)
    };
    pipeline_->updateState(state);

    if (!target_initialized_) {
        pipeline_->setReference({state[0], state[1], state[2], 0.0, 0.0, 0.0});
        target_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Holding initial boat position: X=%.2f, Y=%.2f, Yaw=%.2f", 
                    state[0], state[1], state[2]);
    }
}

void UsvMpcNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Novo setpoint recebido para o Barco: [%.2f, %.2f]", msg->x, msg->y);
    pipeline_->setReference({msg->x, msg->y, 0.0, 0.0, 0.0, 0.0});
}

void UsvMpcNode::trajectoryPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        return;
    }
    
    std::vector<TrajectoryPoint> path_points;
    const double Ts = this->get_parameter("control_period").as_double();
    size_t num_poses = msg->poses.size();
    
    path_points.resize(num_poses);
    for (size_t i = 0; i < num_poses; ++i) {
        path_points[i].x = msg->poses[i].pose.position.x;
        path_points[i].y = msg->poses[i].pose.position.y;
        
        // Convert orientation quaternion to yaw
        tf2::Quaternion q(
            msg->poses[i].pose.orientation.x,
            msg->poses[i].pose.orientation.y,
            msg->poses[i].pose.orientation.z,
            msg->poses[i].pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        path_points[i].psi = yaw;
    }
    
    // Compute feedforward linear and angular velocities using finite difference
    for (size_t i = 0; i < num_poses; ++i) {
        if (i < num_poses - 1) {
            double dx = (path_points[i+1].x - path_points[i].x) / Ts;
            double dy = (path_points[i+1].y - path_points[i].y) / Ts;
            
            // v = xdot * cos(yaw) + ydot * sin(yaw)
            path_points[i].v = dx * std::cos(path_points[i].psi) + dy * std::sin(path_points[i].psi);
            
            // w = yaw_dot
            double diff_psi = path_points[i+1].psi - path_points[i].psi;
            // Normalize yaw difference to [-pi, pi]
            while (diff_psi > M_PI) diff_psi -= 2.0 * M_PI;
            while (diff_psi < -M_PI) diff_psi += 2.0 * M_PI;
            path_points[i].w = diff_psi / Ts;
        } else {
            if (num_poses > 1) {
                path_points[i].v = path_points[i-1].v;
                path_points[i].w = path_points[i-1].w;
            } else {
                path_points[i].v = 0.0;
                path_points[i].w = 0.0;
            }
        }
    }
    
    pipeline_->setExternalReferencePath(path_points);
}

void UsvMpcNode::controlLoop() {
    bool mode_open_loop = false;
    this->get_parameter("mode_open_loop", mode_open_loop);
    if (mode_open_loop) {
        // Bypass MPC when testing in open-loop mode
        return;
    }

    std::string traj_type = this->get_parameter("trajectory_type").as_string();
    if (traj_type == "external") {
        pipeline_->setTrajectoryType(1);
    } else {
        pipeline_->setTrajectoryType(0);
    }

    std::vector<double> control_cmd = pipeline_->computeControl();

    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = this->now();
    wrench_msg.header.frame_id = "boat/base_link";

    if (control_cmd.size() == 3) {
        wrench_msg.wrench.force.x = control_cmd[0];  // X force (Surge)
        wrench_msg.wrench.force.y = control_cmd[1];  // Y force (Sway)
        wrench_msg.wrench.torque.z = control_cmd[2]; // N moment (Yaw)
        wrench_pub_->publish(wrench_msg);

        // Publish predicted horizon trajectory
        auto pred_states = pipeline_->getPredictedStates();
        auto horizon_path = nav_msgs::msg::Path();
        horizon_path.header.stamp = wrench_msg.header.stamp;
        horizon_path.header.frame_id = "world"; // Global simulation frame

        for (const auto& state : pred_states) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = horizon_path.header.stamp;
            pose.header.frame_id = horizon_path.header.frame_id;
            pose.pose.position.x = state[0];
            pose.pose.position.y = state[1];
            pose.pose.position.z = 0.0;
            
            // Convert heading (yaw) to quaternion analytically
            double yaw = state[2];
            double half_yaw = yaw * 0.5;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(half_yaw);
            pose.pose.orientation.w = std::cos(half_yaw);
            
            horizon_path.poses.push_back(pose);
        }
        mpc_horizon_pub_->publish(horizon_path);
    }
}

} // namespace usv_mpc
