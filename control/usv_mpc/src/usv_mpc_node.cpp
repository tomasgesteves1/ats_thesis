#include "usv_mpc/usv_mpc_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <array>

namespace usv_mpc {

UsvMpcNode::UsvMpcNode() 
    : Node("usv_mpc_node"),
      target_initialized_(false),
      last_uav_horizon_time_(0, 0, RCL_ROS_TIME)
{
    pipeline_ = std::make_unique<UsvMpcPipeline>();

    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<std::string>("trajectory_type", "hold");
    this->declare_parameter<double>("control_period", 0.1);
    this->declare_parameter<bool>("mode_open_loop", false);
    this->declare_parameter<bool>("use_tether", true);
    this->declare_parameter<double>("tether_max_length", 15.0);   // d_max (m)
    this->declare_parameter<double>("tether_anchor_height", 1.25); // z_t (m): deck height above the inertial zero (water surface)
    this->declare_parameter<double>("uav_horizon_period", 0.02);  // UAV prediction step (s)
    this->declare_parameter<double>("uav_horizon_timeout", 0.5);  // Staleness limit of the UAV horizon (s)

    // Relative subscriptions & publishers (Rule 4 of CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UsvMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UsvMpcNode::targetCallback, this, std::placeholders::_1));

    trajectory_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_path", 10, std::bind(&UsvMpcNode::trajectoryPathCallback, this, std::placeholders::_1));

    uav_horizon_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "uav_horizon", 10, std::bind(&UsvMpcNode::uavHorizonCallback, this, std::placeholders::_1));

    // Output: thrust (N) and azimuth (rad) commands of each thruster
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/left/thrust", 10);
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/right/thrust", 10);
    left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/left/pos", 10);
    right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/right/pos", 10);
    solve_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("usv_solve_time_ms", 10);

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

void UsvMpcNode::uavHorizonCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_uav_horizon_time_ = this->now();
    const double dt_uav = this->get_parameter("uav_horizon_period").as_double();
    const double Ts = this->get_parameter("control_period").as_double();
    const int stride = std::max(1, static_cast<int>(std::round(Ts / dt_uav)));

    // Resample the UAV horizon (step dt_uav) at the USV prediction nodes (step Ts); the nodes
    // beyond the end of the UAV horizon are left out, so the tether constraint is enforced up to N_c.
    std::vector<std::array<double, 3>> horizon;
    for (size_t k = 0; k < msg->poses.size(); k += stride) {
        const auto& p = msg->poses[k].pose.position;
        horizon.push_back({p.x, p.y, p.z});
    }
    pipeline_->setUavHorizon(horizon);
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

    // Tether coupling: used only while a fresh UAV horizon is available
    bool use_tether = this->get_parameter("use_tether").as_bool();
    if (use_tether && last_uav_horizon_time_.nanoseconds() > 0) {
        const double age = (this->now() - last_uav_horizon_time_).seconds();
        if (age > this->get_parameter("uav_horizon_timeout").as_double()) {
            pipeline_->setUavHorizon({});
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "UAV horizon timeout (%.2f s). Tether constraint disabled.", age);
        }
    } else {
        pipeline_->setUavHorizon({});
    }
    pipeline_->setTetherParameters(this->get_parameter("tether_max_length").as_double(),
                                   this->get_parameter("tether_anchor_height").as_double());

    const auto t_start = std::chrono::steady_clock::now();
    ThrusterCommand cmd = pipeline_->computeControl();
    const double solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    std_msgs::msg::Float64 solve_msg;
    solve_msg.data = solve_ms;
    solve_time_pub_->publish(solve_msg);

    std_msgs::msg::Float64 msg;
    msg.data = cmd[0]; left_thrust_pub_->publish(msg);
    msg.data = cmd[1]; right_thrust_pub_->publish(msg);
    msg.data = cmd[2]; left_pos_pub_->publish(msg);
    msg.data = cmd[3]; right_pos_pub_->publish(msg);

    // Publish predicted horizon trajectory
    auto pred_states = pipeline_->getPredictedStates();
    auto horizon_path = nav_msgs::msg::Path();
    horizon_path.header.stamp = this->now();
    horizon_path.header.frame_id = "world"; // Global simulation frame

    for (const auto& state : pred_states) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = horizon_path.header.stamp;
        pose.header.frame_id = horizon_path.header.frame_id;
        pose.pose.position.x = state[0];
        pose.pose.position.y = state[1];
        pose.pose.position.z = 0.0;

        // Convert heading (yaw) to quaternion analytically
        double half_yaw = state[2] * 0.5;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(half_yaw);
        pose.pose.orientation.w = std::cos(half_yaw);

        horizon_path.poses.push_back(pose);
    }
    mpc_horizon_pub_->publish(horizon_path);
}

} // namespace usv_mpc
