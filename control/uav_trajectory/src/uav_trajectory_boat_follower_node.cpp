#include "uav_trajectory/uav_trajectory_boat_follower_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace uav_trajectory {

// Helper to rotate vector from body frame to world frame using quaternion
static void rotateVectorByQuaternion(double qx, double qy, double qz, double qw, 
                                     const double v_in[3], double v_out[3]) 
{
    double q_xyz[3] = {qx, qy, qz};
    
    // cross(q_xyz, v_in)
    double cross1[3] = {
        q_xyz[1] * v_in[2] - q_xyz[2] * v_in[1],
        q_xyz[2] * v_in[0] - q_xyz[0] * v_in[2],
        q_xyz[0] * v_in[1] - q_xyz[1] * v_in[0]
    };
    
    // cross1 + q_w * v_in
    double temp[3] = {
        cross1[0] + qw * v_in[0],
        cross1[1] + qw * v_in[1],
        cross1[2] + qw * v_in[2]
    };
    
    // cross(q_xyz, temp)
    double cross2[3] = {
        q_xyz[1] * temp[2] - q_xyz[2] * temp[1],
        q_xyz[2] * temp[0] - q_xyz[0] * temp[2],
        q_xyz[0] * temp[1] - q_xyz[1] * temp[0]
    };
    
    v_out[0] = v_in[0] + 2.0 * cross2[0];
    v_out[1] = v_in[1] + 2.0 * cross2[1];
    v_out[2] = v_in[2] + 2.0 * cross2[2];
}

UavTrajectoryBoatFollowerNode::UavTrajectoryBoatFollowerNode() 
    : Node("uav_trajectory_boat_follower_node"),
      latest_boat_odom_(nullptr)
{
    // Declare parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<double>("offset_x", 0.0);
    this->declare_parameter<double>("offset_y", 0.0);
    this->declare_parameter<double>("offset_z", 5.0);
    this->declare_parameter<int>("horizon_stages", 50);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<double>("update_rate_hz", 50.0);
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<bool>("predict_movement", true);

    pipeline_ = std::make_unique<UavTrajectoryPipeline>();

    // Relative subscriber & publisher (Rule 4 of CODE_STANDARDS.md)
    auto qos = rclcpp::SensorDataQoS();
    boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "boat_odom", qos, 
        std::bind(&UavTrajectoryBoatFollowerNode::boatOdomCallback, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);

    double hz = this->get_parameter("update_rate_hz").as_double();
    double period_ms = 1000.0 / std::max(0.1, hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&UavTrajectoryBoatFollowerNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "UAV Trajectory Boat Follower Node initialized.");
}

void UavTrajectoryBoatFollowerNode::boatOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_boat_odom_ = msg;
}

void UavTrajectoryBoatFollowerNode::timerCallback() {
    if (!latest_boat_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for boat odometry message...");
        return;
    }

    double now_sec = this->get_clock()->now().seconds();

    double offset_x = this->get_parameter("offset_x").as_double();
    double offset_y = this->get_parameter("offset_y").as_double();
    double offset_z = this->get_parameter("offset_z").as_double();
    int steps = this->get_parameter("horizon_stages").as_int();
    double dt = this->get_parameter("control_period").as_double();
    std::string world_frame = this->get_parameter("world_frame").as_string();
    bool predict_movement = this->get_parameter("predict_movement").as_bool();

    // Extract current boat position and orientation in world frame
    double boat_x = latest_boat_odom_->pose.pose.position.x;
    double boat_y = latest_boat_odom_->pose.pose.position.y;
    double boat_z = 0.0; // Treat boat as 2D (assume constant water level to avoid vertical heave coupling)

    double qx = latest_boat_odom_->pose.pose.orientation.x;
    double qy = latest_boat_odom_->pose.pose.orientation.y;
    double qz = latest_boat_odom_->pose.pose.orientation.z;
    double qw = latest_boat_odom_->pose.pose.orientation.w;

    // Rotate linear velocities from boat frame to world frame
    double v_body[3] = {
        latest_boat_odom_->twist.twist.linear.x,
        latest_boat_odom_->twist.twist.linear.y,
        latest_boat_odom_->twist.twist.linear.z
    };
    double v_world[3] = {0.0, 0.0, 0.0};
    rotateVectorByQuaternion(qx, qy, qz, qw, v_body, v_world);
    v_world[2] = 0.0; // Ignore vertical velocity to prevent high-frequency wave-induced Z tracking

    auto points = pipeline_->generateBoatFollower(
        now_sec, boat_x, boat_y, boat_z, v_world[0], v_world[1], v_world[2],
        offset_x, offset_y, offset_z, steps, dt, predict_movement
    );

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = world_frame;
    path_msg.header.stamp = this->get_clock()->now();

    for (const auto& pt : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = world_frame;
        pose.header.stamp = path_msg.header.stamp;
        pose.pose.position.x = pt.px;
        pose.pose.position.y = pt.py;
        pose.pose.position.z = pt.pz;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

} // namespace uav_trajectory
