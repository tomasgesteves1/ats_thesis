#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/**
 * @brief Node that bridges Gazebo Ground Truth Odometry to TF.
 * 
 * It simplifies the coordinate system by using the Gazebo 'world' frame as the
 * absolute reference for both the boat and the drone.
 */
class FrameAligner : public rclcpp::Node {
public:
    FrameAligner() : Node("frame_aligner") {
        // Parameters
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<std::string>("boat_frame", "boat/base_link");
        this->declare_parameter<std::string>("drone_frame", "drone/base_link");
        
        this->declare_parameter<std::string>("boat_odom_topic", "/boat/ground_truth/odometry");
        this->declare_parameter<std::string>("drone_odom_topic", "/drone/ground_truth/odometry");
        this->declare_parameter<double>("drone_z_offset", -0.24);

        world_frame_ = this->get_parameter("world_frame").as_string();
        boat_frame_ = this->get_parameter("boat_frame").as_string();
        drone_frame_ = this->get_parameter("drone_frame").as_string();
        drone_z_offset_ = this->get_parameter("drone_z_offset").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        auto qos = rclcpp::SensorDataQoS();

        // Subscriptions
        boat_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("boat_odom_topic").as_string(), qos,
            std::bind(&FrameAligner::boat_callback, this, std::placeholders::_1));

        drone_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("drone_odom_topic").as_string(), qos,
            std::bind(&FrameAligner::drone_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Simplified Frame Aligner: Bridging Gazebo Ground Truth to TF.");
    }

private:
    void boat_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        publish_transform(msg, boat_frame_);
    }

    void drone_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        publish_transform(msg, drone_frame_, drone_z_offset_);
    }

    void publish_transform(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& child_frame, double z_offset = 0.0) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg->header.stamp;
        t.header.frame_id = world_frame_;
        t.child_frame_id = child_frame;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z + z_offset;
        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string world_frame_, boat_frame_, drone_frame_;
    double drone_z_offset_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameAligner>());
    rclcpp::shutdown();
    return 0;
}
