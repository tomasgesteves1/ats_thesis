#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

class FrameAligner : public rclcpp::Node {
public:
    FrameAligner() : Node("frame_aligner") {
        // 1. Declare Parameters
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<std::string>("boat_frame", "boat/base_link");
        this->declare_parameter<std::string>("boat_footprint_frame", "boat/footprint");
        this->declare_parameter<std::string>("drone_map_frame", "drone/map");
        this->declare_parameter<std::string>("drone_base_frame", "drone/base_link");
        
        this->declare_parameter<std::string>("boat_odom_topic", "/boat/ground_truth/odometry");
        this->declare_parameter<std::string>("drone_gt_topic", "/drone/ground_truth/odometry");
        this->declare_parameter<std::string>("drone_px4_odom_topic", "/px4_1/fmu/out/vehicle_odometry");

        this->declare_parameter<double>("stabilized_z", 0.0);

        // 2. Get Parameters
        world_frame_ = this->get_parameter("world_frame").as_string();
        boat_frame_ = this->get_parameter("boat_frame").as_string();
        boat_footprint_frame_ = this->get_parameter("boat_footprint_frame").as_string();
        drone_map_frame_ = this->get_parameter("drone_map_frame").as_string();
        drone_base_frame_ = this->get_parameter("drone_base_frame").as_string();
        stabilized_z_ = this->get_parameter("stabilized_z").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 3. Subscriptions (using SensorDataQoS for BestEffort)
        auto qos = rclcpp::SensorDataQoS();

        boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("boat_odom_topic").as_string(), qos,
            std::bind(&FrameAligner::boat_odom_callback, this, std::placeholders::_1));

        drone_gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("drone_gt_topic").as_string(), qos,
            std::bind(&FrameAligner::drone_gt_callback, this, std::placeholders::_1));

        drone_px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            this->get_parameter("drone_px4_odom_topic").as_string(), qos,
            std::bind(&FrameAligner::drone_px4_odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Frame Aligner Node updated with QoS and TF batching.");
    }

private:
    void boat_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // --- 1. Get Global Orientation ---
        tf2::Quaternion q_world_to_base;
        tf2::fromMsg(msg->pose.pose.orientation, q_world_to_base);

        // --- 2. Extract Yaw for Footprint ---
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_world_to_base).getRPY(roll, pitch, yaw);
        
        tf2::Quaternion q_world_to_footprint;
        q_world_to_footprint.setRPY(0, 0, yaw);

        // --- 3. Prepare world -> boat/footprint ---
        geometry_msgs::msg::TransformStamped t_footprint;
        t_footprint.header.stamp = msg->header.stamp;
        t_footprint.header.frame_id = world_frame_;
        t_footprint.child_frame_id = boat_footprint_frame_;
        t_footprint.transform.translation.x = msg->pose.pose.position.x;
        t_footprint.transform.translation.y = msg->pose.pose.position.y;
        t_footprint.transform.translation.z = stabilized_z_;
        t_footprint.transform.rotation = tf2::toMsg(q_world_to_footprint);

        // --- 4. Calculate relative transform: boat/footprint -> boat/base_link ---
        tf2::Quaternion q_footprint_to_base = q_world_to_footprint.inverse() * q_world_to_base;
        q_footprint_to_base.normalize(); // Ensure unit length to avoid visualization glitches

        geometry_msgs::msg::TransformStamped t_base;

        t_base.header.stamp = msg->header.stamp;
        t_base.header.frame_id = boat_footprint_frame_;
        t_base.child_frame_id = boat_frame_;
        t_base.transform.translation.x = 0.0;
        t_base.transform.translation.y = 0.0;
        t_base.transform.translation.z = msg->pose.pose.position.z - stabilized_z_;
        t_base.transform.rotation = tf2::toMsg(q_footprint_to_base);

        // --- 5. Batch and Send ---
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(t_footprint);
        transforms.push_back(t_base);
        tf_broadcaster_->sendTransform(transforms);
    }

    void drone_gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_gt_pose_ = msg;
        if (!calibrated_ && last_px4_odom_) perform_calibration();
    }

    void drone_px4_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        last_px4_odom_ = msg;
        if (!calibrated_ && last_gt_pose_) perform_calibration();
        if (calibrated_) publish_drone_local_frame(msg);
    }

    void perform_calibration() {
        double world_x = last_gt_pose_->pose.pose.position.x;
        double world_y = last_gt_pose_->pose.pose.position.y;
        double world_z = last_gt_pose_->pose.pose.position.z;
        double px4_x = last_px4_odom_->position[1];
        double px4_y = last_px4_odom_->position[0];
        double px4_z = -last_px4_odom_->position[2];
        map_origin_x_ = world_x - px4_x;
        map_origin_y_ = world_y - px4_y;
        map_origin_z_ = world_z - px4_z;
        publish_static_map_transform();
        calibrated_ = true;
        RCLCPP_INFO(this->get_logger(), "Calibration successful! Origin: [%.3f, %.3f, %.3f]", 
                    map_origin_x_, map_origin_y_, map_origin_z_);
    }

    void publish_drone_local_frame(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = drone_map_frame_;
        t.child_frame_id = drone_base_frame_;
        t.transform.translation.x = msg->position[1];
        t.transform.translation.y = msg->position[0];
        t.transform.translation.z = -msg->position[2];
        t.transform.rotation.x = msg->q[2];
        t.transform.rotation.y = msg->q[1];
        t.transform.rotation.z = -msg->q[3];
        t.transform.rotation.w = msg->q[0];
        tf_broadcaster_->sendTransform(t);
    }

    void publish_static_map_transform() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = world_frame_;
        t.child_frame_id = drone_map_frame_;
        t.transform.translation.x = map_origin_x_;
        t.transform.translation.y = map_origin_y_;
        t.transform.translation.z = map_origin_z_;
        t.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::string world_frame_, boat_frame_, boat_footprint_frame_, drone_map_frame_, drone_base_frame_;
    double stabilized_z_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_gt_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone_px4_odom_sub_;
    nav_msgs::msg::Odometry::SharedPtr last_gt_pose_;
    px4_msgs::msg::VehicleOdometry::SharedPtr last_px4_odom_;
    bool calibrated_{false};
    double map_origin_x_{0.0}, map_origin_y_{0.0}, map_origin_z_{0.0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameAligner>());
    rclcpp::shutdown();
    return 0;
}
