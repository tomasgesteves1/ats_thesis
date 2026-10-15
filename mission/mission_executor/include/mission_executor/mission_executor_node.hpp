#ifndef MISSION_EXECUTOR__MISSION_EXECUTOR_NODE_HPP_
#define MISSION_EXECUTOR__MISSION_EXECUTOR_NODE_HPP_

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mission_executor/mission_executor_pipeline.hpp"

namespace mission_executor
{

/// ROS wrapper of the mission executor: gathers the measurements (TF, odometry, USV horizon, PX4 status), runs the
/// pipeline at a fixed rate and publishes the references of both controllers, the winch and yaw mode switches,
/// and the arm/offboard/disarm commands to PX4. It is the only publisher of the controllers' references.
class MissionExecutorNode : public rclcpp::Node
{
public:
    explicit MissionExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void loop();
    bool gatherMeasurements(Measurements & m);
    void publishCommands(const Output & out);
    void sendPx4Commands(const mission_maneuvers::UavCommand & cmd);
    void publishVehicleCommand(uint16_t command, float param1, float param2 = 0.0f);
    std::string resolveMissionPath(const std::string & mission) const;

    std::unique_ptr<MissionExecutorPipeline> pipeline_;

    // parameters
    std::string world_frame_;
    std::string anchor_frame_;
    std::string uav_frame_;
    int px4_target_system_{2};
    double px4_command_period_{1.0};

    // inputs
    nav_msgs::msg::Odometry::SharedPtr uav_odom_;
    nav_msgs::msg::Odometry::SharedPtr usv_odom_;
    nav_msgs::msg::Path::SharedPtr usv_horizon_;
    px4_msgs::msg::VehicleStatus::SharedPtr status_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uav_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr usv_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr usv_horizon_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

    // outputs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr uav_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr usv_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tether_pull_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yaw_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_px4_command_{0, 0, RCL_ROS_TIME};
};

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__MISSION_EXECUTOR_NODE_HPP_
