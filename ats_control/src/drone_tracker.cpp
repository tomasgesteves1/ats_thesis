#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class DroneTracker : public rclcpp::Node {
public:
	DroneTracker() : Node("drone_tracker", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {
		// QoS settings for PX4: usually needs RELIABLE or BEST_EFFORT depending on the topic. 
        // Best effort is safer for sensor data, but for offboard control, default (reliable) or sensor_data is used.
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Publishers (PX4 is running as instance 1, so topics are prefixed with /px4_1)
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_1/fmu/in/vehicle_command", qos);

		// Subscriber
		boat_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/boat/ground_truth/odometry", 10, std::bind(&DroneTracker::boat_odom_callback, this, std::placeholders::_1));

		this->declare_parameter<int>("target_system", 2);
		this->declare_parameter<double>("follow_height", 5.0);
		this->declare_parameter<double>("wait_time_s", 5.0);

		target_system_ = this->get_parameter("target_system").as_int();
		follow_height_ = this->get_parameter("follow_height").as_double();
		wait_time_s_ = this->get_parameter("wait_time_s").as_double();

		// Timer at 20 Hz (Offboard mode requires at least 2Hz, 20Hz is recommended)
		timer_ = this->create_wall_timer(50ms, std::bind(&DroneTracker::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "Drone Tracker Node started. Waiting %f seconds before taking off...", wait_time_s_);
	}

private:
	void boat_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		boat_x_ = msg->pose.pose.position.x;
		boat_y_ = msg->pose.pose.position.y;
		boat_z_ = msg->pose.pose.position.z;
	}

	void timer_callback() {
		uint64_t wait_ticks = static_cast<uint64_t>(wait_time_s_ * 20.0);
		
		// Durante o período de espera e após, enviamos sempre setpoints
		publish_offboard_control_mode();
		publish_trajectory_setpoint();

		// Aos 5 segundos (ou wait_time), tentamos Arm e Offboard por 1 segundo (20 ticks)
		if (offboard_setpoint_counter_ >= wait_ticks && offboard_setpoint_counter_ < wait_ticks + 20) {
			this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			this->arm();
			if (offboard_setpoint_counter_ == wait_ticks) {
				RCLCPP_INFO(this->get_logger(), "Sending Arm and Offboard commands...");
			}
		}

		if (offboard_setpoint_counter_ < wait_ticks + 20) {
			offboard_setpoint_counter_++;
		}
	}

	void arm() {
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	}

	void publish_offboard_control_mode() {
		px4_msgs::msg::OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint() {
		px4_msgs::msg::TrajectorySetpoint msg{};
		// NED frame: Z is down. So -follow_height_ means follow_height_ meters above the boat.
		// Note: The boat Odometry is in ENU. Gazebo world is ENU. PX4 expects NED in setpoints.
		// ENU to NED: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
		msg.position = { (float)boat_y_, (float)boat_x_, -(float)follow_height_ };
		msg.velocity = {NAN, NAN, NAN};
		msg.acceleration = {NAN, NAN, NAN};
		msg.jerk = {NAN, NAN, NAN};
		msg.yaw = 0.0f; // Face north
		msg.yawspeed = NAN;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
		px4_msgs::msg::VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = target_system_; // Use ROS parameter
		msg.target_component = 1;
		msg.source_system = 255; // Offboard computer
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_odom_sub_;

	uint64_t offboard_setpoint_counter_{0};
	double boat_x_{0.0};
	double boat_y_{0.0};
	double boat_z_{0.0};

	int target_system_{2};
	double follow_height_{5.0};
	double wait_time_s_{5.0};
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneTracker>());
	rclcpp::shutdown();
	return 0;
}
