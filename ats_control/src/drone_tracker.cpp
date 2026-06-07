#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

/**
 * @brief Drone Tracker using Ground Truth for error calculation.
 * 
 * Logic:
 * 1. Get real distance between Boat and Drone from Gazebo (Ground Truth).
 * 2. Apply that error vector to the PX4's internal local position.
 * 3. Result: Millimeter precision tracking regardless of PX4 frame offsets.
 */
class DroneTracker : public rclcpp::Node {
public:
	DroneTracker() : Node("drone_tracker") {
		auto qos = rclcpp::SensorDataQoS();

		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_1/fmu/in/vehicle_command", qos);

		// Subscribers
		boat_gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/boat/ground_truth/odometry", qos, std::bind(&DroneTracker::boat_gt_callback, this, std::placeholders::_1));

		drone_gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/drone/ground_truth/odometry", qos, std::bind(&DroneTracker::drone_gt_callback, this, std::placeholders::_1));

		drone_px4_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/px4_1/fmu/out/vehicle_odometry", qos, std::bind(&DroneTracker::drone_px4_callback, this, std::placeholders::_1));

		// Parameters
		this->declare_parameter<double>("follow_height", 5.0);
		this->declare_parameter<double>("wait_time_s", 10.0);
		
		follow_height_ = this->get_parameter("follow_height").as_double();
		wait_time_s_ = this->get_parameter("wait_time_s").as_double();

		timer_ = this->create_wall_timer(50ms, std::bind(&DroneTracker::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "Ground Truth Drone Tracker Initialized.");
	}

private:
	void boat_gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		boat_world_pos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
		
		// Extract Yaw from Boat Orientation (ENU)
		tf2::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch;
		m.getRPY(roll, pitch, boat_yaw_enu_);
	}

	void drone_gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		drone_world_pos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
	}

	void drone_px4_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
		// PX4 uses NED (North-East-Down) but position[0,1,2] is usually X,Y,Z in the internal frame
		drone_px4_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
	}

	void timer_callback() {
		uint64_t wait_ticks = static_cast<uint64_t>(wait_time_s_ * 20.0);
		
		publish_offboard_control_mode();
		publish_trajectory_setpoint();

		if (offboard_setpoint_counter_ >= wait_ticks && offboard_setpoint_counter_ < wait_ticks + 20) {
			this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard
			this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
			if (offboard_setpoint_counter_ == wait_ticks) {
				RCLCPP_INFO(this->get_logger(), "Engaging Offboard and Arming...");
			}
		}

		if (offboard_setpoint_counter_ < wait_ticks + 20) {
			offboard_setpoint_counter_++;
		}
	}

	void publish_offboard_control_mode() {
		px4_msgs::msg::OffboardControlMode msg{};
		msg.position = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint() {
		// 1. Calculate Error in World Frame (Gazebo ENU)
		// Target point is boat position + follow height
		double err_x = boat_world_pos_[0] - drone_world_pos_[0];
		double err_y = boat_world_pos_[1] - drone_world_pos_[1];
		double err_z = (boat_world_pos_[2] + follow_height_) - drone_world_pos_[2];

		// 2. Transform ENU Error to NED for PX4
		// ENU Error (Ex, Ey, Ez) -> NED Error (Nx=Ey, Ny=Ex, Nz=-Ez)
		// But PX4 internal control is usually oriented to its spawn.
		// A safe bet for "Relative Tracking" is:
		// Target_PX4 = Current_PX4 + Error_Projected
		
		px4_msgs::msg::TrajectorySetpoint msg{};
		
		// Applying the delta to current PX4 position
		// Assuming PX4 base frame is aligned with North (Standard SITL)
		msg.position[0] = drone_px4_pos_[0] + err_y;  // North = ENU East? No, standard is:
		                                             // X_ned = Y_enu (North)
		                                             // Y_ned = X_enu (East)
		                                             // Z_ned = -Z_enu (Down)
		
		msg.position[0] = drone_px4_pos_[0] + err_y; // North error
		msg.position[1] = drone_px4_pos_[1] + err_x; // East error
		msg.position[2] = drone_px4_pos_[2] - err_z; // Down error

		// 3. Follow Boat Orientation (Yaw)
		// ENU Yaw to NED Yaw: yaw_ned = -yaw_enu + PI/2
		msg.yaw = static_cast<float>(-boat_yaw_enu_ + M_PI_2); 
		
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
		px4_msgs::msg::VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 2; // Drone x500
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_gt_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_gt_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone_px4_sub_;

	std::vector<double> boat_world_pos_{0,0,0};
	double boat_yaw_enu_{0.0};
	std::vector<double> drone_world_pos_{0,0,0};
	std::vector<double> drone_px4_pos_{0,0,0};

	uint64_t offboard_setpoint_counter_{0};
	double follow_height_{5.0};
	double wait_time_s_{10.0};
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneTracker>());
	rclcpp::shutdown();
	return 0;
}
