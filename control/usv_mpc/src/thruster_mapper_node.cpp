#include <memory>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace usv_mpc {

class ThrusterMapperNode : public rclcpp::Node
{
public:
  ThrusterMapperNode() 
    : Node("thruster_mapper_node"),
      current_left_angle_(0.0), current_right_angle_(0.0),
      target_left_angle_(0.0), target_right_angle_(0.0),
      target_left_thrust_(0.0), target_right_thrust_(0.0)
  {
    // Declare parameters with WAM-V defaults (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<double>("thruster_x_arm", -2.373776);
    this->declare_parameter<double>("thruster_y_arm", 1.027135);
    this->declare_parameter<double>("reverse_efficiency", 0.746);
    this->declare_parameter<double>("max_thrust", 2350.0);
    this->declare_parameter<double>("max_angle", 1.57079632679); // 90 degrees
    this->declare_parameter<double>("slew_rate_limit", 3.0); // rad/s
    this->declare_parameter<double>("publish_period", 0.02); // 50Hz (20ms)

    // Retrieve parameters
    lx_ = this->get_parameter("thruster_x_arm").as_double();
    ly_ = this->get_parameter("thruster_y_arm").as_double();
    eta_rev_ = this->get_parameter("reverse_efficiency").as_double();
    max_thrust_ = this->get_parameter("max_thrust").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
    slew_rate_limit_ = this->get_parameter("slew_rate_limit").as_double();
    double dt = this->get_parameter("publish_period").as_double();

    // Publishers for WAM-V thruster commands (Rule 4 of CODE_STANDARDS.md - relative)
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/left/thrust", 10);
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/right/thrust", 10);
    left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/left/pos", 10);
    right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("thrusters/right/pos", 10);

    // Subscriber to virtual wrench (Rule 4 of CODE_STANDARDS.md - relative)
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "cmd_wrench", 10, std::bind(&ThrusterMapperNode::wrenchCallback, this, std::placeholders::_1));

    // Timer at 50Hz for smooth execution and slew rate limiting
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&ThrusterMapperNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "USV Thruster Mapper Node inicializado. (lx=%.3f, ly=%.3f, eta_rev=%.3f)",
                lx_, ly_, eta_rev_);
  }

private:
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    double X = msg->wrench.force.x;
    double Y = msg->wrench.force.y;
    double N = msg->wrench.torque.z;

    // Inverse Thrust Allocation equations:
    // f_y_L = 0.5 * Y,  f_y_R = 0.5 * Y
    // f_x_L = 0.5 * X + (lx / (2*ly)) * Y - (1 / (2*ly)) * N
    // f_x_R = 0.5 * X - (lx / (2*ly)) * Y + (1 / (2*ly)) * N
    
    double f_y_L = 0.5 * Y;
    double f_y_R = 0.5 * Y;

    double f_x_L = 0.5 * X + (lx_ / (2.0 * ly_)) * Y - (1.0 / (2.0 * ly_)) * N;
    double f_x_R = 0.5 * X - (lx_ / (2.0 * ly_)) * Y + (1.0 / (2.0 * ly_)) * N;

    // Convert Cartesian forces to Polar (thrust force and steering angle)
    double T_L_raw = std::sqrt(f_x_L * f_x_L + f_y_L * f_y_L);
    double T_R_raw = std::sqrt(f_x_R * f_x_R + f_y_R * f_y_R);

    // Determine thrust direction and apply reverse efficiency compensation if negative
    if (f_x_L < 0.0) {
      target_left_thrust_ = -T_L_raw / eta_rev_;
      target_left_angle_ = -std::atan2(f_y_L, std::abs(f_x_L));
    } else {
      target_left_thrust_ = T_L_raw;
      target_left_angle_ = std::atan2(f_y_L, f_x_L);
    }

    if (f_x_R < 0.0) {
      target_right_thrust_ = -T_R_raw / eta_rev_;
      target_right_angle_ = -std::atan2(f_y_R, std::abs(f_x_R));
    } else {
      target_right_thrust_ = T_R_raw;
      target_right_angle_ = std::atan2(f_y_R, f_x_R);
    }

    // Clamp thrust targets to physical maximums
    target_left_thrust_ = std::clamp(target_left_thrust_, -max_thrust_, max_thrust_);
    target_right_thrust_ = std::clamp(target_right_thrust_, -max_thrust_, max_thrust_);

    // Clamp steering angle to mechanical limits
    target_left_angle_ = std::clamp(target_left_angle_, -max_angle_, max_angle_);
    target_right_angle_ = std::clamp(target_right_angle_, -max_angle_, max_angle_);
  }

  void timerCallback()
  {
    double dt = this->get_parameter("publish_period").as_double();
    double max_angle_step = slew_rate_limit_ * dt;

    // Apply slew rate limits to steering
    double left_angle_diff = target_left_angle_ - current_left_angle_;
    if (left_angle_diff > max_angle_step) {
        current_left_angle_ += max_angle_step;
    } else if (left_angle_diff < -max_angle_step) {
        current_left_angle_ -= max_angle_step;
    } else {
        current_left_angle_ = target_left_angle_;
    }

    double right_angle_diff = target_right_angle_ - current_right_angle_;
    if (right_angle_diff > max_angle_step) {
        current_right_angle_ += max_angle_step;
    } else if (right_angle_diff < -max_angle_step) {
        current_right_angle_ -= max_angle_step;
    } else {
        current_right_angle_ = target_right_angle_;
    }

    // Publish std_msgs::msg::Float64 commands
    auto msg_t_L = std_msgs::msg::Float64();
    msg_t_L.data = target_left_thrust_;

    auto msg_t_R = std_msgs::msg::Float64();
    msg_t_R.data = target_right_thrust_;

    auto msg_a_L = std_msgs::msg::Float64();
    msg_a_L.data = current_left_angle_;

    auto msg_a_R = std_msgs::msg::Float64();
    msg_a_R.data = current_right_angle_;

    left_thrust_pub_->publish(msg_t_L);
    right_thrust_pub_->publish(msg_t_R);
    left_pos_pub_->publish(msg_a_L);
    right_pos_pub_->publish(msg_a_R);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double lx_;
  double ly_;
  double eta_rev_;
  double max_thrust_;
  double max_angle_;
  double slew_rate_limit_;

  double current_left_angle_;
  double current_right_angle_;
  double target_left_angle_;
  double target_right_angle_;
  double target_left_thrust_;
  double target_right_thrust_;
};

} // namespace usv_mpc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_mpc::ThrusterMapperNode>());
  rclcpp::shutdown();
  return 0;
}
