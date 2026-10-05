#include <memory>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"

class USVController : public rclcpp::Node
{
public:
  // Physical parameters and limits from Thesis (Section 4.1 - Simulation Environment)
  // Representative of Torqeedo Cruise 2.0 electric outboard motors
  static constexpr double MAX_THRUST_FWD = 510.0;                       // Maximum forward thrust per motor (N)
  static constexpr double REV_EFFICIENCY = 0.746;                       // Reverse propeller efficiency factor
  static constexpr double MAX_THRUST_REV = -380.0;                      // Maximum reverse thrust per motor (N)
  static constexpr double MAX_STEER_ANGLE = 30.0 * M_PI / 180.0;        // Maximum azimuth steering angle (+/- 30 deg, ~0.5236 rad)
  static constexpr double MAX_STEER_RATE = 10.0 * M_PI / 180.0;         // Maximum steering rate of change (10 deg/s, ~0.1745 rad/s)
  static constexpr double CONTROL_DT = 0.02;                            // Control loop period at 50Hz (s)

  USVController() : Node("usv_controller"), current_left_angle_(0.0), current_right_angle_(0.0),
                    target_left_angle_(0.0), target_right_angle_(0.0),
                    target_left_thrust_(0.0), target_right_thrust_(0.0)
  {
    // Publishers for left and right WAM-V motor thrust forces and joint positions
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/left/thrust", 10);
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/right/thrust", 10);
    
    left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/left/pos", 10);
    right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/right/pos", 10);

    // Subscriber for gamepad input (/joy)
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&USVController::joy_callback, this, std::placeholders::_1));

    // Timer at 50Hz (20ms) to publish commands smoothly and enforce slew rate limits
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&USVController::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "USV Joystick Controller initialized with Thesis limits: Fwd=%.1fN, Rev=%.1fN, Steer=+/-%.1f deg, Rate=%.1f deg/s",
                MAX_THRUST_FWD, MAX_THRUST_REV, 30.0, 10.0);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ensure sufficient axes and buttons exist
    if (msg->axes.size() < 6 || msg->buttons.size() < 6) return;

    // Button Mapping (Typical PS/Xbox layout in ROS joy)
    bool crab_mode = msg->buttons[4] == 1;     // L1 activates Crab Walk Mode

    // Read triggers: L2 and R2 (axes 2 and 5) report 1.0 (unpressed) to -1.0 (fully pressed)
    double l2_val = (1.0 - msg->axes[2]) / 2.0; // Reverse throttle [0.0, 1.0]
    double r2_val = (1.0 - msg->axes[5]) / 2.0; // Forward throttle [0.0, 1.0]
    double linear_cmd = r2_val - l2_val;        // Surge command [-1.0, 1.0]

    // Read Left Stick horizontal (axis 0): Left = 1.0, Right = -1.0
    double lateral_cmd = msg->axes[0]; // Sway or Yaw command

    if (crab_mode) {
      // =======================================================================
      // MODE 1: CRAB WALK (Omnidirectional translation, zero yaw moment)
      // =======================================================================
      double target_Y = lateral_cmd * MAX_THRUST_FWD * 0.5;
      double target_X = (linear_cmd >= 0.0) ? (linear_cmd * MAX_THRUST_FWD) : (linear_cmd * std::abs(MAX_THRUST_REV));

      // Thrust Allocation equations for zero yaw moment:
      double f_y_L = 0.5 * target_Y;
      double f_y_R = 0.5 * target_Y;
      double f_x_L = -1.15553 * target_Y + 0.5 * target_X;
      double f_x_R =  1.15553 * target_Y + 0.5 * target_X;

      // Convert Cartesian forces to Polar (thrust force and steering angle)
      target_left_thrust_ = std::sqrt(f_x_L * f_x_L + f_y_L * f_y_L);
      target_right_thrust_ = std::sqrt(f_x_R * f_x_R + f_y_R * f_y_R);

      // Preserve thrust direction based on surge component sign
      if (f_x_L < 0.0) target_left_thrust_ = -target_left_thrust_;
      if (f_x_R < 0.0) target_right_thrust_ = -target_right_thrust_;

      target_left_angle_ = std::atan2(f_y_L, std::abs(f_x_L));
      target_right_angle_ = std::atan2(f_y_R, std::abs(f_x_R));

      // Correct angle signs when thrust is reversed to keep thrust vector correct
      if (f_x_L < 0.0) target_left_angle_ = -target_left_angle_;
      if (f_x_R < 0.0) target_right_angle_ = -target_right_angle_;

    } else {
      // =======================================================================
      // MODE 2: NORMAL DRIVING & ZERO-RADIUS TURNING
      // =======================================================================
      if (std::abs(linear_cmd) < 0.05) {
        // CASE 2A: No linear velocity -> Zero-radius turn (Spin on the spot)
        // Engines stay straight (0 rad), differential thrust creates pure yaw moment (N)
        // Magnitude limited by reverse thrust capability to maintain pure moment without net surge
        double diff_thrust = lateral_cmd * std::abs(MAX_THRUST_REV) * 0.5;
        target_left_thrust_ = -diff_thrust; // Turn right (lateral_cmd < 0): Left fwd (+), Right rev (-)
        target_right_thrust_ = diff_thrust;
        
        target_left_angle_ = 0.0;
        target_right_angle_ = 0.0;
      } else {
        // CASE 2B: Coordinated turns (engines rotate together within +/- 30 deg)
        double thrust_val = (linear_cmd >= 0.0) ? (linear_cmd * MAX_THRUST_FWD) : (linear_cmd * std::abs(MAX_THRUST_REV));
        target_left_thrust_ = thrust_val;
        target_right_thrust_ = thrust_val;

        // Steering angle up to 30 degrees (0.5236 rad)
        target_left_angle_ = -lateral_cmd * MAX_STEER_ANGLE;
        target_right_angle_ = -lateral_cmd * MAX_STEER_ANGLE;
      }
    }

    // Clamp thrust targets to physical maximums: [-380.0 N, +510.0 N]
    target_left_thrust_ = std::clamp(target_left_thrust_, MAX_THRUST_REV, MAX_THRUST_FWD);
    target_right_thrust_ = std::clamp(target_right_thrust_, MAX_THRUST_REV, MAX_THRUST_FWD);

    // Clamp steering angle to mechanical limits: [-30 deg, +30 deg]
    target_left_angle_ = std::clamp(target_left_angle_, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    target_right_angle_ = std::clamp(target_right_angle_, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  }

  void timer_callback()
  {
    // Steering rotation rate limit (Slew Rate): 10.0 deg/s (0.1745329 rad/s)
    // At 50Hz (dt = 0.02s), max steering change per frame is 0.1745329 * 0.02 = 0.00349066 rad
    double max_angle_step = MAX_STEER_RATE * CONTROL_DT;

    // Apply slew rate limits to left motor steering
    double left_angle_diff = target_left_angle_ - current_left_angle_;
    if (left_angle_diff > max_angle_step) {
        current_left_angle_ += max_angle_step;
    } else if (left_angle_diff < -max_angle_step) {
        current_left_angle_ -= max_angle_step;
    } else {
        current_left_angle_ = target_left_angle_;
    }

    // Apply slew rate limits to right motor steering
    double right_angle_diff = target_right_angle_ - current_right_angle_;
    if (right_angle_diff > max_angle_step) {
        current_right_angle_ += max_angle_step;
    } else if (right_angle_diff < -max_angle_step) {
        current_right_angle_ -= max_angle_step;
    } else {
        current_right_angle_ = target_right_angle_;
    }

    auto left_thrust_msg = std_msgs::msg::Float64();
    left_thrust_msg.data = target_left_thrust_;

    auto right_thrust_msg = std_msgs::msg::Float64();
    right_thrust_msg.data = target_right_thrust_;

    auto left_pos_msg = std_msgs::msg::Float64();
    pos_msg_generator(left_pos_msg, current_left_angle_);

    auto right_pos_msg = std_msgs::msg::Float64();
    pos_msg_generator(right_pos_msg, current_right_angle_);

    // Publish commands
    left_thrust_pub_->publish(left_thrust_msg);
    right_thrust_pub_->publish(right_thrust_msg);
    
    left_pos_pub_->publish(left_pos_msg);
    right_pos_pub_->publish(right_pos_msg);
  }

  void pos_msg_generator(std_msgs::msg::Float64& msg, double angle) {
      msg.data = angle;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double current_left_angle_;
  double current_right_angle_;
  double target_left_angle_;
  double target_right_angle_;
  double target_left_thrust_;
  double target_right_thrust_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVController>());
  rclcpp::shutdown();
  return 0;
}
