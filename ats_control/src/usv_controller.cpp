#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class USVController : public rclcpp::Node
{
public:
  USVController() : Node("usv_controller")
  {
    // Publishers para os motores esquerdo e direito do WAM-V
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);

    // Subscriber para comandos de velocidade (cmd_vel)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&USVController::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "USV Controller iniciado! A aguardar comandos de movimento em /cmd_vel...");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto left_msg = std_msgs::msg::Float64();
    auto right_msg = std_msgs::msg::Float64();

    // Cinematica diferencial basica (Skid Steer)
    // O thrust maximo configurado é de 250 Newtons.
    double max_thrust = 250.0; 

    double linear = msg->linear.x;   // Frente / Tras
    double angular = msg->angular.z; // Rotacao (Esquerda / Direita)

    // Calculo do thrust para cada motor
    double left_thrust = (linear - angular) * max_thrust;
    double right_thrust = (linear + angular) * max_thrust;

    // Limitar os valores de thrust entre -max_thrust e max_thrust
    left_thrust = std::clamp(left_thrust, -max_thrust, max_thrust);
    right_thrust = std::clamp(right_thrust, -max_thrust, max_thrust);

    left_msg.data = left_thrust;
    right_msg.data = right_thrust;

    left_thrust_pub_->publish(left_msg);
    right_thrust_pub_->publish(right_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVController>());
  rclcpp::shutdown();
  return 0;
}
