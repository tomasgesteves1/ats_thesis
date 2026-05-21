#include <memory>
#include <algorithm>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"

class USVController : public rclcpp::Node
{
public:
  USVController() : Node("usv_controller"), current_angle_(0.0), target_angle_(0.0), target_thrust_(0.0)
  {
    // Publishers para a forca (thrust) e angulo (pos) dos motores esquerdo e direito do WAM-V
    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/left/thrust", 10);
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/right/thrust", 10);
    
    left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/left/pos", 10);
    right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat/thrusters/right/pos", 10);

    // Subscriber para comandos do joystick (gamepad)
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&USVController::joy_callback, this, std::placeholders::_1));

    // Timer para enviar os comandos de forma constante e suavizar a rotacao dos motores (50 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&USVController::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "USV Joystick Controller iniciado! Motores soltos (sem L1).");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ignora se o comando nao tiver eixos suficientes
    if (msg->axes.size() < 6 || msg->buttons.size() < 6) return;

    // R1 (Turbo switch) multiplica a velocidade
    bool turbo_switch = msg->buttons[5] == 1;

    double max_thrust = turbo_switch ? 5000.0 : 2500.0;
    double max_angle = 1.0; // +/- ~57 graus (em radianos)

    // No ROS `joy`, os gatilhos L2 e R2 (Axes 2 e 5) geralmente reportam 
    // 1.0 quando nao pressionados e -1.0 quando totalmente pressionados.
    // Vamos converter esse intervalo [1.0, -1.0] para [0.0, 1.0].
    double l2_val = (1.0 - msg->axes[2]) / 2.0; // Acelerador para Tras
    double r2_val = (1.0 - msg->axes[5]) / 2.0; // Acelerador para a Frente

    // Acelerador Final (A Frente tem precedencia ou combinam-se)
    double linear_cmd = r2_val - l2_val;

    // Direcao (Left Stick H: Axis 0 -> Esquerda = 1.0, Direita = -1.0)
    // Para virar o barco para a Esquerda, a traseira tem de ir para a Direita,
    // logo os motores devem apontar para a Direita (- radianos).
    // Invertemos o sinal do eixo 0 para corrigir a direcao.
    double angle_cmd = -msg->axes[0];

    target_thrust_ = linear_cmd * max_thrust;
    target_angle_ = angle_cmd * max_angle;

    // Limitar os valores
    target_thrust_ = std::clamp(target_thrust_, -max_thrust, max_thrust);
    target_angle_ = std::clamp(target_angle_, -max_angle, max_angle);
  }

  void timer_callback()
  {
    // Suavizar a rotacao dos propulsores (limitador de taxa de variacao - Slew Rate)
    // A 50Hz (0.02s), uma mudanca de 0.02 radianos por frame equivale a 1.0 rad/s
    double max_angle_step = 1.0 * 0.02; // Velocidade de rotacao maxima dos propulsores

    // Interpola a diferenca
    double angle_diff = target_angle_ - current_angle_;
    if (angle_diff > max_angle_step) {
        current_angle_ += max_angle_step;
    } else if (angle_diff < -max_angle_step) {
        current_angle_ -= max_angle_step;
    } else {
        current_angle_ = target_angle_;
    }

    auto thrust_msg = std_msgs::msg::Float64();
    thrust_msg.data = target_thrust_;

    auto pos_msg = std_msgs::msg::Float64();
    pos_msg.data = current_angle_;

    // Publicar continuamente
    left_thrust_pub_->publish(thrust_msg);
    right_thrust_pub_->publish(thrust_msg);
    
    left_pos_pub_->publish(pos_msg);
    right_pos_pub_->publish(pos_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double current_angle_;
  double target_angle_;
  double target_thrust_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVController>());
  rclcpp::shutdown();
  return 0;
}
