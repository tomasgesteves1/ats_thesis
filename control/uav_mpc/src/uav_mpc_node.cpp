#include "uav_mpc/uav_mpc_node.hpp"
#include <cmath>

namespace uav_mpc {

UavMpcNode::UavMpcNode() 
    : Node("uav_mpc_node"), 
      target_initialized_(false), 
      target_yaw_(0.0),
      offboard_setpoint_counter_(0),
      qx_(0.0), qy_(0.0), qz_(0.0), qw_(1.0) {
      
    pipeline_ = std::make_unique<UavMpcPipeline>();

    // Namespace relativo (Regra 4 do CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UavMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UavMpcNode::targetCallback, this, std::placeholders::_1));

    // Configuração do QoS idêntico ao drone_tracker para PX4 (SensorDataQoS)
    auto qos = rclcpp::SensorDataQoS();
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", qos);
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/px4_1/fmu/in/vehicle_attitude_setpoint", qos);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", qos);

    // Timer a 20Hz (0.05s) para igualar o dt do MPC
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&UavMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "UAV MPC Node inicializado com sucesso.");
}

void UavMpcNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Nova Referencia Recebida: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    pipeline_->setReference({msg->x, msg->y, msg->z});
}

void UavMpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Zero matemática. Apenas passa dados. (Regra 1 do CODE_STANDARDS.md)
    std::vector<double> state = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    };
    pipeline_->updateState(state);
    
    // Guardar orientação (referencial do mundo ENU)
    qx_ = msg->pose.pose.orientation.x;
    qy_ = msg->pose.pose.orientation.y;
    qz_ = msg->pose.pose.orientation.z;
    qw_ = msg->pose.pose.orientation.w;

    if (!target_initialized_) {
        pipeline_->setReference({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z});
        
        // Guardar yaw inicial como o target de yaw no referencial do mundo (ENU)
        double roll, pitch;
        quaternionToEuler(qx_, qy_, qz_, qw_, roll, pitch, target_yaw_);
        
        target_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Target Inicial fixado na Odometria inicial: Z=%.2f, Yaw Target: %.2f rad", 
            msg->pose.pose.position.z, target_yaw_);
    }
    
    // Debug 1/segundo para sabermos se o odom chega
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Odom recebida -> Z=%.2f", msg->pose.pose.position.z);
}

void UavMpcNode::controlLoop() {
    std::vector<double> control_cmd = pipeline_->computeControl();

    // Sempre publicar OffboardControlMode e VehicleAttitudeSetpoint para manter o watchdog ativo
    publishOffboardControlMode();
    publishAttitudeSetpoint(control_cmd);

    // Enviar comandos de armar e mudar de modo após ~2 segundos (40 iterações a 20Hz)
    if (offboard_setpoint_counter_ >= 40 && offboard_setpoint_counter_ < 60) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 1 = offboard, 6 = offboard submode
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // 1.0 = arm
        
        if (offboard_setpoint_counter_ == 40) {
            RCLCPP_INFO(this->get_logger(), "A ativar Modo Offboard e Armar Drone no PX4...");
        }
    }

    if (offboard_setpoint_counter_ < 60) {
        offboard_setpoint_counter_++;
    }
}

void UavMpcNode::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
}

void UavMpcNode::publishAttitudeSetpoint(const std::vector<double>& control_cmd) {
    if (control_cmd.size() != 3) return;

    // A aceleração desejada calculada pelo MPC é no referencial do mundo (ENU)
    // control_cmd = [u_x, u_y, u_z] em m/s^2
    // Massa do drone = 2.06kg. Força desejada no referencial do mundo (ENU):
    double m = 2.06;
    double F_enu[3] = { control_cmd[0] * m, control_cmd[1] * m, control_cmd[2] * m };

    double thrust_mag = std::sqrt(F_enu[0]*F_enu[0] + F_enu[1]*F_enu[1] + F_enu[2]*F_enu[2]);
    if (thrust_mag < 0.1) {
        thrust_mag = 0.1;
    }

    // 1. Converter vetor de força desejado de ENU para o referencial de mundo do PX4 (NED)
    // ENU (Ex, Ey, Ez) -> NED (Nx = Ey, Ny = Ex, Nz = -Ez)
    double F_ned[3] = { F_enu[1], F_enu[0], -F_enu[2] };

    // 2. Extrair o eixo Z desejado do corpo (Z_body) no referencial NED.
    // Em FRD, a força de impulso aponta para cima (eixo -Z do corpo), portanto:
    // F_ned = R_d * [0, 0, -T] = -T * z_body_FRD  =>  z_body_FRD = -F_ned / T
    double z_body[3] = { -F_ned[0] / thrust_mag, -F_ned[1] / thrust_mag, -F_ned[2] / thrust_mag };

    // 3. Obter o vetor de rumo (heading/yaw) desejado em NED
    // yaw_ned = -yaw_enu + PI/2
    double yaw_ned = -target_yaw_ + M_PI_2;
    double x_yaw[3] = { std::cos(yaw_ned), std::sin(yaw_ned), 0.0 };

    // 4. Calcular o eixo Y do corpo (Y_body = Z_body x X_yaw)
    double y_body[3];
    y_body[0] = -z_body[2] * x_yaw[1];
    y_body[1] = z_body[2] * x_yaw[0];
    y_body[2] = z_body[0] * x_yaw[1] - z_body[1] * x_yaw[0];

    double y_norm = std::sqrt(y_body[0]*y_body[0] + y_body[1]*y_body[1] + y_body[2]*y_body[2]);
    if (y_norm < 1e-6) {
        y_body[0] = -std::sin(yaw_ned);
        y_body[1] = std::cos(yaw_ned);
        y_body[2] = 0.0;
    } else {
        y_body[0] /= y_norm;
        y_body[1] /= y_norm;
        y_body[2] /= y_norm;
    }

    // 5. Calcular o eixo X do corpo (X_body = Y_body x Z_body)
    double x_body[3];
    x_body[0] = y_body[1] * z_body[2] - y_body[2] * z_body[1];
    x_body[1] = y_body[2] * z_body[0] - y_body[0] * z_body[2];
    x_body[2] = y_body[0] * z_body[1] - y_body[1] * z_body[0];

    // 6. Formar a Matriz de Rotação Desejada R_d = [x_body, y_body, z_body]
    double R[3][3];
    R[0][0] = x_body[0]; R[0][1] = y_body[0]; R[0][2] = z_body[0];
    R[1][0] = x_body[1]; R[1][1] = y_body[1]; R[1][2] = z_body[1];
    R[2][0] = x_body[2]; R[2][1] = y_body[2]; R[2][2] = z_body[2];

    // 7. Converter R_d para quaternion q_d (ordem Hamiltoniana [w, x, y, z] para PX4)
    float q_d[4];
    rotationMatrixToQuaternion(R, q_d);

    // 8. Normalizar o thrust (força de empuxo)
    // Para o drone x500 no Gazebo, a força de hover é ~20.21 N, o que equivale a ~0.52 throttle.
    double hover_thrust = 2.06 * 9.81;
    double hover_throttle = 0.52;
    double thrust_normalized = (thrust_mag / hover_thrust) * hover_throttle;
    thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

    // 9. Publicar o setpoint de atitude
    px4_msgs::msg::VehicleAttitudeSetpoint att_msg{};
    att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    att_msg.q_d[0] = q_d[0];
    att_msg.q_d[1] = q_d[1];
    att_msg.q_d[2] = q_d[2];
    att_msg.q_d[3] = q_d[3];
    
    att_msg.thrust_body[0] = 0.0f;
    att_msg.thrust_body[1] = 0.0f;
    att_msg.thrust_body[2] = -static_cast<float>(thrust_normalized); // -Z em FRD é a força para cima

    attitude_setpoint_pub_->publish(att_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "PX4 Setpoint -> F_enu: [%.2f, %.2f, %.2f] N, Thrust Norm: %.2f, q_d (w,x,y,z): [%.2f, %.2f, %.2f, %.2f]",
        F_enu[0], F_enu[1], F_enu[2], thrust_normalized, q_d[0], q_d[1], q_d[2], q_d[3]);
}

void UavMpcNode::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 2; // Drone x500
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
}

void UavMpcNode::quaternionToEuler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) {
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void UavMpcNode::rotationMatrixToQuaternion(double R[3][3], float q[4]) {
    double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        double s = 2.0 * std::sqrt(tr + 1.0);
        q[0] = 0.25 * s;                // w
        q[1] = (R[2][1] - R[1][2]) / s; // x
        q[2] = (R[0][2] - R[2][0]) / s; // y
        q[3] = (R[1][0] - R[0][1]) / s; // z
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q[0] = (R[2][1] - R[1][2]) / s; // w
        q[1] = 0.25 * s;                // x
        q[2] = (R[0][1] + R[1][0]) / s; // y
        q[3] = (R[0][2] + R[2][0]) / s; // z
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q[0] = (R[0][2] - R[2][0]) / s; // w
        q[1] = (R[0][1] + R[1][0]) / s; // x
        q[2] = 0.25 * s;                // y
        q[3] = (R[1][2] + R[2][1]) / s; // z
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q[0] = (R[1][0] - R[0][1]) / s; // w
        q[1] = (R[0][2] + R[2][0]) / s; // x
        q[2] = (R[1][2] + R[2][1]) / s; // y
        q[3] = 0.25 * s;                // z
    }
}

} // namespace uav_mpc
