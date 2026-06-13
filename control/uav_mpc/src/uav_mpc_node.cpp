#include "uav_mpc/uav_mpc_node.hpp"

namespace uav_mpc {

UavMpcNode::UavMpcNode() 
    : Node("uav_mpc_node"), 
      target_initialized_(false), 
      offboard_setpoint_counter_(0) {
      
    pipeline_ = std::make_unique<UavMpcPipeline>();

    // Namespace relativo (Regra 4 do CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UavMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UavMpcNode::targetCallback, this, std::placeholders::_1));

    tether_length_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "tether_length", 10, std::bind(&UavMpcNode::tetherLengthCallback, this, std::placeholders::_1));

    // Inicialização do buffer e listener do TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Configuração do QoS idêntico ao drone_tracker para PX4 (SensorDataQoS)
    auto qos = rclcpp::SensorDataQoS();
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", qos);
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/px4_1/fmu/in/vehicle_attitude_setpoint", qos);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", qos);

    // Publicadores de Visualização (relativos)
    predicted_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "predicted_trajectory", 10);
    target_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "target_point", 10);

    // Publicador para a magnitude da força estimada pelo MPC
    mpc_tether_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "mpc_tether_force_mag", 10);

    // Timer a 20Hz (0.05s) para igualar o dt do MPC
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&UavMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "UAV MPC Node inicializado com sucesso.");
}

void UavMpcNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Nova Referencia Recebida: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    pipeline_->setReference({msg->x, msg->y, msg->z});
}

void UavMpcNode::tetherLengthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    pipeline_->updateTetherLength(msg->data);
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
    
    // Passar a orientação atual do drone para a pipeline
    pipeline_->updateOrientation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    if (!target_initialized_) {
        pipeline_->setReference({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z});
        target_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Target Inicial fixado na Odometria inicial: Z=%.2f", msg->pose.pose.position.z);
    }
    
    // Debug 1/segundo para sabermos se o odom chega
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Odom recebida -> Z=%.2f", msg->pose.pose.position.z);
}

void UavMpcNode::controlLoop() {
    // Obter a posição exata da âncora via TF (world -> boat/tether_anchor)
    try {
        auto transform = tf_buffer_->lookupTransform("world", "boat/tether_anchor", tf2::TimePointZero);
        pipeline_->updateAnchorPosition(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Não foi possível obter transformação de world para boat/tether_anchor: %s", ex.what());
    }

    // Pipeline realiza todos os cálculos e devolve a estrutura final
    UavControlOutput output = pipeline_->computeControl();

    // Sempre publicar OffboardControlMode e VehicleAttitudeSetpoint para manter o watchdog ativo
    publishOffboardControlMode();
    publishAttitudeSetpoint(output);
    
    // Publicar visualização para RViz/Foxglove
    publishVisualizationMarkers(output);

    // Publicar força estimada pelo modelo do MPC
    std_msgs::msg::Float64 force_msg;
    force_msg.data = output.mpc_tether_force_mag;
    mpc_tether_force_pub_->publish(force_msg);

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

void UavMpcNode::publishAttitudeSetpoint(const UavControlOutput& output) {
    px4_msgs::msg::VehicleAttitudeSetpoint att_msg{};
    att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    att_msg.q_d[0] = output.q_d[0];
    att_msg.q_d[1] = output.q_d[1];
    att_msg.q_d[2] = output.q_d[2];
    att_msg.q_d[3] = output.q_d[3];
    
    att_msg.thrust_body[0] = 0.0f;
    att_msg.thrust_body[1] = 0.0f;
    att_msg.thrust_body[2] = -static_cast<float>(output.thrust_normalized); // -Z em FRD é a força para cima

    attitude_setpoint_pub_->publish(att_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "PX4 Setpoint -> Thrust Norm: %.2f, q_d (w,x,y,z): [%.2f, %.2f, %.2f, %.2f]",
        output.thrust_normalized, output.q_d[0], output.q_d[1], output.q_d[2], output.q_d[3]);
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

void UavMpcNode::publishVisualizationMarkers(const UavControlOutput& output) {
    auto current_time = this->get_clock()->now();

    // 1. Marcador da Trajetória Prevista (Horizonte do MPC)
    visualization_msgs::msg::Marker line_msg;
    line_msg.header.frame_id = "world";
    line_msg.header.stamp = current_time;
    line_msg.ns = "predicted_trajectory";
    line_msg.id = 0;
    line_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_msg.action = visualization_msgs::msg::Marker::ADD;
    line_msg.pose.orientation.w = 1.0;
    
    // Configuração visual (verde brilhante semi-transparente)
    line_msg.scale.x = 0.05; // Largura da linha
    line_msg.color.r = 0.0;
    line_msg.color.g = 1.0;
    line_msg.color.b = 0.0;
    line_msg.color.a = 0.8;

    for (const auto& pos : output.predicted_positions) {
        geometry_msgs::msg::Point p;
        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        line_msg.points.push_back(p);
    }
    predicted_trajectory_pub_->publish(line_msg);

    // 2. Marcador do Setpoint/Referência Atual
    if (output.current_reference.size() == 3) {
        visualization_msgs::msg::Marker point_msg;
        point_msg.header.frame_id = "world";
        point_msg.header.stamp = current_time;
        point_msg.ns = "target_point";
        point_msg.id = 1;
        point_msg.type = visualization_msgs::msg::Marker::SPHERE;
        point_msg.action = visualization_msgs::msg::Marker::ADD;
        
        point_msg.pose.position.x = output.current_reference[0];
        point_msg.pose.position.y = output.current_reference[1];
        point_msg.pose.position.z = output.current_reference[2];
        point_msg.pose.orientation.w = 1.0;

        point_msg.scale.x = 0.25; // Diâmetro de 25 cm
        point_msg.scale.y = 0.25;
        point_msg.scale.z = 0.25;

        point_msg.color.r = 1.0; // Vermelho
        point_msg.color.g = 0.0;
        point_msg.color.b = 0.0;
        point_msg.color.a = 1.0;

        target_point_pub_->publish(point_msg);
    }
}

} // namespace uav_mpc
