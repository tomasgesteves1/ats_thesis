#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ats_tether/TetherMoorDynSystem.hpp"

// Gazebo headers for direct communication
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>

using namespace std::placeholders;

class AtsTetherNode : public rclcpp::Node
{
public:
    AtsTetherNode() : Node("ats_tether_node"), initialized_(false)
    {
        // 1. Obter o caminho do ficheiro lines.txt
        std::string pkg_share = ament_index_cpp::get_package_share_directory("ats_tether");

        // 2. Inicializar o nosso motor de física MoorDyn
        tether_physics_ = std::make_unique<ats_tether::TetherMoorDynSystem>(pkg_share, this->get_logger());

        // 3. Preparar vetores de posição (X, Y, Z para Boat e Drone)
        // [0,1,2] -> Boat, [3,4,5] -> Drone
        current_positions_.assign(6, 0.0);

        // 4. Subscrever as poses globais do Gazebo (via ROS bridge)
        boat_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/boat/ground_truth/odometry", 10, std::bind(&AtsTetherNode::boat_callback, this, _1));

        drone_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/drone/ground_truth/odometry", 10, std::bind(&AtsTetherNode::drone_callback, this, _1));

        // 5. Publicador Gazebo nativo para forças
        gz_wrench_pub_ = gz_node_.Advertise<gz::msgs::EntityWrench>("/world/wamv_world/wrench");

        // 6. Timer para correr a física a 50Hz (20ms)
        physics_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AtsTetherNode::physics_loop, this));

        RCLCPP_INFO(this->get_logger(), "Nó AtsTether inicializado. Usando publicador Gazebo nativo para evitar bugs do ROS Jazzy FastCDR.");
    }

private:
    void boat_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Simplificado: Usar posição global do barco + offset Z de 1.3m (âncora)
        current_positions_[0] = msg->pose.pose.position.x;
        current_positions_[1] = msg->pose.pose.position.y;
        current_positions_[2] = msg->pose.pose.position.z + 1.3;

        check_initialization();
    }

    void drone_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Simplificado: Usar posição global do drone - offset Z de 0.2m (gancho)
        current_positions_[3] = msg->pose.pose.position.x;
        current_positions_[4] = msg->pose.pose.position.y;
        current_positions_[5] = msg->pose.pose.position.z - 0.2;

        check_initialization();
    }

    void check_initialization()
    {
        if (initialized_) return;

        // Limitar as tentativas de inicialização para 1 por segundo
        auto now = this->get_clock()->now();
        if ((now - last_init_attempt_).seconds() < 1.0) return;
        last_init_attempt_ = now;

        bool boat_ready = std::abs(current_positions_[0]) > 0.001 || std::abs(current_positions_[1]) > 0.001 || current_positions_[2] > 0.1;
        bool drone_ready = std::abs(current_positions_[3]) > 0.001 || std::abs(current_positions_[4]) > 0.001 || current_positions_[5] > 0.1;

        if (boat_ready && drone_ready)
        {
            if (tether_physics_->initialize(current_positions_))
            {
                initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "MoorDyn inicializado com SUCESSO!");
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "MoorDyn_Init FALHOU. Verifique se o numero de pontos Coupled no lines.txt coincide (devem ser 2).");
            }
        }
    }

    void physics_loop()
    {
        if (!initialized_) return;

        std::vector<double> out_forces;
        if (tether_physics_->step(current_positions_, 0.02, out_forces)) {
            // Aplicar força no Barco
            publish_wrench(boat_link_name_, out_forces[0], out_forces[1], out_forces[2]);

            // Aplicar força no Drone
            publish_wrench(drone_link_name_, out_forces[3], out_forces[4], out_forces[5]);
        }
    }

    void publish_wrench(const std::string& entity_name, double fx, double fy, double fz)
    {
        gz::msgs::EntityWrench msg;
        msg.mutable_entity()->set_name(entity_name);
        msg.mutable_entity()->set_type(gz::msgs::Entity::LINK);
        
        msg.mutable_wrench()->mutable_force()->set_x(fx);
        msg.mutable_wrench()->mutable_force()->set_y(fy);
        msg.mutable_wrench()->mutable_force()->set_z(fz);
        
        msg.mutable_wrench()->mutable_torque()->set_x(0.0);
        msg.mutable_wrench()->mutable_torque()->set_y(0.0);
        msg.mutable_wrench()->mutable_torque()->set_z(0.0);

        gz_wrench_pub_.Publish(msg);
    }

    // Membros ROS 2
    std::unique_ptr<ats_tether::TetherMoorDynSystem> tether_physics_;
    std::vector<double> current_positions_;
    bool initialized_;
    rclcpp::Time last_init_attempt_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
    rclcpp::TimerBase::SharedPtr physics_timer_;

    // Membros Gazebo Transport
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_wrench_pub_;

    // Nomes uniformizados das entidades no Gazebo
    const std::string boat_link_name_ = "wamv::wamv/base_link";
    const std::string drone_link_name_ = "x500::base_link";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AtsTetherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
