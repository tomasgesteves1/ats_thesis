#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ats_tether/TetherMoorDynSystem.hpp"

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

        // 4. Subscrever as poses globais do Gazebo
        boat_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/boat/ground_truth/odometry", 10, std::bind(&AtsTetherNode::boat_callback, this, _1));

        drone_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/drone/ground_truth/odometry", 10, std::bind(&AtsTetherNode::drone_callback, this, _1));

        // 5. Publicador para enviar forças para o Gazebo
        wrench_pub_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>(
            "/world/wamv_world/wrench", 10);
            
        // Publicador para desenhar a catenaria do cabo
        geometry_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/tether_geometry_marker", 10);

        // 6. Timer para correr a física a 50Hz (20ms)
        physics_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AtsTetherNode::physics_loop, this));

        RCLCPP_INFO(this->get_logger(), "Nó AtsTether inicializado. Usando nomes uniformes 'boat' e 'drone'.");
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
        current_positions_[5] = msg->pose.pose.position.z + 0.26;

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
        std::vector<std::vector<double>> cable_nodes;
        
        if (tether_physics_->step(current_positions_, 0.02, out_forces, cable_nodes)) {
            // Aplicar força no Barco
            publish_wrench(boat_link_name_, out_forces[0], out_forces[1], out_forces[2]);

            // Aplicar força no Drone
            publish_wrench(drone_link_name_, out_forces[3], out_forces[4], out_forces[5]);
            
            // Publicar geometria do cabo
            publish_geometry(cable_nodes);
        }
    }

    void publish_geometry(const std::vector<std::vector<double>>& nodes)
    {
        if (nodes.empty()) return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "tether_catenary";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Largura da linha
        marker.scale.x = 0.01; // 1 cm de espessura

        // Cor do cabo (Preto ou Cinzento Escuro)
        marker.color.r = 0.2;
        marker.color.g = 0.2;
        marker.color.b = 0.2;
        marker.color.a = 1.0;

        for (const auto& node_pos : nodes) {
            geometry_msgs::msg::Point p;
            p.x = node_pos[0];
            p.y = node_pos[1];
            p.z = node_pos[2];
            marker.points.push_back(p);
        }

        geometry_pub_->publish(marker);
    }

    void publish_wrench(const std::string& entity_name, double fx, double fy, double fz)
    {
        ros_gz_interfaces::msg::EntityWrench msg;
        msg.entity.name = entity_name;
        msg.entity.type = 3; // LINK
        msg.wrench.force.x = fx;
        msg.wrench.force.y = fy;
        msg.wrench.force.z = fz;
        
        wrench_pub_->publish(msg);
    }

    // Membros
    std::unique_ptr<ats_tether::TetherMoorDynSystem> tether_physics_;
    std::vector<double> current_positions_;
    bool initialized_;
    rclcpp::Time last_init_attempt_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr geometry_pub_;
    rclcpp::TimerBase::SharedPtr physics_timer_;

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
