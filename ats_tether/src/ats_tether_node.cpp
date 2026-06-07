#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
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
        current_positions_[5] = msg->pose.pose.position.z - 0.2;

        check_initialization();
    }

    void check_initialization()
    {
        if (!initialized_ && current_positions_[2] > 0.1 && current_positions_[5] > 0.1)
        {
            if (tether_physics_->initialize(current_positions_))
            {
                initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "MoorDyn inicializado com as posições globais!");
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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr boat_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
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
