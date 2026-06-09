#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>

class ForceVisualizer : public rclcpp::Node {
public:
    ForceVisualizer() : Node("force_visualizer") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tether_forces_markers", 10);
        
        gz_node_.Subscribe("/world/wamv_world/wrench", &ForceVisualizer::gz_wrench_callback, this);
        
        // Initialize filters
        boat_force_filtered_ = {0.0, 0.0, 0.0};
        drone_force_filtered_ = {0.0, 0.0, 0.0};
        
        RCLCPP_INFO(this->get_logger(), "Visualizador de Forcas Iniciado! (C++ Filtered)");
    }

private:
    void gz_wrench_callback(const gz::msgs::EntityWrench &msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "tether_forces";
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = 0; p_start.y = 0;
        
        // Fator de compromisso: 30cm por cada Newton de forca
        double scale_factor = 0.3; 
        double alpha = 0.05; // Fator de Low-pass filter

        double max_length = 10.0; // Comprimento maximo da seta (10 metros)
        
        double raw_x = msg.wrench().force().x();
        double raw_y = msg.wrench().force().y();
        double raw_z = msg.wrench().force().z();
        
        if (msg.entity().name() == "wamv::wamv/base_link") {
            marker.header.frame_id = "boat/base_link";
            marker.id = 0;
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8;
            p_start.z = 1.3;
            
            boat_force_filtered_[0] = (alpha * raw_x) + ((1.0 - alpha) * boat_force_filtered_[0]);
            boat_force_filtered_[1] = (alpha * raw_y) + ((1.0 - alpha) * boat_force_filtered_[1]);
            boat_force_filtered_[2] = (alpha * raw_z) + ((1.0 - alpha) * boat_force_filtered_[2]);
            
            p_end.x = boat_force_filtered_[0] * scale_factor;
            p_end.y = boat_force_filtered_[1] * scale_factor;
            p_end.z = p_start.z + (boat_force_filtered_[2] * scale_factor);
            
        } else if (msg.entity().name() == "x500::base_link") {
            marker.header.frame_id = "drone/base_link";
            marker.id = 1;
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 0.8;
            p_start.z = -0.2;
            
            drone_force_filtered_[0] = (alpha * raw_x) + ((1.0 - alpha) * drone_force_filtered_[0]);
            drone_force_filtered_[1] = (alpha * raw_y) + ((1.0 - alpha) * drone_force_filtered_[1]);
            drone_force_filtered_[2] = (alpha * raw_z) + ((1.0 - alpha) * drone_force_filtered_[2]);
            
            p_end.x = drone_force_filtered_[0] * scale_factor;
            p_end.y = drone_force_filtered_[1] * scale_factor;
            p_end.z = p_start.z + (drone_force_filtered_[2] * scale_factor);
            
        } else {
            return; // Entidade desconhecida
        }

        // Clamp mechanism to prevent giant arrows
        double dx = p_end.x - p_start.x;
        double dy = p_end.y - p_start.y;
        double dz = p_end.z - p_start.z;
        double length = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (length > max_length) {
            double shrink_ratio = max_length / length;
            p_end.x = p_start.x + (dx * shrink_ratio);
            p_end.y = p_start.y + (dy * shrink_ratio);
            p_end.z = p_start.z + (dz * shrink_ratio);
        }
        
        // As forcas de gravidade sao pequenas (~0.3N), vamos desenhar sempre,
        // garantindo uma seta base visivel
        if (length < 0.05) {
             // Forcar um comprimento minimo visual de 10cm apontado para baixo
             p_end.x = p_start.x;
             p_end.y = p_start.y;
             p_end.z = p_start.z - 0.1;
        }
        
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);
        
        // Espessura muito maior para as setas serem visiveis ao longe
        marker.scale.x = 0.15; // Diametro da haste (triplicado)
        marker.scale.y = 0.3;  // Diametro da cabeca (triplicado)
        marker.scale.z = 0.4;  // Comprimento da cabeca (duplicado)

        
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    gz::transport::Node gz_node_;
    std::vector<double> boat_force_filtered_;
    std::vector<double> drone_force_filtered_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceVisualizer>());
    rclcpp::shutdown();
    return 0;
}
