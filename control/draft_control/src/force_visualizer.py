#!/usr/bin/env python3

import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Usamos o subprocess para escutar o Gazebo via CLI
import subprocess
import threading
import json

class ForceVisualizer(Node):
    def __init__(self):
        super().__init__('force_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, '/tether_forces_markers', 10)
        
        self.boat_force = [0.0, 0.0, 0.0]
        self.drone_force = [0.0, 0.0, 0.0]

        # Iniciar thread para ler o Gazebo em background
        self.gz_thread = threading.Thread(target=self.gz_listener)
        self.gz_thread.daemon = True
        self.gz_thread.start()

        # Publicar markers a 10Hz
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.get_logger().info("Force Visualizer (ROS <-> GZ CLI) Started!")

    def gz_listener(self):
        # Escuta o tópico do Gazebo como JSON para fácil parsing
        # (Este truque evita termos de compilar mensagens protobuf em Python)
        process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/wamv_world/wrench'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        current_entity = ""
        fx = fy = fz = 0.0

        for line in iter(process.stdout.readline, ''):
            line = line.strip()
            
            if "name: \"wamv::wamv/base_link\"" in line:
                current_entity = "boat"
            elif "name: \"x500::base_link\"" in line:
                current_entity = "drone"
            
            if "force {" in line:
                # O comando 'gz topic' imprime algo como: force { x: 1.0 y: 2.0 z: 3.0 }
                # Mas em várias linhas. Precisamos de um pequeno state machine.
                pass
                
            if "x:" in line and current_entity != "":
                try: fx = float(line.split("x:")[1])
                except: pass
            if "y:" in line and current_entity != "":
                try: fy = float(line.split("y:")[1])
                except: pass
            if "z:" in line and current_entity != "":
                try: 
                    fz = float(line.split("z:")[1])
                    if current_entity == "boat":
                        self.boat_force = [fx, fy, fz]
                    elif current_entity == "drone":
                        self.drone_force = [fx, fy, fz]
                    current_entity = "" # Reset
                except: pass

    def create_arrow_marker(self, entity_id, force_vec, frame_id, rgba, z_offset=0.0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tether_forces"
        marker.id = entity_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start point (Entity origin)
        p_start = Point()
        p_start.x = 0.0
        p_start.y = 0.0
        p_start.z = z_offset

        # End point (Direction and magnitude of force)
        # Multiplicamos por 2 para ser mais visível (Escala visual)
        scale_factor = 2.0
        p_end = Point()
        p_end.x = force_vec[0] * scale_factor
        p_end.y = force_vec[1] * scale_factor
        p_end.z = z_offset + (force_vec[2] * scale_factor)

        marker.points = [p_start, p_end]

        # Thickness of the arrow
        marker.scale.x = 0.05 # Shaft diameter
        marker.scale.y = 0.1  # Head diameter
        marker.scale.z = 0.2  # Head length

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker

    def publish_markers(self):
        msg = MarkerArray()
        
        # Boat Arrow (Red) - Opcionalmente centrado na âncora (Z = 1.3)
        if any(f != 0.0 for f in self.boat_force):
            msg.markers.append(self.create_arrow_marker(
                0, self.boat_force, "boat/base_link", [1.0, 0.0, 0.0, 0.8], z_offset=1.3))

        # Drone Arrow (Blue) - Opcionalmente centrado no gancho (Z = -0.2)
        if any(f != 0.0 for f in self.drone_force):
            msg.markers.append(self.create_arrow_marker(
                1, self.drone_force, "drone/base_link", [0.0, 0.0, 1.0, 0.8], z_offset=-0.2))

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
