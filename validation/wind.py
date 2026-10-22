#!/usr/bin/env python3
"""Wind on the UAV for tests: drag force 0.5 rho Cd A v^2 along a fixed direction, with gusts of +-30 % (sum of three
slow sinusoids), published as an external force of the tether node. Usage: wind.py <speed m/s> [direction deg]"""
import sys, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

RHO, CD, AREA = 1.225, 1.0, 0.06
GUSTS = [(7.0, 0.0, 0.5), (13.0, 1.3, 0.3), (23.0, 2.1, 0.2)]   # period [s], phase, weight (weights sum to 1)


class Wind(Node):
    def __init__(self, speed, direction_deg):
        super().__init__('wind_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.f_mean = 0.5 * RHO * CD * AREA * speed ** 2
        self.d = math.radians(direction_deg)
        self.pub = self.create_publisher(Vector3, '/moordyn_tether_node/external_force_drone', 10)
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f'wind {speed} m/s towards {direction_deg} deg: mean force {self.f_mean:.2f} N')

    def loop(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        gust = sum(w * math.sin(2 * math.pi * t / p + ph) for p, ph, w in GUSTS)
        f = self.f_mean * (1.0 + 0.3 * gust)
        self.pub.publish(Vector3(x=f * math.cos(self.d), y=f * math.sin(self.d), z=0.0))


rclpy.init()
rclpy.spin(Wind(float(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 90.0))
