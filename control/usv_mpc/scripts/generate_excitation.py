#!/usr/bin/env python3
"""
Automated Excitation Signal Generator Node for WAM-V System Identification.
This node publishes rich, persistent excitation signals (PRBS, steps, and chirps)
to the USV thrusters (forces and angles) to capture full 3-DOF coupled dynamics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
import random


class USVExcitationGenerator(Node):
    def __init__(self):
        super().__init__('usv_excitation_generator')
        
        # Declare parameters
        self.declare_parameter('frequency', 20.0)      # Publication frequency (Hz)
        self.declare_parameter('total_duration', 120.0) # Total test duration (seconds)
        
        self.freq = self.get_parameter('frequency').value
        self.dt = 1.0 / self.freq
        self.total_duration = self.get_parameter('total_duration').value
        self.cooldown_duration = 2.0  # seconds to publish zeros before shutdown
        self.elapsed_time = 0.0
        
        # Calibrated limits from SDF
        self.max_thrust = 2300.0  # N (slightly under physical limit to prevent clipping)
        self.max_angle = 1.5708   # rad (+/- 90 degrees)
        
        # Publishers
        self.left_thrust_pub = self.create_publisher(Float64, '/boat/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/boat/thrusters/right/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/boat/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/boat/thrusters/right/pos', 10)
        
        # Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # PRBS State Variables
        self.prbs_timer = 0.0
        self.prbs_interval = 2.0  # seconds between state changes
        
        self.l_thrust_cmd = 0.0
        self.r_thrust_cmd = 0.0
        self.l_angle_cmd = 0.0
        self.r_angle_cmd = 0.0
        
        self.get_logger().info(f"WAM-V Excitation Generator Node started.")
        self.get_logger().info(f"Target duration: {self.total_duration}s at {self.freq}Hz.")
        self.get_logger().info("Starting sequence in 3 seconds... Prepare your rosbag record command!")
        
    def generate_prbs(self, min_val, max_val, hold_min=1.0, hold_max=4.0):
        """
        Generates a Pseudo-Random Binary Sequence (PRBS) step.
        """
        if self.prbs_timer <= 0.0:
            self.prbs_timer = random.uniform(hold_min, hold_max)
            return random.choice([min_val, 0.0, max_val])
        return None

    def timer_callback(self):
        self.elapsed_time += self.dt
        self.prbs_timer -= self.dt
        
        # Initial countdown/wait
        if self.elapsed_time < 3.0:
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            return
            
        t = self.elapsed_time - 3.0
        
        # ======================================================================
        # PHASED EXCITATION SIGNAL PLANNING
        # ======================================================================
        if t < 30.0:
            # ------------------------------------------------------------------
            # PHASE 1: SURGE PURE DYNAMICS (0s to 30s)
            # Engines straight, apply identical longitudinal PRBS thrust
            # ------------------------------------------------------------------
            new_val = self.generate_prbs(-self.max_thrust * 0.7, self.max_thrust, hold_min=1.5, hold_max=3.5)
            if new_val is not None:
                self.l_thrust_cmd = new_val
                self.r_thrust_cmd = new_val
            self.l_angle_cmd = 0.0
            self.r_angle_cmd = 0.0
            phase_name = "PHASE 1: Surge Pure (PRBS)"
            
        elif t < 60.0:
            # ------------------------------------------------------------------
            # PHASE 2: YAW PURE DYNAMICS (30s to 60s)
            # Engines straight, apply differential PRBS thrust (opposite signs)
            # ------------------------------------------------------------------
            new_val = self.generate_prbs(-self.max_thrust * 0.6, self.max_thrust * 0.6, hold_min=1.0, hold_max=2.5)
            if new_val is not None:
                self.l_thrust_cmd = -new_val
                self.r_thrust_cmd = new_val
            self.l_angle_cmd = 0.0
            self.r_angle_cmd = 0.0
            phase_name = "PHASE 2: Yaw Pure (Differential PRBS)"
            
        elif t < 90.0:
            # ------------------------------------------------------------------
            # PHASE 3: SWAY/LATERAL PURE DYNAMICS (60s to 90s)
            # Apply mathematical crab-walk allocation with PRBS sway force
            # ------------------------------------------------------------------
            # Generate a PRBS sway force target Y_0
            new_val = self.generate_prbs(-self.max_thrust * 0.4, self.max_thrust * 0.4, hold_min=1.5, hold_max=3.0)
            if new_val is not None:
                # Resolve mapping: f_y = 0.5*Y_0, f_x_L = -1.1555*Y_0, f_x_R = 1.1555*Y_0
                Y_0 = new_val
                f_y_L = 0.5 * Y_0
                f_y_R = 0.5 * Y_0
                f_x_L = -1.15553 * Y_0
                f_x_R =  1.15553 * Y_0
                
                # Polar conversion
                self.l_thrust_cmd = np.sqrt(f_x_L**2 + f_y_L**2)
                self.r_thrust_cmd = np.sqrt(f_x_R**2 + f_y_R**2)
                if f_x_L < 0: self.l_thrust_cmd = -self.l_thrust_cmd
                if f_x_R < 0: self.r_thrust_cmd = -self.r_thrust_cmd
                
                self.l_angle_cmd = std_atan2_signed(f_y_L, f_x_L)
                self.r_angle_cmd = std_atan2_signed(f_y_R, f_x_R)
                
            phase_name = "PHASE 3: Sway Pure (Crab Walk PRBS)"
            
        elif t < 120.0:
            # ------------------------------------------------------------------
            # PHASE 4: FULLY COUPLED 3-DOF EXCITATION (90s to 120s)
            # Independent PRBS signals on all 4 actuators simultaneously
            # ------------------------------------------------------------------
            if self.prbs_timer <= 0.0:
                self.prbs_timer = random.uniform(0.8, 2.0)
                self.l_thrust_cmd = random.uniform(-self.max_thrust*0.5, self.max_thrust*0.8)
                self.r_thrust_cmd = random.uniform(-self.max_thrust*0.5, self.max_thrust*0.8)
                self.l_angle_cmd = random.uniform(-self.max_angle*0.6, self.max_angle*0.6)
                self.r_angle_cmd = random.uniform(-self.max_angle*0.6, self.max_angle*0.6)
            phase_name = "PHASE 4: Fully Coupled (Random Actuation)"
            
        elif t < self.total_duration + self.cooldown_duration:
            # Cooldown phase: actively publish zero commands to stop the boat before shutting down
            self.l_thrust_cmd = 0.0
            self.r_thrust_cmd = 0.0
            self.l_angle_cmd = 0.0
            self.r_angle_cmd = 0.0
            phase_name = "COOLDOWN: Stopping Actuators"
        else:
            # Stop sequence completed and verified
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info("="*50)
            self.get_logger().info("EXCITATION SEQUENCE COMPLETED SUCCESSFULLY!")
            self.get_logger().info("You can now stop your rosbag record command.")
            self.get_logger().info("="*50)
            raise KeyboardInterrupt

        # Periodically log current phase name every 4 seconds
        if int(self.elapsed_time * self.freq) % int(4.0 * self.freq) == 0:
            self.get_logger().info(f"{phase_name} | t = {t:.1f}s / {self.total_duration}s")
            
        self.publish_commands(self.l_thrust_cmd, self.r_thrust_cmd, self.l_angle_cmd, self.r_angle_cmd)

    def publish_commands(self, l_thrust, r_thrust, l_angle, r_angle):
        # Enforce strict maximum bounds
        l_thrust = np.clip(l_thrust, -self.max_thrust, self.max_thrust)
        r_thrust = np.clip(r_thrust, -self.max_thrust, self.max_thrust)
        l_angle = np.clip(l_angle, -self.max_angle, self.max_angle)
        r_angle = np.clip(r_angle, -self.max_angle, self.max_angle)
        
        # Publish messages
        msg_l_thrust = Float64()
        msg_l_thrust.data = float(l_thrust)
        self.left_thrust_pub.publish(msg_l_thrust)
        
        msg_r_thrust = Float64()
        msg_r_thrust.data = float(r_thrust)
        self.right_thrust_pub.publish(msg_r_thrust)
        
        msg_l_pos = Float64()
        msg_l_pos.data = float(l_angle)
        self.left_pos_pub.publish(msg_l_pos)
        
        msg_r_pos = Float64()
        msg_r_pos.data = float(r_angle)
        self.right_pos_pub.publish(msg_r_pos)


def std_atan2_signed(f_y, f_x):
    """
    Computes atan2 and preserves the angle sign in case of reverse thrust.
    """
    angle = np.arctan2(f_y, np.abs(f_x))
    if f_x < 0.0:
        angle = -angle
    return angle


def main(args=None):
    rclpy.init(args=args)
    node = USVExcitationGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down excitation generator node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
