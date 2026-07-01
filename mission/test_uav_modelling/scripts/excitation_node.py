#!/usr/bin/env python3
"""
UAV Trajectory Excitation Generator for System Identification (Closed-Loop)
This node publishes a sequence of position step references (doublets) on X, Y, and Z
for the UAV MPC to track. This keeps the drone safe and stable via closed-loop control,
while generating the transient attitude and thrust data required for identification.

Author: Tomas (Tethered UAV-USV Project)
Language: Python 3
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import Float64MultiArray
import numpy as np

class UavTrajectoryExcitationNode(Node):
    def __init__(self):
        super().__init__('uav_trajectory_excitation_node')
        
        # 1. Declare Parameters
        self.declare_parameter('takeoff_height', 4.0)
        self.declare_parameter('step_amplitude_xy', 3.0)      # step in X and Y (meters)
        self.declare_parameter('step_amplitude_z', 1.5)       # step in Z (meters)
        self.declare_parameter('step_duration', 8.0)          # duration of each step state (seconds)
        self.declare_parameter('takeoff_delay', 5.0)          # delay to allow takeoff and stabilization (seconds)
        self.declare_parameter('horizon_stages', 50)          # number of stages in the MPC horizon
        self.declare_parameter('control_period', 0.02)        # MPC control period (Ts)
        self.declare_parameter('world_frame', 'world')

        # Get values
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        self.step_amplitude_xy = self.get_parameter('step_amplitude_xy').get_parameter_value().double_value
        self.step_amplitude_z = self.get_parameter('step_amplitude_z').get_parameter_value().double_value
        self.step_duration = self.get_parameter('step_duration').get_parameter_value().double_value
        self.takeoff_delay = self.get_parameter('takeoff_delay').get_parameter_value().double_value
        self.horizon_stages = self.get_parameter('horizon_stages').get_parameter_value().integer_value
        self.control_period = self.get_parameter('control_period').get_parameter_value().double_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        # 2. State variables
        self.status_received = False
        self.latest_status = VehicleStatus()
        self.current_system_id = 2  # Default to 2
        
        self.start_time = None
        self.land_command_sent = False
        self.last_phase = 0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = self.takeoff_height
        
        # Throttled logging helper
        self.last_log_time = {}

        # 3. Setup QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 4. Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_profile
        )

        # 5. Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/px4_1/fmu/in/vehicle_command',
            qos_profile
        )
        self.path_pub = self.create_publisher(
            Path,
            'reference_path',
            10
        )
        self.diagnostics_pub = self.create_publisher(
            Float64MultiArray,
            'excitation_diagnostics',
            10
        )

        # 6. Timer running at 50Hz (0.02s) to publish reference path
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('UAV Trajectory Excitation Node (Closed-Loop) Initialized!')

    def throttle_info(self, key, msg, period_seconds=2.0):
        now = self.get_clock().now().nanoseconds / 1e9
        if key not in self.last_log_time or (now - self.last_log_time[key]) >= period_seconds:
            self.get_logger().info(msg)
            self.last_log_time[key] = now

    def status_callback(self, msg):
        self.latest_status = msg
        self.status_received = True
        self.current_system_id = msg.system_id

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = 0.0
        msg.target_system = self.current_system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def control_loop(self):
        # 1. Check if the drone has armed to start our timer
        if self.start_time is None:
            if self.status_received and self.latest_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.start_time = self.get_clock().now()
                self.get_logger().info("UAV armed! Starting closed-loop trajectory excitation timer.")
                phase = 1
                elapsed = 0.0
            else:
                self.throttle_info("wait_arm", "Waiting for UAV to be ARMED to start excitation sequence...", 5.0)
                # Keep target at hover starting point
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                phase = 0
                elapsed = 0.0
        else:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            # Sequence state machine based on elapsed time
            # 1. Takeoff and Hover (Wait for UAV to stabilize in the air)
            if elapsed < self.takeoff_delay:
                phase = 1
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_takeoff", f"Takeoff / Hover: t={elapsed:.2f}s/{self.takeoff_delay}s", 2.0)
            
            # --- X Axis Steps ---
            elif elapsed < self.takeoff_delay + self.step_duration:
                phase = 2
                self.target_x = self.step_amplitude_xy
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_x_pos", f"Step X Positive (+{self.step_amplitude_xy}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 2 * self.step_duration:
                phase = 3
                self.target_x = -self.step_amplitude_xy
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_x_neg", f"Step X Negative (-{self.step_amplitude_xy}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 3 * self.step_duration:
                phase = 4
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_x_zero", f"Return X Home (0m): t={elapsed:.2f}s", 2.0)
                
            # --- Y Axis Steps ---
            elif elapsed < self.takeoff_delay + 4 * self.step_duration:
                phase = 5
                self.target_x = 0.0
                self.target_y = self.step_amplitude_xy
                self.target_z = self.takeoff_height
                self.throttle_info("phase_y_pos", f"Step Y Positive (+{self.step_amplitude_xy}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 5 * self.step_duration:
                phase = 6
                self.target_x = 0.0
                self.target_y = -self.step_amplitude_xy
                self.target_z = self.takeoff_height
                self.throttle_info("phase_y_neg", f"Step Y Negative (-{self.step_amplitude_xy}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 6 * self.step_duration:
                phase = 7
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_y_zero", f"Return Y Home (0m): t={elapsed:.2f}s", 2.0)
                
            # --- Z Axis Steps ---
            elif elapsed < self.takeoff_delay + 7 * self.step_duration:
                phase = 8
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height + self.step_amplitude_z
                self.throttle_info("phase_z_up", f"Step Z Up (+{self.step_amplitude_z}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 8 * self.step_duration:
                phase = 9
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height - self.step_amplitude_z
                self.throttle_info("phase_z_down", f"Step Z Down (-{self.step_amplitude_z}m): t={elapsed:.2f}s", 2.0)
                
            elif elapsed < self.takeoff_delay + 9 * self.step_duration:
                phase = 10
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.throttle_info("phase_z_zero", f"Return Z Home ({self.takeoff_height}m): t={elapsed:.2f}s", 2.0)
                
            # --- Land ---
            else:
                phase = 11
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = 0.0
                
                if not self.land_command_sent:
                    self.get_logger().info("Trajectory sequence completed. Sending Land command to PX4...")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.land_command_sent = True

        # Print phase transitions immediately (not throttled)
        if self.start_time is not None:
            if phase != self.last_phase:
                phase_names = {
                    1: "Takeoff & Hover",
                    2: "Step X Positive",
                    3: "Step X Negative",
                    4: "Return X Home",
                    5: "Step Y Positive",
                    6: "Step Y Negative",
                    7: "Return Y Home",
                    8: "Step Z Up",
                    9: "Step Z Down",
                    10: "Return Z Home",
                    11: "Landing"
                }
                name = phase_names.get(phase, "Unknown")
                self.get_logger().info(f"--- [PHASE TRANSITION] Entering Phase {phase}: {name} (elapsed: {elapsed:.2f}s) ---")
                self.last_phase = phase

        # 2. Build and publish the reference path for MPC
        # We only publish the path if we haven't landed yet
        if not self.land_command_sent:
            path_msg = Path()
            path_msg.header.frame_id = self.world_frame
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for _ in range(self.horizon_stages + 1):
                pose = PoseStamped()
                pose.header.frame_id = self.world_frame
                pose.header.stamp = path_msg.header.stamp
                pose.pose.position.x = self.target_x
                pose.pose.position.y = self.target_y
                pose.pose.position.z = self.target_z
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)

        # 3. Publish diagnostics
        diag_msg = Float64MultiArray()
        diag_msg.data = [
            float(self.get_clock().now().nanoseconds / 1e9),
            float(phase),
            elapsed,
            self.target_x,
            self.target_y,
            self.target_z
        ]
        self.diagnostics_pub.publish(diag_msg)

        # Stop timer once landed and disarmed
        if self.land_command_sent and self.status_received and self.latest_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info("Drone disarmed. Sequence complete. Shutting down excitation node.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = UavTrajectoryExcitationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory excitation node stopped by keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
