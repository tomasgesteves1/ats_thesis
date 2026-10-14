import os
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from .transforms import quaternion_to_euler_enu, quaternion_ned_to_euler_enu

def read_bag(bag_path):
    """
    Reads ROS 2 bag from the given path using rosbags (pure Python)
    and returns synchronized time-series arrays.
    """
    bag_p = Path(bag_path)
    if not bag_p.exists():
        raise FileNotFoundError(f"Bag path '{bag_path}' does not exist.")
        
    # Standard topics we want to parse
    odom_topic = '/drone/ground_truth/odometry'
    setpoint_topic = '/px4_1/fmu/in/vehicle_attitude_setpoint'
    hover_topic = '/px4_1/fmu/out/hover_thrust_estimate'
    diag_topic = '/uav_excitation_node/excitation_diagnostics'
    
    # Store raw data
    raw_odom = []      # (t, x, y, z, roll, pitch, yaw, vx, vy, vz)
    raw_setpoint = []  # (t, roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd)
    raw_hover = []     # (t, hover_thrust)
    raw_diag = []      # (t, phase, target_x, target_y, target_z)
    
    # Open bag file using AnyReader
    with AnyReader([bag_p]) as reader:
        connections = [c for c in reader.connections if c.topic in [odom_topic, setpoint_topic, hover_topic, diag_topic]]
        
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            t_sec = timestamp_ns / 1e9
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            if connection.topic == odom_topic:
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                roll, pitch, yaw = quaternion_to_euler_enu([o.x, o.y, o.z, o.w])
                raw_odom.append((t_sec, p.x, p.y, p.z, roll, pitch, yaw, v.x, v.y, v.z))
                
            elif connection.topic == setpoint_topic:
                q_d = msg.q_d  # [w, x, y, z] in PX4
                roll_cmd, pitch_cmd, yaw_cmd = quaternion_ned_to_euler_enu(q_d)
                thrust_cmd = -msg.thrust_body[2]
                raw_setpoint.append((t_sec, roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd))
                
            elif connection.topic == hover_topic:
                if msg.valid:
                    raw_hover.append((t_sec, msg.hover_thrust))
                    
            elif connection.topic == diag_topic:
                if len(msg.data) >= 6:
                    phase = msg.data[1]
                    tx = msg.data[3]
                    ty = msg.data[4]
                    tz = msg.data[5]
                    raw_diag.append((t_sec, phase, tx, ty, tz))
                    
    if not raw_odom:
        raise ValueError(f"No odometry messages found in bag for topic: {odom_topic}")
    
    # Convert to numpy arrays
    odom_arr = np.array(raw_odom)
    t_master = odom_arr[:, 0]
    
    # Subtract initial time to start at 0
    t0 = t_master[0]
    t_master = t_master - t0
    
    # Synchronize signals by interpolating onto the master time grid (odometry)
    def sync_signal(raw_list, num_channels, default_val=0.0):
        if not raw_list:
            return np.ones((len(t_master), num_channels)) * default_val
        arr = np.array(raw_list)
        t_signal = arr[:, 0] - t0
        synced = np.zeros((len(t_master), num_channels))
        for col in range(num_channels):
            synced[:, col] = np.interp(t_master, t_signal, arr[:, col + 1])
        return synced

    setpoint_synced = sync_signal(raw_setpoint, 4, default_val=0.0)
    hover_synced = sync_signal(raw_hover, 1, default_val=0.7265)
    diag_synced = sync_signal(raw_diag, 4, default_val=0.0)
    
    dataset = {
        'time': t_master,
        'x': odom_arr[:, 1],
        'y': odom_arr[:, 2],
        'z': odom_arr[:, 3],
        'roll': odom_arr[:, 4],
        'pitch': odom_arr[:, 5],
        'yaw': odom_arr[:, 6],
        'vx': odom_arr[:, 7],
        'vy': odom_arr[:, 8],
        'vz': odom_arr[:, 9],
        'roll_cmd': setpoint_synced[:, 0],
        'pitch_cmd': setpoint_synced[:, 1],
        'yaw_cmd': setpoint_synced[:, 2],
        'thrust_cmd': setpoint_synced[:, 3],
        'hover_thrust': hover_synced[:, 0],
        'phase': diag_synced[:, 0],
        'target_x': diag_synced[:, 1],
        'target_y': diag_synced[:, 2],
        'target_z': diag_synced[:, 3]
    }
    
    return dataset
