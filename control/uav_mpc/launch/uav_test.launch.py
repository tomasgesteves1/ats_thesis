import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Caminho do ficheiro de configuração YAML
    config_file = os.path.join(
        get_package_share_directory('uav_mpc'),
        'config',
        'uav_mpc.yaml'
    )

    # O nosso nó MPC do Drone
    uav_node = Node(
        package='uav_mpc',
        executable='uav_mpc_node',
        name='uav_mpc_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}],
        remappings=[
            # O tópico de odom do uav_mpc tem de se ligar à Odometria vinda do moordyn_marsupial
            ('odom', '/drone/ground_truth/odometry'),
            # Subscrever o comprimento do cabo real do MoorDyn
            ('tether_length', '/moordyn_tether_node/tether_length'),
        ]
    )

    return LaunchDescription([
        uav_node
    ])
