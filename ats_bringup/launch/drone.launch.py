import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ats_description = get_package_share_directory('ats_description')
    
    # Argumentos
    world_arg = DeclareLaunchArgument('world', default_value='wamv_world')
    
    # Caminhos PX4
    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    px4_build_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default')

    # Configurar Resource Path
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    local_models_dir = os.path.join(pkg_ats_description, 'models')
    new_gz_resource_path = f"{local_models_dir}:{gz_resource_path}"

    # 1. Spawn do Drone via ROS 2 (comunica com o Gazebo que já está aberto)
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(local_models_dir, 'x500', 'model.sdf'),
            '-name', 'drone',
            '-allow_renaming', 'false',
            '-x', '0.3',
            '-y', '0.0',
            '-z', '1.4'
        ],
        output='screen'
    )

    # 2. Ligar o PX4
    px4_env = os.environ.copy()
    px4_env.update({
        'PX4_GZ_WORLD': LaunchConfiguration('world'), 
        'PX4_GZ_MODEL_NAME': 'drone',
        'PX4_SYS_AUTOSTART': '4001',
        'GZ_SIM_RESOURCE_PATH': new_gz_resource_path
    })

    px4_sitl_process = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[os.path.join(px4_build_dir, 'bin', 'px4'), '-i', '1'],
                cwd=px4_build_dir,
                output='screen',
                env=px4_env
            )
        ]
    )

    # 3. Micro XRCE-DDS Agent
    micro_xrce_agent_path = os.path.join(
        get_package_prefix('micro_xrce_vendor'), 'lib', 'micro_xrce_vendor', 'MicroXRCEAgent'
    )
    micro_ros_agent = ExecuteProcess(
        cmd=[micro_xrce_agent_path, 'udp4', '-p', '8888'],
        output='log'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_resource_path),
        world_arg,
        spawn_drone,
        px4_sitl_process,
        micro_ros_agent,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'use_sim_time': True}]
        )
    ])