import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ats_bringup = get_package_share_directory('ats_bringup')
    pkg_ats_description = get_package_share_directory('ats_description')
    
    # Caminhos PX4
    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    px4_build_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default')

    # Configurar Caminhos de Recursos
    local_models_dir = os.path.join(pkg_ats_description, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{local_models_dir}:{gz_resource_path}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    # 1. Mundo (WAM-V World para ter água)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': 'wamv_world'}.items()
    )

    # 2. Spawn do Sistema Marsupial (Barco + Drone)
    spawn_marsupial = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'wamv_world',
            '-file', os.path.join(local_models_dir, 'marsupial', 'model.sdf'),
            '-name', 'marsupial_system',
            '-allow_renaming', 'false',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # 3. Ligar o PX4 ao Drone Aninhado
    # O nome no Gazebo será "marsupial_system::drone"
    px4_env = os.environ.copy()
    px4_env.update({
        'PX4_GZ_WORLD': 'wamv_world', 
        'PX4_GZ_MODEL_NAME': 'marsupial_system',
        'PX4_SYS_AUTOSTART': '4001',
        'GZ_SIM_RESOURCE_PATH': new_gz_resource_path
    })

    px4_sitl_process = TimerAction(
        period=5.0, # Dar tempo para o barco estabilizar na água
        actions=[
            ExecuteProcess(
                cmd=[os.path.join(px4_build_dir, 'bin', 'px4'), '-i', '1'],
                cwd=px4_build_dir,
                output='screen',
                env=px4_env
            )
        ]
    )

    # 4. Micro XRCE-DDS Agent
    micro_xrce_agent_path = os.path.join(
        get_package_prefix('micro_xrce_vendor'), 'lib', 'micro_xrce_vendor', 'MicroXRCEAgent'
    )
    micro_ros_agent = ExecuteProcess(
        cmd=[micro_xrce_agent_path, 'udp4', '-p', '8888'],
        output='log'
    )

    # 5. Bridge de tópicos para o Barco
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/barco/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/barco/odometry', '/wamv/ground_truth/odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_launch,
        spawn_marsupial,
        px4_sitl_process,
        micro_ros_agent,
        bridge
    ])
