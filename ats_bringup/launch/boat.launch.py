import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Pacotes
    pkg_ats_bringup = get_package_share_directory('ats_bringup')
    pkg_ats_description = get_package_share_directory('ats_description')
    
    # Argumentos
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='1.0')
    world_arg = DeclareLaunchArgument('world', default_value='wamv_world')

    # Caminho para o modelo SDF local
    boat_sdf_path = os.path.join(pkg_ats_description, 'models', 'wamv', 'model.sdf')

    # 1. Incluir o Mundo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 2. Spawn do WAM-V usando o SDF
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', boat_sdf_path,
            '-name', 'boat',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )

    # 3. Bridge de tópicos
    # Nota: Quando spawnado como 'boat' no topo, o odometry volta a ser /model/boat/odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/boat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/wamv/thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('/wamv/thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('/wamv/thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('/wamv/thrusters/right/pos', '/boat/thrusters/right/pos'),
            ('/model/boat/odometry', '/boat/ground_truth/odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        world_arg,
        world_launch,
        spawn_entity,
        bridge,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'use_sim_time': True}]
        )
    ])
