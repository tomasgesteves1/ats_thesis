import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Pacotes
    pkg_ats_description = get_package_share_directory('ats_description')
    
    # Argumentos
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='1.0')
    has_tether_arg = DeclareLaunchArgument('has_tether', default_value='false')
    
    # XACRO -> URDF
    xacro_file = os.path.join(pkg_ats_description, 'urdf', 'wamv_target.urdf.xacro')
    robot_description = Command([
        'xacro ', xacro_file,
        ' has_tether:=', LaunchConfiguration('has_tether')
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn do WAM-V
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'wamv',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )

    # Bridge de tópicos
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/wamv/odometry', '/wamv/ground_truth/odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        has_tether_arg,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
