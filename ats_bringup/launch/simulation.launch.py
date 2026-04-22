import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ats_gazebo_src = '/home/tomas/ats_ws/src/ats_gazebo'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(ats_gazebo_src, 'worlds', 'wamv_world.sdf')
    models_path = os.path.join(ats_gazebo_src, 'models')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Ponte para os motores e sensores
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/wamv/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/wamv/joint/left_engine_propeller_joint/cmd_thrust', '/wamv/thrusters/left/thrust'),
            ('/model/wamv/joint/right_engine_propeller_joint/cmd_thrust', '/wamv/thrusters/right/thrust'),
            ('/model/wamv/odometry', '/wamv/ground_truth/odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        bridge
    ])
