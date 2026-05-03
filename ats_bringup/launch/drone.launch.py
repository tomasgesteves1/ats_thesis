import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Caminho base do PX4
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_build_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default')
    px4_models_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')

    # Garantir que o PX4 sabe onde encontrar os modelos 3D do Gazebo
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{px4_models_dir}:{gz_resource_path}" if gz_resource_path else px4_models_dir

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    # Definir as variáveis de ambiente necessárias para o PX4 arrancar e fazer o spawn do modelo no Gazebo
    px4_env = {
        'PX4_GZ_WORLD': 'wamv_world', # Nome do mundo atual (definido no barco)
        'PX4_SIM_MODEL': 'gz_x500',   # Nome do modelo do drone do PX4
        'PX4_GZ_MODEL_POSE': '0,0,1.4', # Posição inicial mais baixa para não capotar ao cair no barco
        'GZ_SIM_RESOURCE_PATH': new_gz_resource_path
    }

    # Processo que corre o PX4 Autopilot em modo SITL
    # A flag "-i 1" define o ID do veiculo (instance 1), útil caso queiras ter barco(0) e drone(1)
    px4_sitl_process = ExecuteProcess(
        cmd=[os.path.join(px4_build_dir, 'bin', 'px4'), '-i', '1'],
        cwd=px4_build_dir,
        output='screen',
        additional_env=px4_env
    )

    # Micro XRCE-DDS Agent para a bridge entre ROS 2 e PX4
    micro_xrce_agent_path = os.path.join(
        get_package_share_directory('micro_xrce_vendor').replace('share/micro_xrce_vendor', 'lib/micro_xrce_vendor'), 
        'MicroXRCEAgent'
    )
    
    micro_ros_agent = ExecuteProcess(
        cmd=[micro_xrce_agent_path, 'udp4', '-p', '8888'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        px4_sitl_process,
        micro_ros_agent
    ])
