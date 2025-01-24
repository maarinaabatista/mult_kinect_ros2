import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Caminho para o arquivo de configuração do RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('mult_kinect_ros2'), 'config', 'mult_kinects.rviz'
    )


    return LaunchDescription([
        # Argumento opcional para definir um nome diferente para o nó
        DeclareLaunchArgument('node_name', default_value='mult_kinect_ros2_node'),

        # Inicia o nó principal do ROS2
        Node(
            package='mult_kinect_ros2',
            executable='mult_kinect_ros2_node',
            name='mult_kinect_ros2_node',
            output='screen',
        ),

        # Abre o RViz com a configuração definida
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])
