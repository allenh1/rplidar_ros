from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='rplidar',
            package='rplidar_ros',
            node_executable='rplidar',
            output='screen',
            parameters=[
                path.join(get_package_share_directory('rplidar_ros'), 'config',
                          'rplidar_a3.yaml')
            ],
        ),
        Node(
            node_name='rplidar_client',
            package='rplidar_ros',
            node_executable='rplidar_client',
            output='screen',
        ),
    ])
