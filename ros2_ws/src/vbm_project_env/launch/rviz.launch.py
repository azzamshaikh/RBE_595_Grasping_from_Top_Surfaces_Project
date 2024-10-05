import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_file = os.path.join(
        get_package_share_directory('vbm_project_env'),
        'rviz',
        'simulation.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_file]
    )

    nodes = [
        rviz_node
    ]

    return LaunchDescription(nodes)