import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    planar_grasp = Node(
        package='planar_grasp',
        executable='planar_grasp',
        output='screen',
    )

    nodes = [
        planar_grasp
    ]

    return LaunchDescription(nodes)