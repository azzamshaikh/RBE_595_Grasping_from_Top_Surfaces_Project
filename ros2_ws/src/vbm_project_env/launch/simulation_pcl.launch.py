import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('vbm_project_env'),
        'worlds',
        'simulation_multiple_objects.world')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
                launch_arguments={'world':world_path}.items(),
    )

    simulation_description_path = os.path.join(get_package_share_directory('vbm_project_env'))
    simulation_urdf_path = os.path.join(simulation_description_path,'urdf','camera.urdf')
    robot_description_config = open(simulation_urdf_path).read()
    robot_description = {'robot_description' : robot_description_config}
    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                        arguments=['-file',simulation_urdf_path,
                                    '-entity','camera',
                                    #'-x','-1.5',
                                    '-z','1.5',
                                    '-P','1.57'],
                        output='both' )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    denoised_point_cloud = Node(
        package='denoise',
        executable='denoise_pointcloud',
        output='screen',
    )
    
    object_point_cloud = Node(
        package='major_plane_detection',
        executable='plane_detection',
        output='log'
    )

    clustering_clouds = Node(
        package="clustering",
        executable="clustering",
        output = "screen"
    )

    planar_surface = Node(
        package="planar_surface",
        executable="planar_surface_pcl",
        output = "screen"
    )

    planar_grasp = Node(
        package="planar_grasp",
        executable="planar_grasp_pcl",
        output = "log"
    )

    grasp_quality = Node(
        package="grasp_quality",
        executable="grasp_quality",
        output="screen"
    )

    rviz_file = os.path.join(
        get_package_share_directory('vbm_project_env'),
        'rviz',
        'simulation_multiple_objects.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_file]
    )

    nodes = [
        gazebo,
        spawn_entity,
        node_robot_state_publisher,
        denoised_point_cloud,
        object_point_cloud,
        clustering_clouds,
        planar_surface,
        planar_grasp,
        grasp_quality,
        rviz_node
    ]

    return LaunchDescription(nodes)