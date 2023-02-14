import launch
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import ament_index_python.packages


def generate_launch_description():
    
    planner_parameters_configuration = os.path.join(get_package_share_directory('lattice_planner'), '../../../../src/path_planning_modules/lattice_planner/config', 'lattice_planner.yaml')

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='lattice_planner',
            # namespace='mpc_trajectory_tracking',
            executable='lattice_planner_node',
            name='lattice_planner_node',
            parameters=[planner_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        )
    ])
