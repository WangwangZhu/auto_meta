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
    
    planner_parameters_configuration = os.path.join(get_package_share_directory('launch_manager'), '../../../../src/launch_manager/config', 'highway_path_planning_with_prediction.yaml')

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='highway_path_planning_with_prediction',
            # namespace='mpc_trajectory_tracking',
            executable='highway_path_planning_with_prediction_node',
            name='highway_path_planning_with_prediction_node',
            parameters=[planner_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        )
    ])
