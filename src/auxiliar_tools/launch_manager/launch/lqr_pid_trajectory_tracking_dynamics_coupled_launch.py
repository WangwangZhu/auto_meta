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
    
    lqr_pid_parameters_configuration = os.path.join(get_package_share_directory('launch_manager'), '../../../../src/auxiliar_tools/launch_manager/config', 'lqr_pid_parameters_configuration.yaml')
    print("ddddddddddddddddddddd")
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='lqr_pid_trajectory_tracking',
            executable='lqr_pid_trajectory_tracking_node',
            name='lqr_pid_trajectory_tracking_node',
            parameters=[lqr_pid_parameters_configuration],
            output='screen',
        )
    ])
