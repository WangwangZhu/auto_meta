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
    
    mpc_parameters_configuration = os.path.join(get_package_share_directory('mpc_trajectory_tracking_dynamics_coupled'), '../../../../src/control_module/mpc_trajectory_tracking_dynamics_coupled/config', 'mpc_parameters_configuration_dynamics_coupled.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='mpc_trajectory_tracking_dynamics_coupled',
            executable='mpc_trajectory_tracking_dynamics_coupled_node',
            name='mpc_trajectory_tracking_dynamics_coupled_node',
            parameters=[mpc_parameters_configuration],
            output='screen',
        )
    ])
