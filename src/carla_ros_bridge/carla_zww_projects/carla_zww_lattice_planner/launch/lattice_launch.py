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
import sys

def generate_launch_description():

    # print(sys.argv[0])
    # print(__file__)
    print("****************")

    print(get_package_share_directory('carla_zww_lattice_planner'))
    print(os.getcwd())
    
    lqr_parameters_configuration = os.path.join(os.getcwd(), 'src/ros-bridge/carla_zww_projects/carla_zww_lattice_planner/config', 'lattice_parameters_configuration.yaml')

    rviz_config_dir = os.path.join(os.getcwd(), 'src/ros-bridge/carla_zww_projects/carla_zww_lattice_planner/rviz', 'lattice_vis.rviz')
    print(lqr_parameters_configuration)

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='carla_zww_lattice_planner',
            executable='lattice_mpc_lateral_longitudinal',
            name='lattice_mpc_lateral_longitudinal',
            parameters=[lqr_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
    ])
