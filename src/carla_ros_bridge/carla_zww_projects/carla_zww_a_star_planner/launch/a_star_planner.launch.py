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
    # print("****************")

    # print(get_package_share_directory('carla_zww_a_star_planner'))
    # print(os.getcwd())
    
    # lqr_parameters_configuration = os.path.join(os.getcwd(), 'src/carla_ros_bridge/carla_zww_projects/carla_zww_a_star_planner/config', 'a_star_parameters_configuration.yaml')

    rviz_config_dir = os.path.join(os.getcwd(), 'src/carla_ros_bridge/carla_zww_projects/carla_zww_a_star_planner/rviz', 'a_star_vis.rviz')
    # print(lqr_parameters_configuration)

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
        Node(
            package='carla_zww_a_star_planner',
            executable='a_star_planner',
            name='a_star_planner',
            # parameters=[lqr_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        Node(
            package='carla_zww_a_star_planner',
            executable='random_map_generator',
            name='random_map_generator',
            # parameters=[lqr_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
    ])
