import os
import yaml

import ament_index_python.packages

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch.substitutions
import launch_ros.actions
# from launch_ros.actions import DeclareLaunchArgument

# 启动的节点名称包括：
# 车辆模型的可视化，全局路径的可视化，惯导的实时解析，rviz可视化工具， 底盘的反馈信号实时解析， 安全机制节点， 键盘监控
# 实际的测试结果是如果太多节点在一起启动的话，会出现非常大的延时

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('visualization'), 'urdf', 'host_vehicle.urdf')

    global_map_path_config = os.path.join(get_package_share_directory('launch_manager'), 'config', 'map_path.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory('visualization'), 'config', 'car_disp.rviz')

    return LaunchDescription([
        
        # This package allows you to publish the state of a robot to tf2
        Node(package='robot_state_publisher', 
             executable='robot_state_publisher',
             output='screen',
             arguments=[urdf]),

        Node(package='visualization',
             executable='host_vehicle_visualization',
             output='screen',
             name='host_vehicle_visualization'),

        Node(package='visualization',
             executable='global_path_visualization',
             output='screen',
             name='global_path_visualization',
             parameters=[global_map_path_config]),

        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
    ])
