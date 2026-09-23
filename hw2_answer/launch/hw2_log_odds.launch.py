"""Run the Homework 2 log-odds mapper with the provided ROS 2 bag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('hw2_answer')
    rviz_config = os.path.join(package_share, 'rviz', 'mapping.rviz')
    bag_file = LaunchConfiguration('bag_file')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag_file', default_value='/hw2_data'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        Node(
            package='hw2_answer',
            executable='log_odds_mapping',
            name='log_odds_mapper',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '--rate', rate],
            output='screen',
        ),
    ])
