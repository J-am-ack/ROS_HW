"""Run the Homework 2 endpoint-voting map builder with a ROS 2 bag."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('hw2_answer')
    rviz_config = os.path.join(package_share, 'rviz', 'mapping.rviz')
    bag_file = LaunchConfiguration('bag_file')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_file',
            default_value='/hw2_data',
            description='Path to the ROS 2 bag directory.',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='ROS 2 bag playback rate.',
        ),
        Node(
            package='hw2_answer',
            executable='simple_mapping',
            name='simple_voting_mapper',
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
