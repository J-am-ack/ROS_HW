"""Run Cartographer on the Homework 2 bag without changing provided packages."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('hw2_answer')
    config_directory = os.path.join(package_share, 'config')
    rviz_config = os.path.join(package_share, 'rviz', 'mapping.rviz')
    bag_file = LaunchConfiguration('bag_file')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag_file', default_value='/hw2_data'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['-0.05', '0', '0.05', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('scan', '/urg_node/scan'),
                ('imu', '/encoder_imu_node/cartoimu'),
                ('odom', '/encoder_imu_node/odom'),
            ],
            arguments=[
                '-configuration_directory', config_directory,
                '-configuration_basename', 'hw2_2d.lua',
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '--rate', rate, '--clock'],
            output='screen',
        ),
    ])
