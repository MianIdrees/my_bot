"""
rviz_remote.launch.py — Remote Visualization from Desktop PC

Launches RViz2 on the desktop to visualize:
  - Laser scan from RPLidar
  - Map (from SLAM or map_server)
  - TF tree
  - Costmaps
  - Global + local path
  - Robot model

Prerequisites:
  1. Both machines on same WiFi
  2. Same ROS_DOMAIN_ID on both machines
  3. Robot bringup running on Orange Pi 5

Usage (on Desktop PC):
  ros2 launch my_bot rviz_remote.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')

    rviz_config = LaunchConfiguration('rviz_config')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_path, 'config', 'nav2_view.rviz'),
        description='Path to RViz config file',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        declare_rviz_config,
        rviz_node,
    ])
