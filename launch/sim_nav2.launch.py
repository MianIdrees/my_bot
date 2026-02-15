"""
sim_nav2.launch.py — Gazebo Simulation + Nav2 Autonomous Navigation

Starts the full Gazebo simulation stack (sim_gazebo.launch.py) and adds
the Nav2 stack for autonomous navigation using a pre-built map.

Prerequisites:
  - A saved map (from sim_slam.launch.py) as .yaml + .pgm files.

Usage:
  ros2 launch my_bot sim_nav2.launch.py map:=/path/to/my_map.yaml
  ros2 launch my_bot sim_nav2.launch.py map:=/path/to/my_map.yaml world:=simple_room.sdf

Steps after launch:
  1. In RViz, use "2D Pose Estimate" to set the initial robot pose on the map
  2. Use "Nav2 Goal" to command the robot to navigate to a target
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_room.sdf',
        description='World SDF file name (in worlds/ directory)',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map YAML file (required)',
    )

    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    # --- 1. Include full Gazebo simulation stack ---
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items(),
    )

    # --- 2. Nav2 bringup (full navigation stack) ---
    # Nav2 controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
    # Gazebo DiffDrive plugin listens on /cmd_vel, so we relay below.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py'),
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_path, 'config', 'nav2_params_sim.yaml'),
        }.items(),
    )

    # --- 3. RViz for visualization ---
    rviz_config = os.path.join(pkg_path, 'config', 'nav2_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        sim_gazebo,
        nav2_bringup,
        rviz,
    ])
