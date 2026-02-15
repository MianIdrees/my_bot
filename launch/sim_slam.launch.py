"""
sim_slam.launch.py — Gazebo Simulation + SLAM Toolbox

Starts the full Gazebo simulation stack (sim_gazebo.launch.py) and adds
SLAM Toolbox for mapping. Drive the robot with teleop to build a map.

Usage:
  ros2 launch my_bot sim_slam.launch.py
  ros2 launch my_bot sim_slam.launch.py world:=simple_room.sdf

To teleoperate (in another terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

To save map:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World SDF file name (in worlds/ directory)',
    )

    world = LaunchConfiguration('world')

    # --- 1. Include full Gazebo simulation stack ---
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items(),
    )

    # --- 2. SLAM Toolbox (online async, sim config) ---
    # Delayed by 5 seconds to ensure Gazebo /clock and TF are ready.
    # Without this delay, SLAM starts with time=0 and all TF lookups fail.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_path, 'config', 'mapper_params_online_async_sim.yaml'),
            {'use_sim_time': True},
        ],
    )

    delayed_slam = TimerAction(
        period=5.0,
        actions=[slam_toolbox],
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

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz],
    )

    return LaunchDescription([
        world_arg,
        sim_gazebo,
        delayed_slam,
        delayed_rviz,
    ])
