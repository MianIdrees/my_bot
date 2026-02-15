"""
sim_gazebo.launch.py — Launch Gazebo Harmonic with the robot

Starts:
  1. Gazebo Harmonic (gz-sim) with a selected world
  2. Robot State Publisher (sim URDF with Gazebo plugins)
  3. Spawns the robot model into Gazebo
  4. ros_gz_bridge to bridge /scan (gpu_lidar → LaserScan)
  5. ros2_control controllers (diff_cont + joint_broad)

Usage:
  ros2 launch my_bot sim_gazebo.launch.py
  ros2 launch my_bot sim_gazebo.launch.py world:=simple_room.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from ros_gz_sim.actions import GzServer

import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World SDF file name (in worlds/ directory)',
    )

    world = LaunchConfiguration('world')

    # --- 1. Process robot URDF (simulation version) ---
    xacro_file = os.path.join(pkg_path, 'description', 'robot_sim.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
        }],
    )

    # --- 2. Start Gazebo Harmonic server ---
    gz_server = GzServer(
        world_sdf_file=PathJoinSubstitution([
            pkg_path, 'worlds', world,
        ]),
    )

    # --- 3. Start Gazebo GUI (gz sim gui) ---
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    # --- 4. Spawn robot into Gazebo ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    # --- 5. Bridge: Gazebo topics ↔ ROS 2 topics ---
    # Bridge the gpu_lidar sensor from Gazebo to ROS /scan
    # Bridge /clock for use_sim_time
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # --- 6. Spawn ros2_control controllers (after robot is loaded) ---
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen',
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen',
    )

    # Delay controller spawning to ensure Gazebo + ros2_control plugin is ready
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[diff_drive_spawner, joint_broad_spawner],
    )

    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gz_server,
        gz_gui,
        spawn_robot,
        bridge,
        delayed_controllers,
    ])
