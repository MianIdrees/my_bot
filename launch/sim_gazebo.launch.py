"""
sim_gazebo.launch.py — Launch Gazebo Harmonic with the robot

Starts:
  1. Gazebo Harmonic (gz-sim) with a selected world
  2. Robot State Publisher (sim URDF with Gazebo plugins)
  3. Spawns the robot model into Gazebo
  4. ros_gz_bridge for /cmd_vel, /odom, /scan, /tf, /joint_states, /clock

Usage:
  ros2 launch my_bot sim_gazebo.launch.py
  ros2 launch my_bot sim_gazebo.launch.py world:=simple_room.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
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
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Teleop commands: ROS → Gazebo  (] = ROS-to-Gazebo direction)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry: Gazebo → ROS  ([ = Gazebo-to-ROS direction)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR scan: Gazebo → ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # TF (odom→base_link from DiffDrive): Gazebo → ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states (wheel positions): Gazebo → ROS
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Clock for use_sim_time: Gazebo → ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gz_server,
        gz_gui,
        spawn_robot,
        bridge,
    ])
