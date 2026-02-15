"""
sim_gazebo.launch.py — Launch Gazebo Harmonic with the robot

Starts:
  1. Gazebo Harmonic (gz-sim) with a selected world
  2. Robot State Publisher (sim URDF with Gazebo plugins)
  3. Spawns the robot model into Gazebo
  4. ros_gz_bridge for /cmd_vel, /odom, /scan, /clock
  5. odom_to_tf node (converts /odom → odom→base_link TF)

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
    # NOTE: /tf and /joint_states bridges are omitted because
    #   gz.msgs.Pose_V → tf2_msgs/TFMessage conversion is unreliable.
    #   The odom_to_tf node below handles odom→base_link TF instead.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Teleop commands: ROS → Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry: Gazebo → ROS (used by odom_to_tf node)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR scan: Gazebo → ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Clock for use_sim_time: Gazebo → ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states: Gazebo → ROS (for wheel TF in robot_state_publisher)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    )

    # --- 6. odom→base_link TF from /odom topic ---
    # More reliable than bridging /tf directly from Gazebo Harmonic
    odom_to_tf = Node(
        package='my_bot',
        executable='odom_to_tf_node.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gz_server,
        gz_gui,
        spawn_robot,
        bridge,
        odom_to_tf,
    ])
