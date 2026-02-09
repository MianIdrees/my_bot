"""
Launch file for real RPLIDAR C1 hardware with robot visualization.

Brings up:
  1. robot_state_publisher  – publishes URDF TF tree (use_sim_time=false)
  2. joint_state_publisher  – publishes static joint states for visualization
  3. rplidar_ros node       – reads from the real RPLIDAR C1 on /dev/ttyUSB0
  4. rviz2                  – shows robot model + live LaserScan
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'
    pkg_path = get_package_share_directory(package_name)

    # ── Launch arguments ──────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar C1',
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Baud rate (460800 for C1)',
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='RPLidar scan mode (Standard, Express, Boost, Sensitivity)',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_path, 'config', 'real_lidar.rviz'),
        description='Full path to the RViz config file',
    )

    # ── 1. Robot State Publisher (real time, NOT sim time) ─────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # ── 2. Joint State Publisher (provides fixed joint states) ────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── 3. RPLIDAR C1 node (sllidar_ros2 – official Slamtec driver) ───
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser_frame',      # matches URDF link name
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
    )

    # ── 4. RViz2 ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': False}],
    )

    # Delay RViz a bit so TF tree is ready
    delayed_rviz = TimerAction(period=2.0, actions=[rviz])

    # ── Build launch description ──────────────────────────────────────
    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        scan_mode_arg,
        rviz_config_arg,
        rsp,
        joint_state_publisher,
        rplidar_node,
        delayed_rviz,
    ])
