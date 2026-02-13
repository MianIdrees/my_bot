"""
bringup_hardware.launch.py — Full Robot Bringup on Real Hardware (Orange Pi 5)

Launches ALL nodes needed for the real robot:
  1. Robot State Publisher (URDF → TF static tree)
  2. RPLidar C1 driver (laser scans → /scan)
  3. Differential Drive serial node (Arduino bridge → /odom, /joint_states, cmd_vel)

Usage (on Orange Pi 5):
  ros2 launch my_bot bringup_hardware.launch.py

Optional arguments:
  lidar_port:=/dev/rplidar      Serial port for RPLidar
  arduino_port:=/dev/arduino    Serial port for Arduino Uno
  ticks_per_rev:=990.0          JGB37-520 encoder ticks per wheel revolution (11 PPR × 90:1)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')

    # ========================== ARGUMENTS ==========================

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/rplidar',
        description='Serial port for RPLidar C1',
    )
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port', default_value='/dev/arduino',
        description='Serial port for Arduino Nano motor controller',
    )
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev', default_value='990.0',
        description='JGB37-520 encoder ticks per full wheel revolution (11 PPR x 90:1 gear)',
    )

    # ========================== ROBOT STATE PUBLISHER ==========================

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp_hardware.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # ========================== RPLIDAR C1 ==========================

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 460800,       # RPLidar C1 default baud rate
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': False,
        }],
    )

    # ========================== DIFF DRIVE (Arduino Bridge) ==========================

    diff_drive_node = Node(
        package='my_bot',
        executable='diff_drive_node.py',
        name='diff_drive_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_port'),
            'baud_rate': 115200,
            'wheel_separation': 0.35,
            'wheel_radius': 0.0325,
            'ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
            'max_motor_speed': 0.37,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'publish_rate': 20.0,
        }],
    )

    return LaunchDescription([
        lidar_port_arg,
        arduino_port_arg,
        ticks_per_rev_arg,
        rsp,
        rplidar_node,
        diff_drive_node,
    ])
