"""
Complete robot launch file for Orange Pi 5 hardware deployment.

This launch file brings up all necessary nodes for running the physical robot:
  1. Robot state publisher (publishes URDF and TF tree)
  2. Joint state publisher (publishes static joint states)
  3. RPLidar C1 node (laser scanner)
  4. Serial motor controller (Arduino interface)

Usage:
  ros2 launch my_bot launch_robot.launch.py

Arguments:
  serial_port_lidar:=/dev/ttyUSB0   - RPLidar C1 serial port
  serial_port_arduino:=/dev/ttyACM0 - Arduino Nano serial port
  lidar_baudrate:=460800            - RPLidar baud rate
  arduino_baudrate:=115200          - Arduino baud rate
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'my_bot'
    pkg_path = get_package_share_directory(package_name)
    
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    
    serial_port_lidar_arg = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar C1 (e.g., /dev/ttyUSB0)'
    )
    
    serial_port_arduino_arg = DeclareLaunchArgument(
        'serial_port_arduino',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino Nano (e.g., /dev/ttyACM0)'
    )
    
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='460800',
        description='Baud rate for RPLidar C1 (460800 for C1 model)'
    )
    
    arduino_baudrate_arg = DeclareLaunchArgument(
        'arduino_baudrate',
        default_value='115200',
        description='Baud rate for Arduino serial communication'
    )
    
    # ========================================================================
    # 1. Robot State Publisher
    # ========================================================================
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # ========================================================================
    # 2. Joint State Publisher (for visualization)
    # ========================================================================
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # ========================================================================
    # 3. RPLidar C1 Node (Laser Scanner)
    # ========================================================================
    
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port_lidar'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )
    
    # ========================================================================
    # 4. Serial Motor Controller (Arduino Interface)
    # ========================================================================
    
    motor_controller = Node(
        package='my_bot',
        executable='serial_motor_controller.py',
        name='serial_motor_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port_arduino'),
            'baud_rate': LaunchConfiguration('arduino_baudrate'),
            'timeout': 1.0,
            'reconnect_interval': 5.0,
        }]
    )
    
    # ========================================================================
    # Build Launch Description
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        serial_port_lidar_arg,
        serial_port_arduino_arg,
        lidar_baudrate_arg,
        arduino_baudrate_arg,
        
        # Nodes
        rsp,
        joint_state_publisher,
        rplidar_node,
        motor_controller,
    ])
