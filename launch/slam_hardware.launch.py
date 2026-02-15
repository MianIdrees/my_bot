"""
slam_hardware.launch.py — SLAM on Real Hardware

Launches SLAM Toolbox configured for real robot:
  - use_sim_time: false
  - Uses /scan from RPLidar C1
  - Uses /odom from diff_drive_node

Prerequisites:
  ros2 launch my_bot bringup_hardware.launch.py

Usage (on LattePanda Alpha):
  ros2 launch my_bot slam_hardware.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_path = get_package_share_directory('my_bot')

    slam_params_file = os.path.join(
        pkg_path, 'config', 'mapper_params_online_async_hardware.yaml'
    )

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        namespace='',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (false for real hardware)',
        ),
        slam_toolbox,
        activate_event,
        configure_event,
    ])
