"""
navigation_launch.py — Autonomous Navigation on a Saved Map

Launches the Nav2 stack on top of an already-running simulation.
  - map_server + AMCL for localization on the saved map
  - Full Nav2 navigation: planner, controller, behaviors, bt_navigator
  - Does NOT open RViz (launch_sim.launch.py already opens it)

Usage:
  Terminal 1:  ros2 launch my_bot launch_sim.launch.py
               (wait ~15 sec for Gazebo + controllers to start)
  Terminal 2:  ros2 launch my_bot navigation_launch.py

Then in the RViz window (already open from launch_sim):
  1. Click "2D Pose Estimate" → set robot's initial position on the map
  2. Click "2D Goal Pose"     → click a destination → robot navigates!
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_path = get_package_share_directory('my_bot')

    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start the Nav2 lifecycle nodes',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'config', 'my_map_save.yaml'),
        description='Full path to the map yaml file to load',
    )

    # ---------- Rewrite params ----------
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Ensure line-buffered logging
    stdout_linebuf = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # =====================================================================
    # Localization  (map_server + AMCL)
    # =====================================================================
    localization_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    configured_params,
                    {'yaml_filename': map_yaml},
                ],
            ),

            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']},
                ],
            ),
        ]
    )

    # =====================================================================
    # Navigation  (controller, planner, behaviors, smoother, bt_navigator)
    # =====================================================================
    navigation_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel')],
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                    ]},
                ],
            ),
        ]
    )

    # Delay navigation so localization is ready first
    delayed_navigation = TimerAction(
        period=5.0,
        actions=[navigation_nodes],
    )

    # =====================================================================
    # Assemble
    # =====================================================================
    return LaunchDescription([
        stdout_linebuf,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        localization_nodes,
        delayed_navigation,
    ])
