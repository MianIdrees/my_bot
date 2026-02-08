import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file
    package_name = 'my_bot'
    pkg_path = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include Gazebo Harmonic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r ', os.path.join(pkg_path, 'worlds', 'obstacle_world_2.sdf')]}.items()
    )

    # Spawn the robot in Gazebo (delayed to ensure robot_description is published)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_bot',
            '-z', '0.05'
        ],
        output='screen'
    )

    # Delay spawning so robot_state_publisher publishes the description first
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    # --- ros2_control: spawn controllers AFTER the robot is spawned ---

    # Joint State Broadcaster (publishes /joint_states)
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen',
    )

    # Diff Drive Controller (subscribes to /diff_cont/cmd_vel, publishes /odom & /tf)
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen',
    )

    # Twist relay: converts /cmd_vel (Twist) → /diff_cont/cmd_vel (TwistStamped)
    # so standard teleop tools work without remapping
    twist_relay = Node(
        package='my_bot',
        executable='twist_relay.py',
        name='twist_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Chain: after spawn completes → start joint_broad → then start diff_cont
    delayed_joint_broad = TimerAction(
        period=7.0,
        actions=[joint_broad_spawner],
    )

    delayed_diff_cont = TimerAction(
        period=9.0,
        actions=[diff_cont_spawner],
    )

    # Bridge Gazebo and ROS2 topics
    # NOTE: /cmd_vel, /odom, /joint_states, /tf are now handled by ros2_control
    #       so only bridge clock and sensor topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # RViz2 with sim time enabled
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch!
    return LaunchDescription([
        rsp,
        gazebo,
        bridge,
        delayed_spawn,
        delayed_joint_broad,
        delayed_diff_cont,
        twist_relay,
        rviz,
    ])
