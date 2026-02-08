# my_bot

A differential drive mobile robot built with ROS 2 Jazzy and simulated in Gazebo Harmonic. The robot features a 2D lidar (RPLidar), two drive wheels with a caster wheel, and supports keyboard teleoperation.

## Features

- **Differential Drive** — Two-wheeled robot with caster wheel, controlled via `cmd_vel` twist messages
- **2D Lidar** — 360-degree laser scanner (simulated as GPU lidar in Gazebo, RPLidar A1/A2 on hardware)
- **Gazebo Harmonic Simulation** — Full physics simulation with sensor plugins, diff-drive control, and joint state publishing
- **RViz2 Visualization** — Real-time robot model and sensor data visualization
- **Keyboard Teleoperation** — Drive the robot using `teleop_twist_keyboard`
- **ROS 2 / Gazebo Bridge** — Clock, odometry, transforms, joint states, and laser scan all bridged between Gazebo and ROS 2

## Project Structure

```
my_bot/
├── config/                  # RViz config and parameter files
├── description/             # Robot URDF/Xacro model files
│   ├── robot.urdf.xacro     # Main robot description (includes all xacro files)
│   ├── robot_core.xacro     # Chassis, wheels, caster, materials
│   ├── lidar.xacro          # Lidar sensor link, joint, and Gazebo plugin
│   ├── gazebo_control.xacro # Diff-drive and joint state publisher plugins
│   └── inertial_macros.xacro# Inertia calculation macros
├── launch/
│   ├── launch_sim.launch.py # Launch simulation (Gazebo + RViz + bridges)
│   ├── rsp.launch.py        # Robot state publisher
│   └── rplidar.launch.py    # Physical RPLidar hardware launch
├── worlds/                  # Gazebo world files
├── CMakeLists.txt
└── package.xml
```

## Dependencies

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/)
- `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `xacro`
- `teleop_twist_keyboard` (for keyboard control)
- `rplidar_ros` (for physical lidar only)

## Build

```bash
cd ~/Robot_simulation
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Simulation (Gazebo + RViz)

```bash
ros2 launch my_bot launch_sim.launch.py
```

### Keyboard Control

In a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Launch Physical RPLidar

```bash
ros2 launch my_bot rplidar.launch.py
# Or specify the serial port:
ros2 launch my_bot rplidar.launch.py serial_port:=/dev/ttyUSB0
```

## Robot Specs

| Parameter | Value |
|---|---|
| Chassis | 0.3 x 0.3 x 0.15 m |
| Wheel Radius | 0.05 m |
| Wheel Separation | 0.35 m |
| Caster Wheel Radius | 0.05 m |
| Lidar Range | 0.3 – 12 m, 360 samples |
| Max Linear Velocity | 0.5 m/s |
| Max Angular Velocity | 1.0 rad/s |

## License

MIT License — see [LICENSE.md](LICENSE.md)