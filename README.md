# my_bot

A differential drive mobile robot built with ROS 2 Jazzy and simulated in Gazebo Harmonic. The robot features a 2D lidar (RPLidar C1), two drive wheels with a caster wheel, and supports keyboard teleoperation.

**🚀 Ready for both simulation and real hardware deployment!**

> **Hardware Deployment:** See [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) for complete Orange Pi 5 setup guide with RPLidar C1, Arduino Nano motor controller, and distributed ROS 2 configuration.

## Features

- **Differential Drive** — Two-wheeled robot with caster wheel, controlled via `cmd_vel` twist messages
- **2D Lidar** — 360-degree laser scanner (simulated as GPU lidar in Gazebo, RPLidar C1 on hardware)
- **Gazebo Harmonic Simulation** — Full physics simulation with sensor plugins, diff-drive control, and joint state publishing
- **RViz2 Visualization** — Real-time robot model and sensor data visualization
- **Keyboard Teleoperation** — Drive the robot using `teleop_twist_keyboard`
- **ROS 2 / Gazebo Bridge** — Clock, odometry, transforms, joint states, and laser scan all bridged between Gazebo and ROS 2

## Project Structure

```
my_bot/
├── arduino/                 # Arduino motor controller firmware
│   ├── differential_drive_controller.ino  # Motor control sketch
│   └── README.md            # Arduino setup guide
├── config/                  # RViz config and parameter files
├── description/             # Robot URDF/Xacro model files
│   ├── robot.urdf.xacro     # Main robot description (includes all xacro files)
│   ├── robot_core.xacro     # Chassis, wheels, caster, materials
│   ├── lidar.xacro          # Lidar sensor link, joint, and Gazebo plugin
│   ├── gazebo_control.xacro # Diff-drive and joint state publisher plugins
│   └── inertial_macros.xacro# Inertia calculation macros
├── launch/
│   ├── launch_sim.launch.py       # Launch simulation (Gazebo + RViz + bridges)
│   ├── launch_robot.launch.py    # Launch physical robot (Orange Pi 5)
│   ├── launch_real_lidar.launch.py # Launch with real RPLidar C1
│   ├── rsp.launch.py              # Robot state publisher
│   └── rplidar.launch.py          # Physical RPLidar hardware launch
├── scripts/
│   └── serial_motor_controller.py # ROS 2 ↔ Arduino serial bridge
├── worlds/                  # Gazebo world files
├── HARDWARE_DEPLOYMENT.md   # 📘 Complete hardware deployment guide
├── CMakeLists.txt
└── package.xml
```

## Dependencies

### Simulation (Ubuntu 24.04 + ROS 2 Jazzy)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/)
- `ros_gz_sim`, `ros_gz_bridge`
- `robot_state_publisher`, `xacro`
- `teleop_twist_keyboard` (for keyboard control)

### Hardware (Orange Pi 5 + ROS 2 Humble)
- [ROS 2 Humble](https://docs.ros.org/en/humble/) (recommended for Orange Pi 5)
- `sllidar_ros2` (for RPLidar C1)
- `serial` Python package (for Arduino communication)
- `robot_state_publisher`, `joint_state_publisher`

See [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) for complete installation instructions.

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

### Launch Complete Physical Robot (Orange Pi 5)

```bash
# Launch all robot hardware: RPLidar C1 + Motor Controller + Robot State Publisher
ros2 launch my_bot launch_robot.launch.py

# Custom serial ports (if needed):
ros2 launch my_bot launch_robot.launch.py serial_port_lidar:=/dev/ttyUSB0 serial_port_arduino:=/dev/ttyACM0
```

> **Note:** For complete hardware setup instructions, see [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md)

## Robot Specs

| Parameter | Value |
|---|---|
| Chassis | 0.3 x 0.3 x 0.15 m |
| Wheel Radius | 0.05 m |
| Wheel Separation | 0.35 m |
| Caster Wheel Radius | 0.05 m |
| Lidar | RPLidar C1 (0.3 – 12 m, 360°) |
| Max Linear Velocity | 0.5 m/s |
| Max Angular Velocity | 1.0 rad/s |

## Hardware Deployment

Ready to deploy to real hardware? See the comprehensive guides:

- **[HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md)** — Complete Orange Pi 5 setup, Ubuntu installation, ROS 2 configuration, network setup, and distributed system architecture
- **[arduino/README.md](arduino/README.md)** — Arduino Nano motor controller setup, wiring diagrams, upload instructions, and testing procedures

### Quick Hardware Checklist

- [ ] Orange Pi 5 with Ubuntu 22.04 + ROS 2 Humble
- [ ] RPLidar C1 connected via USB
- [ ] Arduino Nano with motor controller firmware uploaded
- [ ] L298N motor driver wired to motors
- [ ] Network configured (static IPs on same subnet)
- [ ] ROS_DOMAIN_ID set to same value on both machines
- [ ] All launch files tested

## License

MIT License — see [LICENSE.md](LICENSE.md)