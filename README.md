# my_bot — Gazebo Simulation Branch

Differential drive robot simulation using **ROS 2 Jazzy** and **Gazebo Harmonic (gz-sim)**.

This branch mirrors the exact robot geometry from the hardware branch
(`feature/lattepanda-alpha-daniel`) so that SLAM maps and Nav2 parameters
transfer directly to the physical robot.

## Robot Specifications

| Parameter | Value |
|---|---|
| Length x Width x Height | 437 x 212 x 89 mm |
| Weight | 2.4 kg |
| Wheel diameter | 69 mm |
| Wheel separation (center-to-center) | 181 mm |
| Drive type | Differential (2 rear + 1 front caster) |
| LiDAR | Simulated RPLidar C1 (360 deg, 8 m range) |

## Quick Start

### 1. Build

```bash
cd ~/Robot_simulation
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo + SLAM (mapping)

```bash
ros2 launch my_bot sim_slam.launch.py world:=simple_room.sdf
```

Teleoperate the robot to build a map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 3. Launch Gazebo + Nav2 (autonomous navigation)

```bash
ros2 launch my_bot sim_nav2.launch.py map:=~/my_map.yaml world:=simple_room.sdf

""

# Kill any running simulation
pkill -f "ros2 launch" ; pkill -f "gz sim" ; pkill -f gzserver

# Launch Nav2 with your saved map
cd ~/Robot_simulation
source install/setup.bash
ros2 launch my_bot sim_nav2.launch.py map:=/home/idrees/Robot_simulation/my_map.yaml world:=simple_room.sdf

""

```

In RViz:
1. Set initial pose with **2D Pose Estimate**
2. Send goals with **Nav2 Goal**

### 4. Launch Gazebo only (no SLAM/Nav2)

```bash
ros2 launch my_bot sim_gazebo.launch.py world:=simple_room.sdf
```

## Launch Files

| File | Description |
|---|---|
| `sim_gazebo.launch.py` | Gazebo + robot spawn + ros2_control + LiDAR bridge |
| `sim_slam.launch.py` | sim_gazebo + SLAM Toolbox + RViz |
| `sim_nav2.launch.py` | sim_gazebo + Nav2 stack + RViz |

## Worlds

| File | Description |
|---|---|
| `empty_world.sdf` | Open ground plane |
| `simple_room.sdf` | 5x5 m room with walls and obstacles |

## Branch Relationship

```
main
 └── feature/rpi-hardware-deployment
      └── feature/lattepanda-alpha-daniel       (hardware)
           └── feature/lattepanda-alpha-daniel-sim  (this branch)
```
