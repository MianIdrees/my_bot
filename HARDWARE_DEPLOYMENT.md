# 🤖 Hardware Deployment Guide — Real Robot on Orange Pi 5

Complete guide to deploy the ROS2 + Nav2 autonomous navigation stack from simulation to real hardware.

## 📋 Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Phase 2 — Orange Pi 5 Setup](#phase-2--orange-pi-5-setup)
3. [Phase 3 — Hardware Integration](#phase-3--hardware-integration)
4. [Phase 4 — SLAM Mapping](#phase-4--slam-mapping)
5. [Phase 5 — Autonomous Navigation](#phase-5--autonomous-navigation)
6. [Phase 6 — Network Configuration](#phase-6--network-configuration)
7. [Phase 7 — Remote Monitoring](#phase-7--remote-monitoring)
8. [Troubleshooting Guide](#troubleshooting-guide)
9. [Debug Checklist](#debug-checklist)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    DESKTOP PC (Ubuntu 24.04)                 │
│  ┌──────────┐                                               │
│  │  RViz2   │ ← Visualizes /scan, /map, /tf, costmaps      │
│  └──────────┘                                               │
│  Sends: 2D Pose Estimate, 2D Goal Pose                      │
│  Views: Path planning, costmaps, robot position              │
└───────────────────────┬─────────────────────────────────────┘
                        │ WiFi (same network)
                        │ ROS_DOMAIN_ID=42
┌───────────────────────┴─────────────────────────────────────┐
│                 ORANGE PI 5 (Ubuntu 22.04)                    │
│                                                               │
│  ┌─────────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │ robot_state_pub │  │  slam_toolbox │  │   Nav2 stack   │  │
│  │ (URDF→TF)       │  │  (mapping)    │  │ (navigation)   │  │
│  └─────────────────┘  └──────────────┘  └────────────────┘  │
│                                                               │
│  ┌─────────────────┐  ┌──────────────────────────────────┐  │
│  │ rplidar_node    │  │    diff_drive_node.py             │  │
│  │ /scan topic     │  │    /cmd_vel → Arduino PWM         │  │
│  │ USB: /dev/rplidar│  │    Arduino enc → /odom + TF       │  │
│  └────────┬────────┘  └──────────────┬───────────────────┘  │
│           │ USB                      │ USB                    │
└───────────┼──────────────────────────┼───────────────────────┘
            │                          │
     ┌──────┴──────┐          ┌────────┴────────┐
     │ RPLidar C1  │          │  Arduino Nano   │
     │             │          │  ┌────────────┐ │
     └─────────────┘          │  │ L298 Driver │ │
                              │  │ 2x DC Motor │ │
                              │  │ 2x Encoder  │ │
                              │  └────────────┘ │
                              └─────────────────┘
```

### TF Tree Structure
```
map → odom → base_link → chassis → laser_frame
                   ├─→ left_wheel
                   ├─→ right_wheel
                   └─→ base_footprint
```

- **map → odom**: Published by SLAM Toolbox (during mapping) or AMCL (during navigation)
- **odom → base_link**: Published by diff_drive_node.py (from encoder odometry)
- **base_link → chassis, wheels, laser_frame**: Published by robot_state_publisher (from URDF)

---

## Phase 2 — Orange Pi 5 Setup

### Task 2.1 — System Verification

SSH into Orange Pi 5 and verify the system:

```bash
# Connect via SSH
ssh <username>@<orange-pi-ip>

# Check Ubuntu version (should be 22.04)
lsb_release -a

# Check architecture (should be aarch64)
uname -m

# Check kernel
uname -r

# Check available RAM
free -h

# Check CPU info
cat /proc/cpuinfo | head -20

# Check disk space
df -h

# Check USB devices (should see RPLidar and Arduino when connected)
lsusb

# Check available serial ports
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

### Task 2.2 — Install ROS2 Humble + Dependencies

```bash
# ============================================================
# 1. Install ROS2 Humble (Ubuntu 22.04 / aarch64)
# ============================================================

# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 GPG key
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (base — no desktop GUI needed on Orange Pi)
sudo apt update
sudo apt install -y ros-humble-ros-base python3-argcomplete

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version


# ============================================================
# 2. Install Build Tools
# ============================================================

sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip


# ============================================================
# 3. Install Navigation2
# ============================================================

sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common


# ============================================================
# 4. Install SLAM Toolbox
# ============================================================

sudo apt install -y ros-humble-slam-toolbox


# ============================================================
# 5. Install RPLidar ROS2 Driver
# ============================================================

# Option A: From apt (if available)
sudo apt install -y ros-humble-rplidar-ros

# Option B: If not available from apt, build from source
# mkdir -p ~/rplidar_ws/src
# cd ~/rplidar_ws/src
# git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
# cd ~/rplidar_ws
# colcon build --symlink-install
# source install/setup.bash


# ============================================================
# 6. Install ros2_control + Controllers
# ============================================================

sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster


# ============================================================
# 7. Install TF2 and Transform Tools
# ============================================================

sudo apt install -y \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations


# ============================================================
# 8. Install Robot Description Tools
# ============================================================

sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro


# ============================================================
# 9. Install Serial Communication
# ============================================================

sudo apt install -y python3-serial
pip3 install pyserial


# ============================================================
# 10. Install CycloneDDS (for multi-machine networking)
# ============================================================

sudo apt install -y ros-humble-rmw-cyclonedds-cpp


# ============================================================
# 11. Initialize rosdep
# ============================================================

sudo rosdep init 2>/dev/null || true
rosdep update


# ============================================================
# 12. Verify all installations
# ============================================================

echo "=== Verification ==="
ros2 pkg list | grep -E "nav2|slam_toolbox|rplidar|tf2_ros|robot_state_publisher|xacro" | sort
echo "===================="
```

### Task 2.3 — Clone Repository and Build

```bash
# ============================================================
# Clone the repository on Orange Pi 5
# ============================================================

mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/MianIdrees/my_bot.git
cd my_bot
git checkout feature/rpi-hardware-deployment

# ============================================================
# Install package dependencies via rosdep
# ============================================================

cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# ============================================================
# Build the workspace
# ============================================================

cd ~/robot_ws
colcon build --symlink-install --packages-select my_bot

# ============================================================
# Source the workspace (add to .bashrc)
# ============================================================

echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/robot_ws/install/setup.bash

# ============================================================
# Verify the package
# ============================================================

ros2 pkg list | grep my_bot
ros2 launch my_bot --show-args bringup_hardware.launch.py
```

---

## Phase 3 — Hardware Integration

### Task 3.1 — RPLidar C1 Setup

```bash
# ============================================================
# 1. Detect RPLidar device
# ============================================================

# Plug in the RPLidar C1 via USB
lsusb
# Look for "Silicon Labs CP210x" or similar

ls -la /dev/ttyUSB*
# Should show /dev/ttyUSB0 (if only device connected)

# ============================================================
# 2. Set permissions
# ============================================================

# Temporary (for testing):
sudo chmod 666 /dev/ttyUSB0

# Permanent (install udev rules):
sudo cp ~/robot_ws/src/my_bot/config/99-robot-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify symlinks:
ls -la /dev/rplidar /dev/arduino

# ============================================================
# 3. Test RPLidar node standalone
# ============================================================

ros2 launch my_bot rplidar.launch.py serial_port:=/dev/rplidar

# In another terminal:
ros2 topic list | grep scan
ros2 topic hz /scan          # Should show ~5-10 Hz
ros2 topic echo /scan --once  # See one scan message
```

### Task 3.2 — Arduino Nano Firmware

1. Open `firmware/motor_controller.ino` in the Arduino IDE
2. **Verify pin assignments match your wiring** (see comments in the code)
3. Select Board: "Arduino Nano"
4. Select Processor: "ATmega328P" (or "ATmega328P (Old Bootloader)")
5. Select Port
6. Upload

**Testing the Arduino (from Orange Pi terminal):**

```bash
# Open serial monitor
sudo apt install -y minicom
minicom -D /dev/arduino -b 115200

# You should see:
#   Arduino motor controller ready
#   e 0 0           (encoder data every 50ms)

# Type commands (press Enter after each):
#   m 100 100       (both motors forward at ~40% speed)
#   m -100 -100     (both motors backward)
#   m 0 0           (stop)
#   r               (reset encoder counts)

# Exit minicom: Ctrl+A then X
```

### Task 3.3 — Diff Drive Node Testing

```bash
# Test diff_drive_node standalone
ros2 run my_bot diff_drive_node.py --ros-args \
  -p serial_port:=/dev/arduino \
  -p ticks_per_rev:=1320.0

# In another terminal, send a velocity command:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Check odometry is being published:
ros2 topic echo /odom --once
ros2 topic hz /odom

# Check joint states:
ros2 topic echo /joint_states --once
```

### Task 3.4 — TF Tree Verification

```bash
# Launch full bringup
ros2 launch my_bot bringup_hardware.launch.py \
  lidar_port:=/dev/rplidar \
  arduino_port:=/dev/arduino

# In another terminal, check TF:
ros2 run tf2_tools view_frames

# This generates frames.pdf — transfer to desktop to view:
# scp orangepi@<ip>:~/frames.pdf .

# Or check TF in terminal:
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame

# Expected TF tree:
#   odom → base_link → chassis → laser_frame
#                    → left_wheel
#                    → right_wheel
#                    → base_footprint
```

---

## Phase 4 — SLAM Mapping

### Step-by-step mapping procedure:

```bash
# ============================================================
# ON ORANGE PI 5
# ============================================================

# Terminal 1: Robot bringup
ros2 launch my_bot bringup_hardware.launch.py \
  lidar_port:=/dev/rplidar \
  arduino_port:=/dev/arduino

# Terminal 2: SLAM
ros2 launch my_bot slam_hardware.launch.py

# ============================================================
# ON DESKTOP PC
# ============================================================

# Terminal 1: RViz visualization
ros2 launch my_bot rviz_remote.launch.py

# Terminal 2: Teleoperation (drive the robot to build the map)
sudo apt install -y ros-humble-teleop-twist-keyboard  # or ros-jazzy-...
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Drive slowly around the environment to build the map.
# In RViz, you should see the map building in real-time.

# ============================================================
# SAVE THE MAP (when mapping is complete)
# ============================================================

# On Orange Pi 5 (or desktop — both work):
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_real_map

# This creates:
#   ~/maps/my_real_map.yaml
#   ~/maps/my_real_map.pgm

# Copy map to the robot workspace config:
cp ~/maps/my_real_map.yaml ~/robot_ws/src/my_bot/config/
cp ~/maps/my_real_map.pgm  ~/robot_ws/src/my_bot/config/
```

---

## Phase 5 — Autonomous Navigation

```bash
# ============================================================
# ON ORANGE PI 5
# ============================================================

# Terminal 1: Robot bringup (if not already running)
ros2 launch my_bot bringup_hardware.launch.py \
  lidar_port:=/dev/rplidar \
  arduino_port:=/dev/arduino

# Terminal 2: Navigation stack
ros2 launch my_bot navigation_hardware.launch.py \
  map:=/home/<user>/robot_ws/src/my_bot/config/my_real_map.yaml

# ============================================================
# ON DESKTOP PC
# ============================================================

# Terminal 1: RViz
ros2 launch my_bot rviz_remote.launch.py

# In RViz:
# 1. Click "2D Pose Estimate" → click on robot's position on map
#    (this initializes AMCL localization)
# 2. Wait for AMCL particle cloud to converge
# 3. Click "2D Goal Pose" → click destination on map
# 4. Robot navigates autonomously!
```

---

## Phase 6 — Network Configuration

### On BOTH machines (Orange Pi 5 AND Desktop PC):

```bash
# ============================================================
# Option A: Source the setup script
# ============================================================
source ~/robot_ws/src/my_bot/scripts/setup_network.sh

# ============================================================
# Option B: Add to .bashrc (permanent)
# ============================================================
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# ============================================================
# Verify networking
# ============================================================

# On Orange Pi 5, run a test publisher:
ros2 topic pub /test std_msgs/msg/String "data: 'hello from robot'" --once

# On Desktop PC, listen:
ros2 topic list    # Should show /test and any robot topics
ros2 topic echo /test

# If topics are not visible across machines:
# 1. Check both are on the same WiFi network
# 2. Check ROS_DOMAIN_ID matches on both
# 3. Check firewall: sudo ufw allow 7400:7500/udp
# 4. Try: ros2 multicast receive (tests DDS multicast)
```

### Install CycloneDDS on Desktop (Ubuntu 24.04):

```bash
# If using ROS2 Jazzy on desktop:
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

# If using ROS2 Humble on desktop:
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

> **IMPORTANT**: Both machines must use the same DDS implementation. CycloneDDS is recommended for multi-machine setups.

---

## Phase 7 — Remote Monitoring

### Desktop PC Setup:

```bash
# Install my_bot package on desktop too (for launch files and RViz config):
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/MianIdrees/my_bot.git
cd my_bot
git checkout feature/rpi-hardware-deployment
cd ~/robot_ws
colcon build --symlink-install --packages-select my_bot
source ~/robot_ws/install/setup.bash

# Launch RViz with pre-configured display:
ros2 launch my_bot rviz_remote.launch.py
```

### What you should see in RViz:

| Display | Topic | Source |
|---------|-------|--------|
| LaserScan | /scan | RPLidar C1 |
| Map | /map | SLAM or map_server |
| TF | /tf, /tf_static | All nodes |
| RobotModel | /robot_description | robot_state_publisher |
| Local Costmap | /local_costmap/costmap | Nav2 |
| Global Costmap | /global_costmap/costmap | Nav2 |
| Global Path | /plan | Nav2 planner |
| Local Path | /local_plan | Nav2 controller |
| Particle Cloud | /particle_cloud | AMCL |

---

## Troubleshooting Guide

### Serial Port Issues

```bash
# Problem: Permission denied on /dev/ttyUSB*
sudo usermod -aG dialout $USER
# Log out and back in, or:
newgrp dialout

# Problem: Wrong device assignment (USB0 vs USB1 swap)
# Solution: Use udev rules (config/99-robot-devices.rules)
# Then use /dev/rplidar and /dev/arduino instead

# Problem: Device not detected
lsusb                        # Check USB device is connected
dmesg | tail -20             # Check kernel messages
ls -la /dev/ttyUSB*          # List serial devices
```

### RPLidar Issues

```bash
# Problem: RPLidar not publishing /scan
ros2 topic hz /scan

# Check rplidar node status:
ros2 node list | grep rplidar

# Check if the motor is spinning (the RPLidar motor should spin)
# Try different baud rates:
# RPLidar C1 typically uses 460800 baud

# Check RPLidar health:
ros2 topic echo /scan --once  # Look for data
```

### Odometry Issues

```bash
# Problem: Robot moves but /odom doesn't change
# Check encoder wiring — channels A and B may be swapped
# Test: push robot forward, watch encoder ticks increase

# Check serial communication:
ros2 topic echo /odom --once

# Problem: Robot rotates instead of going straight
# Encoder direction may be inverted for one wheel
# Fix: swap IN1/IN2 or IN3/IN4 wires for that motor
# Or: negate ticks in Arduino code for that encoder
```

### TF Issues

```bash
# Problem: "Could not find a connection between 'map' and 'base_link'"
ros2 run tf2_tools view_frames  # Check for broken links

# Common causes:
# 1. SLAM or AMCL not running → no map→odom transform
# 2. diff_drive_node not running → no odom→base_link transform
# 3. robot_state_publisher not running → no base_link→laser_frame

# Check individual transforms:
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### Nav2 Issues

```bash
# Problem: Nav2 nodes stuck in "inactive" state
ros2 lifecycle list /controller_server
ros2 lifecycle set /controller_server activate

# Problem: "Transform timeout" warnings
# Increase transform_tolerance in nav2_params_hardware.yaml

# Problem: Robot doesn't move after sending goal
ros2 topic echo /cmd_vel          # Check if Nav2 publishes commands
ros2 topic echo /odom --once      # Check if odometry is working

# Problem: Robot oscillates or spins in place
# Reduce desired_linear_vel and rotate_to_heading_angular_vel
# in nav2_params_hardware.yaml
```

### Network Issues

```bash
# Problem: Desktop can't see Orange Pi topics
# 1. Check IP connectivity:
ping <orange-pi-ip>

# 2. Check ROS_DOMAIN_ID:
echo $ROS_DOMAIN_ID   # Must match on both machines

# 3. Check DDS multicast:
ros2 multicast receive   # On one machine
ros2 multicast send      # On the other

# 4. Check firewall:
sudo ufw status
sudo ufw allow 7400:7500/udp   # DDS ports
sudo ufw allow 11811/udp        # Discovery

# 5. Check RMW implementation:
echo $RMW_IMPLEMENTATION  # Should be same on both
```

---

## Debug Checklist

Use this checklist when things aren't working:

### Hardware
- [ ] Arduino Nano powered and connected via USB
- [ ] RPLidar C1 motor spinning
- [ ] Both serial ports detected (`ls /dev/ttyUSB*`)
- [ ] Correct ports assigned (RPLidar vs Arduino)
- [ ] udev rules installed

### ROS2 Base
- [ ] `ros2 topic list` shows expected topics
- [ ] `/scan` topic publishing at ~5-10 Hz
- [ ] `/odom` topic publishing at ~20 Hz
- [ ] `/joint_states` publishing
- [ ] `/robot_description` parameter set

### TF Tree
- [ ] `odom → base_link` (diff_drive_node)
- [ ] `base_link → chassis → laser_frame` (robot_state_publisher)
- [ ] `map → odom` (SLAM or AMCL)
- [ ] No warnings about missing transforms

### SLAM
- [ ] SLAM toolbox node active
- [ ] Map building in RViz
- [ ] Map saved successfully

### Navigation
- [ ] Map loaded by map_server
- [ ] AMCL initialized with Pose Estimate
- [ ] All Nav2 lifecycle nodes "active"
- [ ] Robot responds to 2D Goal Pose

### Networking
- [ ] Same WiFi network
- [ ] Same ROS_DOMAIN_ID
- [ ] Same RMW_IMPLEMENTATION
- [ ] Topics visible cross-machine

---

## Quick Reference — Launch Commands

### Orange Pi 5 (Robot):

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1. Robot bringup (always first)
ros2 launch my_bot bringup_hardware.launch.py \
  lidar_port:=/dev/rplidar \
  arduino_port:=/dev/arduino

# 2a. SLAM mode (mapping):
ros2 launch my_bot slam_hardware.launch.py

# 2b. Navigation mode (after map is saved):
ros2 launch my_bot navigation_hardware.launch.py \
  map:=/path/to/map.yaml
```

### Desktop PC:

```bash
# Source environment
source /opt/ros/<distro>/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# RViz visualization
ros2 launch my_bot rviz_remote.launch.py

# Teleop (for SLAM mapping)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Send navigation goals via RViz GUI
```

---

## Encoder Calibration

To determine `ticks_per_rev` for your specific encoders:

```bash
# 1. Mark a starting position on the wheel
# 2. On Orange Pi, reset encoders:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once  # ensure node running
# Or via minicom: send "r" to Arduino

# 3. Manually rotate the wheel exactly ONE full revolution
# 4. Read encoder count:
#    In minicom, the "e" messages show tick counts
#    ticks_per_rev = ticks after one full revolution

# 5. Update the parameter in bringup_hardware.launch.py
```

## Motor Direction Calibration

```bash
# Send forward command:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}}" --once

# Both wheels should spin FORWARD
# If a wheel spins backward, swap IN1/IN2 (or IN3/IN4) wires
# OR negate the PWM in Arduino code for that motor
```


""
ssh orangepi@192.168.18.38    # password: orangepi


Terminal 1 — Hardware Bringup (always first):

source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch my_bot bringup_hardware.launch.py

This starts 3 nodes:
robot_state_publisher — publishes URDF TF tree
rplidar_node — reads RPLidar C1, publishes /scan at 10Hz
diff_drive_node — talks to Arduino, converts /cmd_vel → motor PWM, reads encoders → publishes /odom at 20Hz


Terminal 2 — SLAM mapping (to build a map):

source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot slam_hardware.launch.py
This starts slam_toolbox which builds a map from /scan + /odom.


Terminal 3 — Save the map (after driving around):
source /opt/ros/humble/setup.bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map



Terminal 2 (alternative) — Nav2 navigation (after you have a map):
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot navigation_hardware.launch.py map:=/home/orangepi/maps/my_map.yaml



On Desktop PC (your laptop) — For visualization & control:

Terminal 1 — RViz2 visualization:

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Option A: Use our launch file (if my_bot package is built on desktop too)
ros2 launch my_bot rviz_remote.launch.py

# Option B: Run RViz2 directly
rviz2

In RViz2, add these displays:

RobotModel (topic: /robot_description)
LaserScan (topic: /scan)
Map (topic: /map)
TF (show transforms)
Odometry (topic: /odom)
Path (topic: /plan — for Nav2 paths)
Set Fixed Frame to map

Terminal 2 — Keyboard teleoperation (to drive the robot):

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Typical Workflow Sequence:
Step 1: Wire everything, power on
Step 2: SSH into Orange Pi → run bringup_hardware.launch.py
Step 3: On desktop → open RViz2, verify /scan + /odom visible
Step 4: On desktop → run teleop_twist_keyboard, verify robot moves
Step 5: SSH into Orange Pi → run slam_hardware.launch.py 
Step 6: Drive robot around with teleop to build map (watch in RViz2)
Step 7: SSH into Orange Pi → save map
Step 8: Stop SLAM → start navigation_hardware.launch.py with saved map
Step 9: In RViz2 → click "2D Pose Estimate" to localize
Step 10: In RViz2 → click "2D Goal Pose" to navigate autonomously



Encoder Calibration (important after wiring)
After motors and encoders are connected, you need to calibrate ticks_per_rev. Spin one wheel exactly one full revolution by hand and note the tick count:

# On Orange Pi, with bringup running:
ros2 topic echo /odom    # watch the encoder counts



python3 -c "
import serial, time
ser = serial.Serial('/dev/arduino', 115200, timeout=1)
time.sleep(2)
ser.write(b'r\n')  # reset counters
time.sleep(0.1)
print('Rotate LEFT wheel one full turn, then press Enter')
input()
for i in range(3):
    line = ser.readline().decode().strip()
    print(line)
ser.close()
"

Then update the launch parameter:
ros2 launch my_bot bringup_hardware.launch.py ticks_per_rev:=<your_count>
















































