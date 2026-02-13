# Complete Robot Deployment Guide — Simulation to Real Hardware

**Project**: ROS2 Differential Drive Robot with Nav2 + SLAM  
**Hardware**: Orange Pi 5 + Arduino Uno + RPLidar C1 + L298N + 2x JGB37-520 DC Motors  
**Verified**: February 2026  

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Components](#2-hardware-components)
3. [Complete Wiring Guide](#3-complete-wiring-guide)
4. [Orange Pi 5 Setup (One-Time)](#4-orange-pi-5-setup-one-time)
5. [Arduino Firmware Upload](#5-arduino-firmware-upload)
6. [Operating the Robot](#6-operating-the-robot)
7. [Desktop Visualization (RViz2)](#7-desktop-visualization-rviz2)
8. [SLAM Mapping](#8-slam-mapping)
9. [Autonomous Navigation (Nav2)](#9-autonomous-navigation-nav2)
10. [Encoder Calibration](#10-encoder-calibration)
11. [Troubleshooting](#11-troubleshooting)
12. [Quick Reference Card](#12-quick-reference-card)

---

## 1. System Overview

### What Runs Where

```
┌─────────────────────────────────────────────────────────────────┐
│                DESKTOP PC (Ubuntu 24.04 — Your Laptop)          │
│                                                                  │
│   RViz2 ← visualize map, laser scan, robot pose, paths          │
│   teleop_twist_keyboard ← drive robot with keyboard             │
│                                                                  │
│   ROS2 Jazzy | ROS_DOMAIN_ID=42 | CycloneDDS                    │
└──────────────────────────┬──────────────────────────────────────┘
                           │ WiFi (same network)
                           │ DDS auto-discovery
┌──────────────────────────┴──────────────────────────────────────┐
│              ORANGE PI 5 (Ubuntu 22.04 — On the Robot)           │
│                                                                  │
│   bringup_hardware.launch.py (ALWAYS first)                      │
│   ├── robot_state_publisher   → publishes URDF TF tree           │
│   ├── rplidar_node            → reads RPLidar → /scan at 10 Hz   │
│   └── diff_drive_node.py      → Arduino bridge → /odom at 20 Hz │
│                                                                  │
│   slam_hardware.launch.py (for mapping)                          │
│   └── slam_toolbox            → builds map from /scan + /odom    │
│                                                                  │
│   navigation_hardware.launch.py (for autonomous navigation)      │
│   └── Nav2 stack              → path planning + obstacle avoid   │
│                                                                  │
│   ROS2 Humble | ROS_DOMAIN_ID=42 | CycloneDDS                   │
└───────────┬──────────────────────────┬──────────────────────────┘
            │ USB (/dev/rplidar)       │ USB (/dev/arduino)
     ┌──────┴──────┐           ┌───────┴────────┐
     │  RPLidar C1 │           │  Arduino Uno   │
     │  460800 baud│           │  115200 baud   │
     └─────────────┘           │    ↕ serial     │
                               │  L298N driver  │
                               │  2x DC motors  │
                               │  2x Encoders   │
                               └────────────────┘
```

### TF Frame Tree

```
map ──→ odom ──→ base_link ──→ chassis ──→ laser_frame
                     │
                     ├──→ left_wheel
                     ├──→ right_wheel
                     └──→ base_footprint
```

| Transform | Published By | Rate |
|-----------|-------------|------|
| map → odom | SLAM Toolbox (mapping) or AMCL (navigation) | 20 Hz |
| odom → base_link | diff_drive_node.py (encoder odometry) | 20 Hz |
| base_link → everything else | robot_state_publisher (from URDF) | static |

### ROS2 Topics

| Topic | Type | Published By | Used By |
|-------|------|-------------|---------|
| `/scan` | LaserScan | rplidar_node | SLAM, Nav2, RViz2 |
| `/odom` | Odometry | diff_drive_node | SLAM, Nav2, RViz2 |
| `/cmd_vel` | Twist | teleop / Nav2 | diff_drive_node |
| `/map` | OccupancyGrid | slam_toolbox / map_server | Nav2, RViz2 |
| `/tf` | TFMessage | RSP + diff_drive + SLAM | everything |
| `/joint_states` | JointState | diff_drive_node | RSP |
| `/robot_description` | String | RSP | RViz2 |

---

## 2. Hardware Components

### Bill of Materials

| # | Component | Specification | Notes |
|---|-----------|--------------|-------|
| 1 | Orange Pi 5 | 16GB RAM, RK3588S, Ubuntu 22.04 | Brain of the robot |
| 2 | Arduino Uno | ATmega328P, CH340 USB-serial | Motor + encoder controller |
| 3 | RPLidar C1 | Slamtec, CP210x USB-serial | 2D laser scanner, 10 Hz, 8m range |
| 4 | L298N Motor Driver | Dual H-bridge, 12V/2A per channel | Drives both motors |
| 5 | JGB37-520 DC Motor (x2) | 12V, 110RPM, 11 PPR Hall encoder, ~90:1 gear ratio | Left and right drive motors |
| 6 | 65mm Rubber Wheels (x2) | 65mm diameter × 26mm width | Mounted on 6mm D-shaft via coupling |
| 7 | 12V Battery | LiPo 3S (11.1V) or 12V lead-acid | Powers motors via L298N |
| 8 | USB Cables (x2) | USB-B for Arduino, micro-USB for RPLidar | Data + power |
| 9 | Jumper Wires | Male-male and male-female | For all signal connections |
| 10 | Robot Chassis | ~30cm × 30cm platform | With 2 drive wheels + 1 caster |
| 11 | Motor Brackets (x2) | Metal L-brackets | Included with JGB37-520 kit |
| 12 | Shaft Couplings (x2) | 6mm to wheel adapter | Included with JGB37-520 kit |

### Robot Dimensions (from URDF)

- **Chassis**: 0.30m × 0.30m × 0.15m (length × width × height)
- **Wheel radius**: 0.0325m (65mm diameter)
- **Wheel separation**: 0.35m (center-to-center distance between wheels)
- **Wheel width**: 0.026m (26mm)
- **Max speed**: ~0.37 m/s (110 RPM × π × 0.065m)

---

## 3. Complete Wiring Guide

### JGB37-520 Motor — 6-Pin Connector

Each JGB37-520 motor has a 6-pin connector with these pins (as labelled on the connector):

| Pin | Wire Color | Function |
|-----|-----------|----------|
| **M1** | Red | Motor terminal + (to L298N output) |
| **GND** | Black | Encoder ground (to Arduino GND) |
| **C2** | Yellow | Encoder signal B (to Arduino digital pin) |
| **C1** | Green | Encoder signal A (to Arduino interrupt pin) |
| **Vcc** | Blue | Encoder power (to Arduino 5V) |
| **M2** | White | Motor terminal - (to L298N output) |

### 3.1 — Arduino Uno → L298N Motor Driver (6 wires)

```
ARDUINO UNO                          L298N MOTOR DRIVER
─────────────                        ──────────────────
Pin 5  (PWM) ─────── yellow ───────→ ENA  (Left motor speed)
Pin 7        ─────── orange ───────→ IN1  (Left motor direction A)
Pin 8        ─────── orange ───────→ IN2  (Left motor direction B)

Pin 6  (PWM) ─────── yellow ───────→ ENB  (Right motor speed)
Pin 9        ─────── blue ─────────→ IN3  (Right motor direction A)
Pin 10       ─────── blue ─────────→ IN4  (Right motor direction B)
```

> **IMPORTANT**: Remove the jumper caps from ENA and ENB on the L298N board!
> Those jumpers hard-wire speed to maximum. You need Arduino PWM control instead.

### 3.2 — L298N → JGB37-520 Motors (4 wires)

Connect the L298N output terminals to the motor terminals (M1 and M2 on each motor's 6-pin connector):

```
L298N                                JGB37-520 MOTORS
─────                                ────────────────
OUT1 ─────── red ──────────────────→ Left Motor  M1 (Red wire)
OUT2 ─────── white ────────────────→ Left Motor  M2 (White wire)

OUT3 ─────── red ──────────────────→ Right Motor M1 (Red wire)
OUT4 ─────── white ────────────────→ Right Motor M2 (White wire)
```

> If a motor spins in the wrong direction, just swap M1 and M2 for that motor.

### 3.3 — JGB37-520 Encoders → Arduino Uno (4 signal + 4 power wires)

From each motor's 6-pin connector, connect the encoder pins:

```
LEFT MOTOR (6-pin connector)         ARDUINO UNO
────────────────────────────         ───────────
C1  (Green wire)  ─────────────────→ Pin 2   (Hardware Interrupt INT0)
C2  (Yellow wire) ─────────────────→ Pin 4   (Direction sensing)
Vcc (Blue wire)   ─────────────────→ 5V
GND (Black wire)  ─────────────────→ GND

RIGHT MOTOR (6-pin connector)        ARDUINO UNO
─────────────────────────────        ───────────
C1  (Green wire)  ─────────────────→ Pin 3   (Hardware Interrupt INT1)
C2  (Yellow wire) ─────────────────→ Pin 12  (Direction sensing)
Vcc (Blue wire)   ─────────────────→ 5V
GND (Black wire)  ─────────────────→ GND
```

> **Pins 2 and 3 MUST be used for C1** — they are the only hardware interrupt pins on Arduino Uno. The firmware uses interrupts for accurate tick counting.

> **If encoder counts go in the wrong direction**: swap C1 and C2 wires for that motor.

### 3.4 — Power Wiring

```
12V BATTERY                          L298N
───────────                          ─────
(+) positive ───── thick red ──────→ 12V terminal (screw terminal)
(-) negative ───── thick black ────→ GND terminal (screw terminal)

L298N                                ARDUINO UNO
─────                                ───────────
GND ──────────── black ────────────→ GND pin
                                     (COMMON GROUND — CRITICAL!)

ARDUINO UNO                          ORANGE PI 5
───────────                          ───────────
USB-B port ────── USB cable ───────→ Any USB-A port
                                     (provides data + 5V power to Arduino)

RPLidar C1                           ORANGE PI 5
──────────                           ───────────
Micro-USB ─────── USB cable ───────→ Any USB-A port (different from Arduino)
```

> **Do NOT connect L298N's 5V output to Arduino's 5V pin** — the Arduino is already powered from the Orange Pi via USB.

> **The L298N GND ↔ Arduino GND wire is CRITICAL**. Without it, the motor driver signals will not work correctly.

### 3.5 — Complete Pin Reference Table

| Arduino Uno Pin | Connected To | Function |
|-----------------|-------------|----------|
| **Pin 2** | Left Motor C1 (Green) | Interrupt INT0 (encoder tick counting) |
| **Pin 3** | Right Motor C1 (Green) | Interrupt INT1 (encoder tick counting) |
| **Pin 4** | Left Motor C2 (Yellow) | Direction sensing |
| **Pin 5** (PWM) | L298N ENA | Left motor speed (0-255) |
| **Pin 6** (PWM) | L298N ENB | Right motor speed (0-255) |
| **Pin 7** | L298N IN1 | Left motor direction |
| **Pin 8** | L298N IN2 | Left motor direction |
| **Pin 9** | L298N IN3 | Right motor direction |
| **Pin 10** | L298N IN4 | Right motor direction |
| **Pin 12** | Right Motor C2 (Yellow) | Direction sensing |
| **5V** | Both Motor Vcc (Blue) | Encoder power |
| **GND** | Both Motor GND (Black) + L298N GND | Common ground |
| **USB-B** | Orange Pi 5 USB-A | Serial data (115200 baud) + 5V power |

### 3.6 — Physical Layout on Robot

```
                  FRONT (caster wheel side)
         ┌──────────────────────────────────┐
         │                                  │
         │        ┌──────────────┐          │
         │        │   RPLidar    │          │   ← mounted on TOP
         │        │     C1       │          │     centered, clear view
         │        └──────────────┘          │
         │                                  │
         │   ┌──────────┐  ┌────────────┐   │
         │   │ Orange   │  │  Arduino   │   │
         │   │  Pi 5    │  │   Uno      │   │
         │   └──────────┘  └────────────┘   │
         │                                  │
         │        ┌──────────────┐          │
         │        │    L298N     │          │
         │        │   Driver     │          │
         │        └──────────────┘          │
         │                                  │
         │     ┌──────────────────┐         │
         │     │   12V Battery   │         │
         │     └──────────────────┘         │
         │                                  │
    ═════╪══════════════════════════════════╪═════
    LEFT │          0.35m apart             │ RIGHT
    WHEEL│        (center to center)        │ WHEEL
    ═════╪══════════════════════════════════╪═════
         │                                  │
         │          (caster wheel)          │
         └──────────────────────────────────┘
                    REAR
```

---

## 4. Orange Pi 5 Setup (One-Time)

> All of this has already been done on your Orange Pi at 192.168.18.38.
> Included here for reference if you ever need to redo it.

### 4.1 — SSH Connection

```bash
ssh orangepi@192.168.18.38
# password: orangepi
```

### 4.2 — System Info (Verified)

- **OS**: Ubuntu 22.04.5 LTS (Jammy)
- **Architecture**: aarch64 (ARM64)
- **Kernel**: 5.10.0-1012-rockchip
- **RAM**: 16 GB
- **Disk**: ~49 GB free

### 4.3 — ROS2 Humble Installed Packages

```
ros-humble-ros-base
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-nav2-common
ros-humble-slam-toolbox
ros-humble-rplidar-ros
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-diff-drive-controller
ros-humble-joint-state-broadcaster
ros-humble-tf2, tf2-ros, tf2-tools, tf-transformations
ros-humble-rmw-cyclonedds-cpp
python3-serial (pyserial)
arduino-cli v1.4.1
```

### 4.4 — Environment Configuration (~/.bashrc on Orange Pi)

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 4.5 — Workspace Location

```
~/robot_ws/
├── src/
│   └── my_bot/          ← our robot package
│       ├── config/      ← Nav2, SLAM, RViz config files
│       ├── description/ ← URDF xacro files
│       ├── firmware/    ← Arduino motor_controller.ino
│       ├── launch/      ← all launch files
│       ├── scripts/     ← diff_drive_node.py, start_robot.sh
│       └── worlds/      ← Gazebo worlds (sim only)
├── build/
└── install/
```

### 4.6 — USB Device Detection

| Device | USB Chip | Vendor:Product | udev Symlink | ttyUSB |
|--------|---------|----------------|-------------|--------|
| RPLidar C1 | Silicon Labs CP210x | 10c4:ea60 | `/dev/rplidar` | ttyUSB0 or ttyUSB2 |
| Arduino Uno | CH340 | 1a86:7523 | `/dev/arduino` | ttyUSB1 |

The udev rules at `/etc/udev/rules.d/99-robot-devices.rules` create the stable symlinks `/dev/rplidar` and `/dev/arduino`, so the exact ttyUSB number doesn't matter.

### 4.7 — Rebuilding the Workspace

If you change any source code on the Orange Pi:

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Arduino Firmware Upload

### 5.1 — What the Firmware Does

The file `firmware/motor_controller.ino` runs on the Arduino Uno and:
- Controls 2x JGB37-520 DC12V 110RPM geared motors via L298N driver
- Reads Hall encoder ticks (11 PPR × 90:1 gear = 990 ticks/rev) via hardware interrupts on pins 2 and 3
- Sends encoder counts to Orange Pi every 50ms: `e <left_ticks> <right_ticks>\n`
- Receives motor commands from Orange Pi: `m <left_pwm> <right_pwm>\n` (range -255 to 255)
- Accepts encoder reset: `r\n` → responds `r ok\n`
- Safety: stops motors if no command received for 500ms

### 5.2 — Upload from Orange Pi (Verified Working)

```bash
ssh orangepi@192.168.18.38

# The firmware is located at:
# ~/robot_ws/src/my_bot/firmware/motor_controller/motor_controller.ino

# Compile for Arduino Uno
arduino-cli compile --fqbn arduino:avr:uno \
  ~/robot_ws/src/my_bot/firmware/motor_controller

# Upload to Arduino (with verification)
arduino-cli upload -p /dev/arduino --fqbn arduino:avr:uno \
  ~/robot_ws/src/my_bot/firmware/motor_controller
```

### 5.3 — Verify Upload

```bash
# Quick serial test
python3 -c "
import serial, time
ser = serial.Serial('/dev/arduino', 115200, timeout=3)
time.sleep(3)
ser.reset_input_buffer()
for i in range(5):
    line = ser.readline().decode().strip()
    print(line)
ser.write(b'r\n')
time.sleep(0.1)
print(ser.readline().decode().strip())
ser.close()
"
```

Expected output:
```
e 0 0
e 0 0
e 0 0
e 0 0
e 0 0
r ok
```

### 5.4 — Known Issue: Counterfeit Arduino Boards

We discovered that one Arduino Uno board had a counterfeit ATmega328P chip. The upload appeared to succeed, but flash verification failed (every byte read back as `0x62`). The solution was to swap it with a genuine board.

To verify your board, upload with full verification:
```bash
~/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/bin/avrdude \
  -C ~/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/etc/avrdude.conf \
  -v -patmega328p -carduino -P/dev/arduino -b115200 \
  -Uflash:w:<path-to-hex-file>:i
```

If you see `flash verification mismatch`, the board is defective. Replace it.

---

## 6. Operating the Robot

### 6.1 — Startup Sequence (Orange Pi)

Open an SSH session to the Orange Pi:
```bash
ssh orangepi@192.168.18.38
```

**Step 1 — Hardware Bringup (always first, always required):**
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot bringup_hardware.launch.py
```

This starts 3 nodes:
| Node | Function | Topic |
|------|----------|-------|
| `robot_state_publisher` | Publishes URDF TF tree (base_link→chassis→laser_frame→wheels) | `/tf_static`, `/robot_description` |
| `rplidar_node` | Reads RPLidar C1 via USB, publishes laser scans | `/scan` at 10 Hz |
| `diff_drive_node.py` | Bridges Arduino ↔ ROS2: cmd_vel→PWM, encoders→odometry | `/odom` at 20 Hz, `/joint_states`, `/tf` |

Wait until you see:
```
[rplidar_node]: RPLidar health status : OK.
[diff_drive_node]: Connected to Arduino on /dev/arduino
[rplidar_node]: current scan mode: Standard, sample rate: 5 Khz, max_distance: 16.0 m
```

**Step 2 — Choose your mode** (open a second SSH terminal):

For **SLAM mapping** (first time, to build a map):
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot slam_hardware.launch.py
```

For **Autonomous Navigation** (after you have a saved map):
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot navigation_hardware.launch.py map:=/home/orangepi/maps/my_map.yaml
```

### 6.2 — Quick Start Script (Alternative)

```bash
# Bringup only
~/robot_ws/src/my_bot/scripts/start_robot.sh bringup

# Bringup + SLAM
~/robot_ws/src/my_bot/scripts/start_robot.sh slam

# Bringup + Nav2 (requires map)
~/robot_ws/src/my_bot/scripts/start_robot.sh nav
```

### 6.3 — Shutdown Sequence

```bash
# Ctrl+C in each terminal to stop the launch files
# Or kill all ROS2 processes:
pkill -f "ros2\|rplidar\|slam_toolbox\|diff_drive\|robot_state"
```

---

## 7. Desktop Visualization (RViz2)

> **Gazebo is NOT used with real hardware.** Gazebo is a simulator — when running on real hardware, the real world IS your environment. You visualize sensor data and maps using RViz2 on your desktop.

### 7.1 — Desktop Environment Setup

Add to your `~/.bashrc` on the desktop:
```bash
source /opt/ros/jazzy/setup.bash    # or humble if installed
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Install CycloneDDS if not present:
```bash
# For Jazzy:
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

# For Humble:
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

### 7.2 — Launch RViz2

**Option A — Using our launch file** (requires my_bot built on desktop):
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch my_bot rviz_remote.launch.py
```

**Option B — Launch RViz2 directly** (always works):
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rviz2
```

### 7.3 — Configure RViz2 Displays

In RViz2, set **Fixed Frame** to `map` (top-left dropdown).

Add these displays (click "Add" button):

| Display Type | Topic | What It Shows |
|-------------|-------|---------------|
| **RobotModel** | `/robot_description` | 3D robot model (chassis + wheels) |
| **LaserScan** | `/scan` | Red dots showing lidar measurements |
| **Map** | `/map` | Occupancy grid built by SLAM |
| **TF** | (built-in) | Coordinate frames (map, odom, base_link) |
| **Odometry** | `/odom` | Robot pose arrow showing position estimate |
| **Path** | `/plan` | Green line showing Nav2 planned path |

### 7.4 — Verify Cross-Machine Communication

On your desktop, check that Orange Pi topics are visible:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# List topics from Orange Pi
ros2 topic list

# Should show:
# /scan
# /odom
# /map
# /tf
# /tf_static
# /joint_states
# /cmd_vel
# /robot_description
```

If topics don't appear:
1. Verify both machines are on the **same WiFi network**
2. Verify both use `ROS_DOMAIN_ID=42`
3. Verify both use `rmw_cyclonedds_cpp`
4. Check firewall: `sudo ufw allow 7400:7500/udp`

### 7.5 — Keyboard Teleoperation (Drive the Robot)

Open a new terminal on your desktop:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Install if needed:
```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

**Keyboard controls:**
```
   u    i    o
   j    k    l
   m    ,    .

i = forward       , = backward
j = turn left      l = turn right
u = forward+left   o = forward+right
k = STOP

q/z = increase/decrease max speed
w/x = increase/decrease linear speed only
e/c = increase/decrease angular speed only
```

**Start with low speeds!** Press `z` several times to reduce speed before driving.

---

## 8. SLAM Mapping

### 8.1 — Build a Map

**On Orange Pi** (Terminal 1):
```bash
ros2 launch my_bot bringup_hardware.launch.py
```

**On Orange Pi** (Terminal 2):
```bash
ros2 launch my_bot slam_hardware.launch.py
```

**On Desktop** (Terminal 1):
```bash
rviz2    # Add Map display for /map topic, set Fixed Frame = map
```

**On Desktop** (Terminal 2):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Now **slowly drive the robot** around the entire area you want to map. Watch the map build in real-time in RViz2.

Tips:
- Drive slowly (reduce speed with `z` key)
- Make overlapping loops to help SLAM close loops
- Avoid fast rotations
- Cover all areas you want the robot to navigate later

### 8.2 — Save the Map

When the map looks complete in RViz2, **save it** on the Orange Pi:

```bash
# On Orange Pi (Terminal 3):
source /opt/ros/humble/setup.bash

mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This creates two files:
- `~/maps/my_map.yaml` — map metadata
- `~/maps/my_map.pgm` — occupancy grid image (white=free, black=wall, gray=unknown)

### 8.3 — Copy Map to Desktop (for backup)

```bash
# On your desktop:
mkdir -p ~/robot_maps
scp orangepi@192.168.18.38:~/maps/my_map.* ~/robot_maps/
```

---

## 9. Autonomous Navigation (Nav2)

### 9.1 — Prerequisites

- A saved map from SLAM (Section 8)
- Bringup running on Orange Pi

### 9.2 — Launch Nav2

**On Orange Pi** (Terminal 1 — if not already running):
```bash
ros2 launch my_bot bringup_hardware.launch.py
```

**On Orange Pi** (Terminal 2):
```bash
ros2 launch my_bot navigation_hardware.launch.py \
  map:=/home/orangepi/maps/my_map.yaml
```

Wait until you see `lifecycle_manager_navigation` report all nodes active.

**On Desktop**:
```bash
rviz2    # Set Fixed Frame = map
         # Add: Map, LaserScan, RobotModel, Path, TF
```

### 9.3 — Set Initial Pose

The robot doesn't know where it is on the map initially. In RViz2:

1. Click **"2D Pose Estimate"** button (top toolbar)
2. Click and drag on the map at the robot's **actual physical location**
3. The arrow direction = the direction the robot is facing
4. AMCL will begin localizing — the laser scan should align with map walls

### 9.4 — Send Navigation Goals

1. Click **"2D Goal Pose"** (Nav2 Goal) button in RViz2 toolbar
2. Click and drag on the map where you want the robot to go
3. Nav2 will plan a path (green line) and the robot will drive autonomously
4. The robot avoids obstacles detected by the lidar

### 9.5 — Nav2 Parameters

The Nav2 configuration is in `config/nav2_params_hardware.yaml`:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Controller frequency | 10 Hz | How fast the controller runs |
| Max linear speed | 0.2 m/s | Maximum forward speed |
| Max angular speed | 0.5 rad/s | Maximum rotation speed |
| Planner | NavfnPlanner (A*) | Global path planning algorithm |
| Controller | DWB | Dynamic Window controller |
| Costmap resolution | 0.05 m | 5cm per pixel |
| Inflation radius | 0.3 m | Safety buffer around obstacles |
| Robot radius | ~0.18 m | Based on 0.30m chassis + margin |

---

## 10. Encoder Calibration

### 10.1 — Why Calibrate

The `ticks_per_rev` parameter tells the diff_drive_node how many encoder ticks equal one full wheel revolution. This directly affects odometry accuracy. The default is 990 ticks/rev (JGB37-520: 11 PPR × 90:1 gear ratio) but your specific motor may differ slightly.

### 10.2 — Measure Ticks Per Revolution

**On the Orange Pi**, with bringup running:

```bash
# Method 1: Read from Arduino serial directly
python3 -c "
import serial, time
ser = serial.Serial('/dev/arduino', 115200, timeout=1)
time.sleep(2)
ser.write(b'r\n')  # reset counters to zero
time.sleep(0.5)
ser.reset_input_buffer()
print('>>> Rotate the LEFT wheel exactly ONE full revolution by hand')
print('>>> Then press Enter')
input()
for i in range(3):
    line = ser.readline().decode().strip()
    print(f'  {line}')
print()
print('The first number after \"e\" is your left ticks_per_rev')
ser.close()
"
```

Repeat for the right wheel to verify they match.

### 10.3 — Apply Calibration

```bash
# Pass the measured value when launching:
ros2 launch my_bot bringup_hardware.launch.py ticks_per_rev:=<your_value>
```

Or permanently edit [launch/bringup_hardware.launch.py](launch/bringup_hardware.launch.py) line 42:
```python
'ticks_per_rev', default_value='<your_value>',  # default 990 for JGB37-520 110RPM
```

### 10.4 — Verify Motor Direction

After connecting motors, test that forward commands move the robot forward:

```bash
# On desktop:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

Both wheels should spin in the **forward** direction. If one is reversed, swap its OUT1/OUT2 wires on the L298N.

Test turning:
```bash
# Turn left (right wheel forward, left wheel backward):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once
```

---

## 11. Troubleshooting

### No serial output from Arduino

| Symptom | Cause | Fix |
|---------|-------|-----|
| Upload succeeds but no serial data | Counterfeit ATmega328P | Replace the Arduino board; verify with `avrdude` flash verification |
| `brltty` keeps stealing the CH340 | Ubuntu's braille display driver claims CH340 | `sudo apt remove brltty` |
| `/dev/ttyUSB1` disappears | USB power issue or loose cable | Reconnect, check `dmesg \| tail` |

### RPLidar issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Timeout at 115200 baud | RPLidar C1 uses 460800 baud | Already configured in our launch files |
| No `/dev/rplidar` symlink | udev rule not loaded | `sudo udevadm control --reload-rules && sudo udevadm trigger` |
| Health status not OK | Dirty lens or obstructed | Clean the lidar lens; ensure nothing blocks rotation |

### RViz2 can't see topics

| Symptom | Cause | Fix |
|---------|-------|-----|
| Empty topic list on desktop | Different ROS_DOMAIN_ID | Both machines must use `export ROS_DOMAIN_ID=42` |
| Topics listed but no data | Different DDS middleware | Both must use `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| DDS discovery fails | Firewall blocking UDP | `sudo ufw allow 7400:7500/udp` |
| Type hash warnings (Jazzy↔Humble) | Normal cross-version behavior | Warnings can be ignored; data still flows |

### SLAM not building map

| Symptom | Cause | Fix |
|---------|-------|-----|
| "Message Filter dropping message" | TF not available yet | This is normal at startup; SLAM starts after a few seconds |
| "frame 'laser_frame' does not exist" | Robot state publisher not running | Always start `bringup_hardware.launch.py` FIRST |
| Map not updating | Robot not moving enough | Drive the robot; SLAM needs motion to register scans |

### Motors not responding

| Symptom | Cause | Fix |
|---------|-------|-----|
| diff_drive_node connects but motors don't move | L298N ENA/ENB jumpers still on | Remove the jumper caps from ENA and ENB |
| One motor doesn't spin | Loose L298N wiring | Check OUT1-4 connections and IN1-4 wires |
| Motors spin wrong direction | M1/M2 polarity reversed | Swap M1↔M2 wires on the L298N output for that motor |
| Motors stop after 0.5s | No cmd_vel being sent | The firmware has a 500ms safety timeout; keep sending commands |

### Odometry drifting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Odometry drifting badly | Wrong ticks_per_rev | Calibrate (Section 10); default 990 for JGB37-520 |
| Robot turns when going straight | Wheel diameters differ | Adjust wheel_radius per side in diff_drive_node params |
| Position jumps | Encoder wires loose | Check C1/C2 connections on 6-pin connector; secure with hot glue |
| Encoder counts wrong direction | C1/C2 swapped | Swap C1 and C2 wires for that motor |

---

## 12. Quick Reference Card

### SSH into Orange Pi
```bash
ssh orangepi@192.168.18.38     # password: orangepi
```

### Standard Environment (add to every terminal)
```bash
# On Orange Pi:
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# On Desktop:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Orange Pi Commands
```bash
# Bringup (always first)
ros2 launch my_bot bringup_hardware.launch.py

# SLAM mapping
ros2 launch my_bot slam_hardware.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Navigation with saved map
ros2 launch my_bot navigation_hardware.launch.py map:=/home/orangepi/maps/my_map.yaml

# Check topics
ros2 topic list

# Check TF
ros2 run tf2_ros tf2_echo map base_link

# Check node health
ros2 node list
```

### Desktop Commands
```bash
# Visualize
rviz2

# Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Check scan data
ros2 topic echo /scan --once

# Send a test velocity
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once
```

### Arduino Commands (on Orange Pi)
```bash
# Compile firmware
arduino-cli compile --fqbn arduino:avr:uno ~/robot_ws/src/my_bot/firmware/motor_controller

# Upload firmware
arduino-cli upload -p /dev/arduino --fqbn arduino:avr:uno ~/robot_ws/src/my_bot/firmware/motor_controller

# Serial monitor
python3 -c "import serial,time; s=serial.Serial('/dev/arduino',115200,timeout=1); time.sleep(2); [print(s.readline().decode().strip()) for _ in range(10)]; s.close()"
```

### Rebuild Workspace (on Orange Pi)
```bash
cd ~/robot_ws && colcon build --symlink-install && source install/setup.bash
```

### File Locations

| File | Location on Orange Pi | Purpose |
|------|----------------------|---------|
| Bringup launch | `~/robot_ws/src/my_bot/launch/bringup_hardware.launch.py` | Start robot hardware |
| SLAM launch | `~/robot_ws/src/my_bot/launch/slam_hardware.launch.py` | Start mapping |
| Nav2 launch | `~/robot_ws/src/my_bot/launch/navigation_hardware.launch.py` | Start navigation |
| RViz remote | `~/robot_ws/src/my_bot/launch/rviz_remote.launch.py` | Desktop visualization |
| diff_drive_node | `~/robot_ws/src/my_bot/scripts/diff_drive_node.py` | Arduino↔ROS2 bridge |
| Arduino firmware | `~/robot_ws/src/my_bot/firmware/motor_controller/motor_controller.ino` | Motor + encoder controller |
| Nav2 params | `~/robot_ws/src/my_bot/config/nav2_params_hardware.yaml` | Navigation tuning |
| SLAM params | `~/robot_ws/src/my_bot/config/mapper_params_online_async_hardware.yaml` | SLAM tuning |
| udev rules | `/etc/udev/rules.d/99-robot-devices.rules` | USB device symlinks |
| Saved maps | `~/maps/` | Map files |

---

*Guide generated from verified hardware testing on February 13, 2026.*
