# Running Guide — Daniel's Robot (LattePanda Alpha)

Step-by-step guide to bring up the robot, run SLAM mapping, and autonomous navigation with Nav2.

---

## Prerequisites

- [x] Ubuntu 22.04 installed on LattePanda Alpha
- [x] ROS2 Humble installed and sourced
- [x] Workspace built (`colcon build --symlink-install`)
- [x] Arduino Leonardo firmware uploaded (see INSTALLATION_GUIDE.md)
- [x] udev rules installed
- [x] RPLidar C1 connected via USB
- [x] L298N + motors + encoders wired correctly
- [x] 12V battery connected to L298N

---

## Step 1: Upload Arduino Firmware (first time / after changes)

```bash
# Open Arduino IDE, load firmware/motor_controller.ino
# Select: Board → Arduino Leonardo, Port → /dev/ttyACM0
# Upload

# Quick test via serial monitor (115200 baud):
#   Send: r        → Should respond: r ok
#   Send: m 100 100 → Both wheels should spin forward
#   Send: m 0 0     → Stop
```

---

## Step 2: Verify Hardware Connections

```bash
# Check Arduino Leonardo
ls -la /dev/arduino
# Should show symlink to /dev/ttyACM*

# Check RPLidar C1
ls -la /dev/rplidar
# Should show symlink to /dev/ttyUSB*

# Quick lidar test
ros2 launch my_bot rplidar.launch.py serial_port:=/dev/rplidar
# In another terminal:
ros2 topic echo /scan --once
# Should show laser scan data
# Ctrl+C to stop
```

---

## Step 3: Launch Robot Bringup

This starts the core robot: URDF/TF tree, LiDAR, and motor controller.

```bash
ros2 launch my_bot bringup_hardware.launch.py
```

**What this launches:**
- `robot_state_publisher` — publishes URDF and static TF tree
- `rplidar_node` — publishes `/scan` from RPLidar C1
- `diff_drive_node` — bridges ROS2 ↔ Arduino Leonardo:
  - Subscribes to `/cmd_vel` → sends motor commands
  - Publishes `/odom` (odometry from encoders)
  - Publishes `/joint_states` (wheel positions)
  - Broadcasts TF: `odom → base_link`

**Verify it's working:**
```bash
# In a new terminal:

# Check topics are publishing
ros2 topic list
# Should show: /scan, /odom, /cmd_vel, /joint_states, /robot_description, /tf, /tf_static

# Check TF tree
ros2 run tf2_tools view_frames
# Should show: map(optional) → odom → base_link → chassis → laser_frame

# Check scan data
ros2 topic hz /scan
# Should show ~5-10 Hz

# Check odometry
ros2 topic hz /odom
# Should show ~20 Hz

# Test motor control (robot will move!)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
# Robot should move forward briefly

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Step 4: Teleoperation (manual driving)

```bash
# Install teleop keyboard (if not already)
sudo apt install -y ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keys: `i` = forward, `k` = stop, `j` = turn left, `l` = turn right, `,` = backward

**Speed settings:** Press `z`/`x` to decrease/increase linear speed (start with 0.1-0.2 m/s)

---

## Step 5: SLAM — Build a Map

With bringup running, start SLAM in a **new terminal**:

```bash
ros2 launch my_bot slam_hardware.launch.py
```

**Visualize on the LattePanda or remote PC:**
```bash
# On the same machine or a remote PC on the same network:
rviz2 -d ~/robot_ws/src/my_bot/config/nav2_view.rviz
```

**In RViz2:**
- Add display: `Map` → topic: `/map`
- Add display: `LaserScan` → topic: `/scan`
- Add display: `TF`
- You should see the map building as you drive the robot around

**Drive the robot** using teleop (Step 4) to explore the environment.

**Save the map** when done:
```bash
# Save the map to a file
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# This creates:
#   ~/maps/my_map.pgm   (occupancy grid image)
#   ~/maps/my_map.yaml   (metadata)
```

Stop SLAM: Ctrl+C

---

## Step 6: Autonomous Navigation with Nav2

With bringup still running, launch Nav2 with your saved map:

```bash
ros2 launch my_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml
```

**In RViz2:**
1. Click **"2D Pose Estimate"** — click and drag on the map where the robot currently is
   (this initializes AMCL localization)
2. Wait a few seconds for the particle cloud to converge
3. Click **"2D Goal Pose"** — click and drag where you want the robot to go
4. The robot should plan a path and navigate autonomously!

---

## Step 7: Verify Everything

```bash
# Check all required topics are active:
ros2 topic list | grep -E "scan|odom|cmd_vel|map|plan"

# Expected output:
#   /scan
#   /odom
#   /cmd_vel
#   /map
#   /plan (when Nav2 is running)

# Check TF tree is complete:
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link → chassis → laser_frame
#                                      → left_wheel
#                                      → right_wheel

# Check node list:
ros2 node list
# Should include:
#   /robot_state_publisher
#   /rplidar_node
#   /diff_drive_node
#   /slam_toolbox (if running SLAM)
#   /amcl, /map_server, /controller_server, etc. (if running Nav2)
```

---

## Remote Operation (optional)

If you want to run RViz2 on a separate desktop PC:

### On LattePanda Alpha:
```bash
# Set ROS_DOMAIN_ID (same on both machines)
export ROS_DOMAIN_ID=42

# Launch bringup + SLAM/Nav2 as above
```

### On Desktop PC:
```bash
# Install ROS2 Humble on desktop (if not already)
# Set same domain ID
export ROS_DOMAIN_ID=42

# Verify topics are visible
ros2 topic list

# Launch RViz2
rviz2 -d path/to/nav2_view.rviz

# Or run teleop from desktop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Note:** Both machines must be on the same network. If using WiFi, ensure multicast is enabled.

---

## Quick Reference — All Commands

```bash
# === BRINGUP ===
ros2 launch my_bot bringup_hardware.launch.py

# === SLAM (mapping) ===
ros2 launch my_bot slam_hardware.launch.py

# === SAVE MAP ===
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# === NAV2 (autonomous navigation) ===
ros2 launch my_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml

# === TELEOP ===
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# === RVIZ ===
rviz2 -d ~/robot_ws/src/my_bot/config/nav2_view.rviz

# === DEBUG ===
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### Robot doesn't move
1. Check Arduino is connected: `ls /dev/arduino`
2. Check diff_drive_node is running: `ros2 node list | grep diff_drive`
3. Check cmd_vel is received: `ros2 topic echo /cmd_vel`
4. Check serial connection: open Arduino Serial Monitor, send `m 100 100`
5. Check L298N power (12V battery connected?)
6. Check motor wiring to L298N OUT pins

### Wheels spin wrong direction
- Swap IN1/IN2 wires (left motor) or IN3/IN4 wires (right motor) on L298N
- OR swap motor terminal wires (M1/M2) on L298N outputs
- OR change sign in firmware: swap `HIGH`/`LOW` for that motor's direction pins

### Encoder counts wrong direction
- In Arduino Serial Monitor, send `r` then manually rotate wheel forward
- Send `d` to see debug output — ticks should increase for forward rotation
- If backwards: swap encoder Green/Yellow wires for that motor

### SLAM map quality is poor
- Drive slowly (0.1-0.15 m/s)
- Make multiple passes through the same area
- Ensure RPLidar has clear line of sight (no obstructions on the robot)
- Check `/scan` data in RViz2 — scans should be consistent

### Nav2 goal fails
- Check map quality — re-map if needed
- Ensure initial pose estimate is accurate (2D Pose Estimate in RViz)
- Check that the goal is in free space on the map
- Look at terminal output for error messages

### Encoder ticks_per_rev calibration
The default `ticks_per_rev=528` assumes a 48:1 gear ratio. To calibrate:
1. Mark the wheel position
2. In Arduino Serial Monitor: send `r` (reset ticks)
3. Manually rotate the wheel exactly ONE full revolution
4. Read the tick count from the `e` messages
5. Update `ticks_per_rev` in `bringup_hardware.launch.py`
