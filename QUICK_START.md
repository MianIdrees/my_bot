# Orange Pi 5 Quick Start Guide

This is a condensed version of [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) for quick reference.

## 🎯 Recommended Configuration

| Component | Specification | Reason |
|-----------|--------------|--------|
| **Orange Pi 5 OS** | Ubuntu 22.04 LTS | Best ARM64/Rockchip support |
| **Orange Pi 5 ROS** | Humble Hawksbill | LTS, excellent hardware support |
| **Laptop OS** | Ubuntu 24.04 LTS | Keep current setup |
| **Laptop ROS** | Jazzy Jalisco | Keep current setup |
| **Network** | Static IPs, same subnet | Reliable communication |
| **ROS Domain** | ROS_DOMAIN_ID=42 | Both machines |

✅ **Why different ROS versions?** Humble and Jazzy communicate perfectly over network, and this gives you the best stability on Orange Pi while keeping latest features on laptop.

## 📋 Quick Setup Steps

### 1. Prepare Orange Pi 5 (30 minutes)

```bash
# Download Armbian Ubuntu 22.04 for Orange Pi 5
# https://www.armbian.com/orange-pi-5/

# Flash to microSD card (64GB+ recommended)
# Boot, configure user, update system
sudo apt update && sudo apt upgrade -y

# Set static IP
sudo nano /etc/netplan/01-netcfg.yaml
# Configure: 192.168.1.100/24
sudo netplan apply
```

### 2. Install ROS 2 Humble (20 minutes)

```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble Base
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions

# Install required packages
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-sllidar-ros2

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone and Build Robot Package (5 minutes)

```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone your repository
git clone https://github.com/MianIdrees/my_bot.git

# Build
cd ~/robot_ws
colcon build --symlink-install
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Upload Arduino Firmware (10 minutes)

**On your laptop** (before connecting to Orange Pi):

```bash
# Install Arduino IDE
sudo apt install -y arduino

# Connect Arduino Nano via USB
# Open Arduino IDE
# Tools → Board → Arduino Nano
# Tools → Processor → ATmega328P (Old Bootloader)
# File → Open → ~/robot_ws/src/my_bot/arduino/differential_drive_controller.ino
# Click Upload button
```

See [arduino/README.md](arduino/README.md) for detailed wiring and troubleshooting.

### 5. Connect Hardware

```
Orange Pi 5 USB → RPLidar C1 USB
Orange Pi 5 USB → Arduino Nano USB Mini-B

Arduino Nano Pins:
  Pin 3 (PWM) → L298N ENA
  Pin 4      → L298N IN1
  Pin 5      → L298N IN2
  Pin 6 (PWM) → L298N ENB
  Pin 7      → L298N IN3
  Pin 8      → L298N IN4
  GND        → L298N GND

L298N → Motors:
  OUT1, OUT2 → Left Motor
  OUT3, OUT4 → Right Motor
  12V, GND   → 12V Power Supply
```

### 6. Configure Network (5 minutes)

**On Orange Pi 5:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Verify IP: 192.168.1.100
ip addr show wlan0
```

**On Laptop:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Test connectivity
ping 192.168.1.100
```

### 7. Test Everything (10 minutes)

**On Orange Pi 5:**
```bash
# Launch robot
ros2 launch my_bot launch_robot.launch.py
```

**On Laptop:**
```bash
# Check topics from Orange Pi
ros2 topic list
# Should see: /scan, /cmd_vel, /tf, etc.

# Launch RViz for visualization
rviz2

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ⚠️ Common Issues

### RPLidar not detected
```bash
ls -l /dev/ttyUSB*
sudo usermod -a -G dialout $USER
sudo reboot
```

### Arduino not detected
```bash
ls -l /dev/ttyACM*
sudo usermod -a -G dialout $USER
sudo reboot
```

### Motors not moving
1. Check 12V power supply to L298N
2. Verify all wiring connections
3. Test with serial terminal: `screen /dev/ttyACM0 115200`
4. Send: `L:0.1,A:0.0` (should move forward)

### ROS 2 nodes not discovering
```bash
# Same domain on both machines?
echo $ROS_DOMAIN_ID

# Ping working?
ping 192.168.1.100

# Firewall disabled? (for testing)
sudo ufw disable
```

## 📚 Full Documentation

- **[HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md)** - Complete guide (32KB)
- **[arduino/README.md](arduino/README.md)** - Arduino setup and troubleshooting (10KB)
- **[README.md](README.md)** - Main project documentation

## 🎯 Next Steps After Basic Setup

1. **Test autonomous navigation** - Add Nav2 stack
2. **Improve odometry** - Enable encoder feedback in Arduino
3. **Add IMU sensor** - Better orientation estimation
4. **Create systemd service** - Auto-start robot on boot
5. **Implement SLAM** - Map your environment

## 💡 Pro Tips

- **Use SSH** - `ssh robot@192.168.1.100` for remote access
- **Screen/Tmux** - Run multiple terminals over SSH
- **Low latency** - Use wired Ethernet instead of WiFi when possible
- **Battery monitoring** - Add voltage sensor for autonomous operation
- **Heatsink** - Add to L298N if it gets hot
- **Cable management** - Keep motor wires away from signal wires

## 🆘 Need Help?

1. Check the **Troubleshooting** section in [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md)
2. Verify all wiring matches the diagrams in [arduino/README.md](arduino/README.md)
3. Monitor Arduino serial: `arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200`
4. Check ROS 2 logs: `ros2 node list` and `ros2 topic hz /scan`

---

**Total Setup Time:** ~90 minutes from zero to driving robot! 🚀
