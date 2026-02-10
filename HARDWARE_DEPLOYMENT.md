# Hardware Deployment Guide for Orange Pi 5

This guide covers deploying your ROS 2 Jazzy differential drive robot from simulation to real hardware using Orange Pi 5, RPLidar C1, encoder motors, and Arduino Nano.

## Table of Contents
1. [Hardware Components](#hardware-components)
2. [Ubuntu & ROS 2 Installation on Orange Pi 5](#ubuntu--ros-2-installation-on-orange-pi-5)
3. [System Setup & Configuration](#system-setup--configuration)
4. [Network Configuration](#network-configuration)
5. [ROS 2 Installation & Package Setup](#ros-2-installation--package-setup)
6. [Hardware Connections & Wiring](#hardware-connections--wiring)
7. [Arduino Nano Motor Controller Setup](#arduino-nano-motor-controller-setup)
8. [Testing & Validation](#testing--validation)
9. [Distributed ROS 2 Setup](#distributed-ros-2-setup)
10. [Troubleshooting](#troubleshooting)

---

## Hardware Components

### Your Hardware List:
- **Orange Pi 5** (Rockchip RK3588S SoC)
- **RPLidar C1** 2D laser scanner
- **Encoder Motors** (differential drive - 2 motors)
- **Arduino Nano** (motor driver interface)
- **Motor Driver** (L298N or similar - assumed)
- **Power Supply** (12V for motors, 5V for Orange Pi 5 & Arduino)
- **Development Laptop** (Ubuntu 24.04 with ROS 2 Jazzy)

---

## Ubuntu & ROS 2 Installation on Orange Pi 5

### Recommended OS Configuration

**✅ RECOMMENDED: Ubuntu 22.04 LTS (Jammy Jellyfish) for Orange Pi 5**

**Why Ubuntu 22.04 instead of 24.04?**

| Aspect | Ubuntu 22.04 LTS | Ubuntu 24.04 LTS |
|--------|------------------|------------------|
| **ROS 2 Support** | ✅ Humble (LTS, tier 1) | ✅ Jazzy (latest) |
| **Orange Pi 5 Support** | ✅ Excellent (official images) | ⚠️ Limited (bleeding edge) |
| **ARM64 Stability** | ✅ Mature | ⚠️ New |
| **LTS Support** | Until 2027 | Until 2029 |
| **Rockchip Drivers** | ✅ Well-tested | ⚠️ Newer, less tested |
| **Community Support** | ✅ Large | Growing |

### Strategy 1 (RECOMMENDED): Two-ROS-Version Approach

**Most compatible and reliable setup:**

#### On Orange Pi 5:
- **OS:** Ubuntu 22.04 LTS (Jammy)
- **ROS 2:** Humble Hawksbill (LTS)
  - Long-term support until May 2027
  - Excellent ARM64/Rockchip stability
  - All required packages available

#### On Your Laptop:
- **OS:** Ubuntu 24.04 LTS (Noble) - Keep as is
- **ROS 2:** Jazzy Jalisco - Keep as is
  - Continue development in simulation
  - Use for visualization (RViz2) and mission planning

#### Compatibility:
- ✅ **ROS 2 Humble and Jazzy can communicate over network**
- ✅ All message types are compatible (sensor_msgs/LaserScan, geometry_msgs/Twist, etc.)
- ✅ DDS middleware handles version differences automatically
- ⚠️ Just ensure both use the same DDS (FastDDS or CycloneDDS)

### Strategy 2 (Advanced): Same Version on Both

If you want ROS 2 Jazzy on both machines:

#### Option A: Jazzy on Orange Pi 5 (Advanced)
- Install Ubuntu 24.04 for Orange Pi 5 (community builds)
- Build ROS 2 Jazzy from source on ARM64
- ⚠️ More complex, requires manual driver setup
- ⚠️ May have hardware compatibility issues

#### Option B: Humble on Laptop (Downgrade)
- Reinstall Ubuntu 22.04 on laptop
- Use ROS 2 Humble for development
- ⚠️ Loses latest features, more work

**Recommendation: Use Strategy 1** - Most stable and battle-tested approach.

---

## System Setup & Configuration

### Step 1: Download Orange Pi 5 Ubuntu Image

Download the **official Orange Pi 5 Ubuntu 22.04 image**:

```bash
# Visit Orange Pi official website
http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-5.html

# Or use Armbian (community-supported, very stable)
https://www.armbian.com/orange-pi-5/

# Recommended: Armbian 22.04 LTS with Linux 6.1+ kernel
```

**Recommended Image:** 
- **Armbian 23.11+ with Ubuntu 22.04 (Jammy) base**
- Kernel 6.1 LTS (Rockchip-optimized)
- Full hardware support for RK3588S

### Step 2: Flash the Image to microSD or eMMC

```bash
# On your laptop, use balenaEtcher or dd
# Download from: https://www.balena.io/etcher/

# Alternative: Using dd (Linux)
sudo dd if=Armbian_*.img of=/dev/sdX bs=4M status=progress
sync
```

**Storage Recommendations:**
- **Development/Testing:** microSD card (64GB+ UHS-I U3)
- **Production:** eMMC module (32GB+) - Much faster and more reliable

### Step 3: First Boot Configuration

```bash
# 1. Insert SD card into Orange Pi 5
# 2. Connect HDMI, keyboard, mouse, ethernet
# 3. Power on

# On first boot, you'll set:
# - Username: robot (or your choice)
# - Password: [secure password]
# - Timezone: Your timezone
# - Locale: en_US.UTF-8

# After setup, update system
sudo apt update
sudo apt upgrade -y
```

### Step 4: Enable Necessary Hardware Interfaces

```bash
# Enable I2C, SPI, UART for Arduino communication
sudo orangepi-config
# or
sudo armbian-config

# Navigate to: System → Hardware
# Enable:
# - i2c3 (or check your board docs)
# - uart3 (for Arduino serial communication)
# - spidev (if using SPI motor driver)

# Reboot after changes
sudo reboot
```

### Step 5: Install Essential Tools

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    curl \
    wget \
    net-tools \
    ssh \
    vim \
    htop \
    can-utils \
    i2c-tools

# Install USB permissions for RPLIDAR
sudo apt install -y udev
```

---

## Network Configuration

### Configure Wi-Fi on Orange Pi 5

```bash
# Method 1: Using nmcli (NetworkManager CLI)
# List available networks
nmcli device wifi list

# Connect to your Wi-Fi
sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"

# Verify connection
ip addr show wlan0
```

### Set Static IP (Recommended for ROS 2)

```bash
# Edit netplan configuration
sudo nano /etc/netplan/01-netcfg.yaml

# Add this configuration:
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24  # Static IP for Orange Pi
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"

# Apply configuration
sudo netplan apply

# Verify
ip addr show wlan0
ping -c 3 192.168.1.1  # Ping gateway
```

### Configure Laptop Network

On your Ubuntu 24.04 laptop, ensure it's on the same network:

```bash
# If using static IP on laptop too (optional but recommended)
# IP: 192.168.1.101/24
# Gateway: 192.168.1.1

# Test connectivity
ping 192.168.1.100  # Ping Orange Pi
```

### Enable SSH for Remote Access

```bash
# On Orange Pi 5
sudo systemctl enable ssh
sudo systemctl start ssh

# From laptop, test SSH access
ssh robot@192.168.1.100
```

---

## ROS 2 Installation & Package Setup

### Install ROS 2 Humble on Orange Pi 5

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (Desktop or Base)
sudo apt update
sudo apt upgrade -y

# For Orange Pi 5, install ROS 2 Base (no GUI tools needed on robot)
sudo apt install -y ros-humble-ros-base

# Install development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Install Required ROS 2 Packages

```bash
# Install packages for robot operation
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros

# Install RPLidar C1 driver (sllidar_ros2)
sudo apt install -y ros-humble-sllidar-ros2

# Install serial communication packages (for Arduino)
sudo apt install -y \
    ros-humble-serial-driver \
    ros-humble-rosidl-default-runtime

# Source ROS 2 in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Clone and Build Your Robot Package

```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone your repository
git clone https://github.com/MianIdrees/my_bot.git

# Install dependencies
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
cd ~/robot_ws
colcon build --symlink-install

# Source the workspace
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Hardware Connections & Wiring

### RPLidar C1 Connection

```
RPLidar C1 → Orange Pi 5
━━━━━━━━━━━━━━━━━━━━━━━━━
USB Cable → USB 3.0 Port (any available port)

Notes:
- RPLidar C1 uses USB 2.0 interface
- Default device: /dev/ttyUSB0
- Baud rate: 460800 (configured in launch file)
```

### Orange Pi 5 → Arduino Nano Connection

```
Orange Pi 5 UART → Arduino Nano
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GPIO Pin 8 (TX) → Arduino RX (Pin 0)
GPIO Pin 10 (RX) → Arduino TX (Pin 1)
GND → GND

Alternative: USB Serial Connection
Orange Pi USB → Arduino Nano USB (Mini-B)
Device: /dev/ttyACM0 or /dev/ttyUSB1
```

### Arduino Nano → Motor Driver (L298N) Connection

```
Arduino → L298N Motor Driver → Motors
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Arduino D3 (PWM) → ENA (Motor A Speed)
Arduino D4 → IN1 (Motor A Direction 1)
Arduino D5 → IN2 (Motor A Direction 2)

Arduino D6 (PWM) → ENB (Motor B Speed)
Arduino D7 → IN3 (Motor B Direction 1)
Arduino D8 → IN4 (Motor B Direction 2)

Arduino GND → L298N GND
Arduino 5V → L298N 5V (Logic)

Power Supply (12V) → L298N 12V, GND
L298N OUT1, OUT2 → Left Motor
L298N OUT3, OUT4 → Right Motor
```

### Motor Encoders Connection

```
Encoder Connections (if available):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Left Encoder A → Arduino D2 (INT0)
Left Encoder B → Arduino D9
Right Encoder A → Arduino D3 (INT1)
Right Encoder B → Arduino D10

Encoder VCC → Arduino 5V
Encoder GND → Arduino GND
```

### Power Distribution

```
Power Requirements:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Motors: 12V @ 2-3A (peak)
Orange Pi 5: 5V @ 4A (USB-C PD)
RPLidar C1: 5V (via USB from Orange Pi)
Arduino Nano: 5V (via USB from Orange Pi or external)

Recommended Setup:
- 12V Battery or Power Supply (for motors)
- DC-DC Buck Converter (12V → 5V @ 5A) for Orange Pi
- L298N powered directly from 12V supply
- Arduino powered via USB from Orange Pi
```

### Complete Wiring Diagram Summary

```
                    ┌─────────────────┐
                    │   Orange Pi 5   │
                    │   (Ubuntu 22.04)│
                    │   ROS 2 Humble  │
                    └────────┬────────┘
                             │
            ┌────────────────┼────────────────┐
            │                │                │
         USB│             UART│             WiFi│
            │                │                │
    ┌───────▼──────┐  ┌─────▼──────┐   ┌────▼──────┐
    │ RPLidar C1   │  │Arduino Nano│   │  Laptop   │
    │              │  │            │   │(Viz/Plan) │
    └──────────────┘  └─────┬──────┘   └───────────┘
                            │
                         GPIO│(D3-D8)
                            │
                    ┌───────▼───────┐
                    │ L298N Driver  │
                    └───┬───────┬───┘
                        │       │
                   ┌────▼──┐ ┌──▼────┐
                   │Motor │ │ Motor │
                   │Left  │ │ Right │
                   └──────┘ └───────┘
```

---

## Arduino Nano Motor Controller Setup

### Arduino Code for Differential Drive Control

Create this Arduino sketch to receive Twist commands via serial and control motors:

```cpp
// File: differential_drive_controller.ino
// Arduino Nano Motor Controller for ROS 2
// Receives Twist commands via Serial, controls L298N driver

// Motor pins
#define ENA 3   // PWM pin for left motor speed
#define IN1 4   // Left motor direction 1
#define IN2 5   // Left motor direction 2

#define ENB 6   // PWM pin for right motor speed
#define IN3 7   // Right motor direction 1
#define IN4 8   // Right motor direction 2

// Encoder pins (optional)
#define LEFT_ENC_A 2   // Interrupt pin
#define LEFT_ENC_B 9
#define RIGHT_ENC_A 3  // Interrupt pin
#define RIGHT_ENC_B 10

// Robot physical parameters
const float WHEEL_SEPARATION = 0.35;  // meters (from your URDF)
const float WHEEL_RADIUS = 0.05;       // meters

// Command timeout (milliseconds)
const unsigned long CMD_TIMEOUT = 1000;
unsigned long lastCmdTime = 0;

// Current velocities
float linearVel = 0.0;
float angularVel = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // Match this with ROS 2 serial node
  
  // Configure motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  // Optional: Setup encoder interrupts
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  Serial.println("Arduino Differential Drive Controller Ready");
}

void loop() {
  // Check for incoming serial data (Twist messages)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    lastCmdTime = millis();
  }
  
  // Safety: Stop motors if no command received for timeout period
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    stopMotors();
  }
  
  // Execute motor control
  driveMotors(linearVel, angularVel);
  
  delay(10);  // 100Hz loop
}

void parseCommand(String cmd) {
  // Expected format: "L:linear_vel,A:angular_vel"
  // Example: "L:0.5,A:0.2"
  
  int linIdx = cmd.indexOf("L:");
  int angIdx = cmd.indexOf("A:");
  
  if (linIdx >= 0 && angIdx >= 0) {
    String linStr = cmd.substring(linIdx + 2, angIdx);
    String angStr = cmd.substring(angIdx + 2);
    
    linearVel = linStr.toFloat();
    angularVel = angStr.toFloat();
  }
}

void driveMotors(float linear, float angular) {
  // Convert twist to wheel velocities (differential drive kinematics)
  float leftVel = linear - (angular * WHEEL_SEPARATION / 2.0);
  float rightVel = linear + (angular * WHEEL_SEPARATION / 2.0);
  
  // Convert to PWM values (0-255)
  // Assuming max velocity of 0.5 m/s (from your specs)
  int leftPWM = constrain(abs(leftVel) * 255.0 / 0.5, 0, 255);
  int rightPWM = constrain(abs(rightVel) * 255.0 / 0.5, 0, 255);
  
  // Set left motor
  if (leftVel > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (leftVel < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, leftPWM);
  
  // Set right motor
  if (rightVel > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (rightVel < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, rightPWM);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  linearVel = 0.0;
  angularVel = 0.0;
}
```

### Upload Arduino Sketch

```bash
# On your laptop (before connecting to Orange Pi):
# 1. Install Arduino IDE
sudo apt install -y arduino

# 2. Connect Arduino Nano via USB
# 3. Open Arduino IDE
arduino

# 4. Select Board: Tools → Board → Arduino Nano
# 5. Select Processor: Tools → Processor → ATmega328P (Old Bootloader)
# 6. Select Port: Tools → Port → /dev/ttyUSB0 or /dev/ttyACM0
# 7. Upload the sketch

# Alternative: Using arduino-cli
sudo apt install -y arduino-cli
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:nano differential_drive_controller.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:nano differential_drive_controller.ino
```

### Create ROS 2 Serial Bridge Node

Create a Python node to bridge ROS 2 Twist messages to Arduino serial:

```bash
# Create the node file
mkdir -p ~/robot_ws/src/my_bot/scripts
cd ~/robot_ws/src/my_bot/scripts
```

Create `serial_motor_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialMotorController(Node):
    def __init__(self):
        super().__init__('serial_motor_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Serial Motor Controller Ready')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to serial command"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Format: "L:linear,A:angular\n"
        command = f"L:{linear:.3f},A:{angular:.3f}\n"
        
        try:
            self.serial.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def destroy_node(self):
        """Clean up serial connection"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable:

```bash
chmod +x ~/robot_ws/src/my_bot/scripts/serial_motor_controller.py

# Rebuild workspace
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Testing & Validation

### Test 1: RPLidar C1 Connection

```bash
# On Orange Pi 5, verify USB device
ls -l /dev/ttyUSB*

# Should show: /dev/ttyUSB0

# Set permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# Test RPLidar
ros2 launch my_bot rplidar.launch.py

# In another terminal, check scan data
ros2 topic echo /scan --once
```

### Test 2: Arduino Serial Communication

```bash
# Verify Arduino is connected
ls -l /dev/ttyACM*

# Should show: /dev/ttyACM0

# Set permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0

# Test serial manually
sudo apt install -y screen
screen /dev/ttyACM0 115200

# Type: L:0.1,A:0.0
# Motors should move forward slowly
# Ctrl+A then K to exit screen
```

### Test 3: Motor Controller Node

```bash
# Terminal 1: Start serial motor controller
ros2 run my_bot serial_motor_controller.py --ros-args -p serial_port:=/dev/ttyACM0

# Terminal 2: Publish test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Motors should move forward
# Press Ctrl+C to stop
```

### Test 4: Complete Robot Launch

Create a launch file for the complete robot:

```bash
cd ~/robot_ws/src/my_bot/launch
```

Create `launch_robot.launch.py`:

```python
"""
Complete robot launch for Orange Pi 5 hardware.
Includes: Robot state publisher, RPLidar, motor controller
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    pkg_path = get_package_share_directory(package_name)
    
    # Launch arguments
    serial_port_lidar = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )
    
    serial_port_arduino = DeclareLaunchArgument(
        'serial_port_arduino',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )
    
    # RPLidar C1
    rplidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port_lidar'),
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen'
    )
    
    # Serial motor controller
    motor_controller = Node(
        package='my_bot',
        executable='serial_motor_controller.py',
        name='serial_motor_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port_arduino'),
            'baud_rate': 115200,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_lidar,
        serial_port_arduino,
        rsp,
        joint_state_publisher,
        rplidar,
        motor_controller,
    ])
```

Update `CMakeLists.txt` to install the Python script:

```cmake
# Add to CMakeLists.txt after the existing install commands
install(PROGRAMS
  scripts/serial_motor_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Rebuild and test:

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Launch complete robot
ros2 launch my_bot launch_robot.launch.py
```

---

## Distributed ROS 2 Setup

### Configure ROS 2 Domain ID (Same on Both Machines)

```bash
# On Orange Pi 5 and Laptop, add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### Configure DDS Discovery (FastDDS)

Create DDS config files for reliable network communication:

**On Orange Pi 5** (`~/fastdds.xml`):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="robot_pi" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                        <leaseDuration>
                            <sec>20</sec>
                        </leaseDuration>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

**On Laptop** (`~/fastdds.xml`):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="laptop_viz" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                        <leaseDuration>
                            <sec>20</sec>
                        </leaseDuration>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

Add to both machines' `~/.bashrc`:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### Test Distributed Setup

**On Orange Pi 5:**

```bash
# Terminal 1: Launch robot hardware
ros2 launch my_bot launch_robot.launch.py
```

**On Laptop:**

```bash
# Terminal 1: Verify topics from Orange Pi
ros2 topic list

# Should see: /scan, /cmd_vel, /tf, /joint_states, etc.

# Terminal 2: Launch RViz for visualization
ros2 launch my_bot launch_real_lidar.launch.py

# Or just RViz
rviz2
```

### Verify Communication

```bash
# On laptop, check scan data from Orange Pi
ros2 topic hz /scan
ros2 topic echo /scan --once

# On laptop, control robot on Orange Pi
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Troubleshooting

### Issue 1: RPLidar Not Detected

```bash
# Check USB connection
lsusb | grep -i lidar
# Should show: "Silicon Labs CP210x"

# Check device
ls -l /dev/ttyUSB*

# If not found, check kernel messages
dmesg | grep ttyUSB

# Fix permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# Create udev rule for persistent permissions
sudo nano /etc/udev/rules.d/99-rplidar.rules

# Add:
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reboot
sudo reboot
```

### Issue 2: Arduino Not Detected

```bash
# Check connection
lsusb | grep -i arduino

# Check device
ls -l /dev/ttyACM*

# Fix permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0

# Create udev rule
sudo nano /etc/udev/rules.d/99-arduino.rules

# Add:
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"

# Reload
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Issue 3: Motors Not Responding

**Check Arduino Serial:**

```bash
# Monitor Arduino serial output
sudo apt install -y minicom
minicom -D /dev/ttyACM0 -b 115200

# You should see: "Arduino Differential Drive Controller Ready"
```

**Check Power:**
- Verify 12V power supply to L298N
- Check motor driver LED indicators
- Measure voltage at motor terminals

**Check Wiring:**
- Verify all connections match the wiring diagram
- Check for loose connections
- Test motors directly with power supply

### Issue 4: ROS 2 Nodes Not Discovering Each Other

```bash
# Check ROS_DOMAIN_ID on both machines
echo $ROS_DOMAIN_ID
# Should be the same (e.g., 42)

# Check network connectivity
ping 192.168.1.100  # From laptop to Orange Pi
ping 192.168.1.101  # From Orange Pi to laptop

# Check firewall (disable for testing)
sudo ufw disable

# Check ROS 2 multicast
ros2 multicast receive
ros2 multicast send

# Use ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Check DDS vendor
echo $RMW_IMPLEMENTATION
# Should be: rmw_fastrtps_cpp or rmw_cyclonedds_cpp
```

### Issue 5: High CPU Usage on Orange Pi 5

```bash
# Monitor CPU usage
htop

# Reduce ROS 2 QoS for better performance
# Edit launch file, add to node parameters:
# qos_overrides = {
#     '/cmd_vel': {
#         'reliability': 'best_effort',
#         'durability': 'volatile',
#         'history': 'keep_last',
#         'depth': 1
#     }
# }

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
```

### Issue 6: Lag in Visualization

On your laptop, ensure use_sim_time is false when connecting to real hardware:

```bash
# Edit RViz launch or start with parameter
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false
```

---

## Performance Optimization

### Orange Pi 5 Optimizations

```bash
# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make permanent
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils

# Enable swap (if needed)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### ROS 2 Optimizations

```bash
# Use CycloneDDS (sometimes more efficient than FastDDS)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Set as default in ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

---

## Next Steps

### 1. Add Odometry Publishing
- Implement encoder reading on Arduino
- Publish odometry data to ROS 2
- Use for better localization

### 2. Add Navigation Stack
- Install Nav2 on Orange Pi 5
- Configure SLAM (slam_toolbox or cartographer)
- Implement autonomous navigation

### 3. Add IMU Sensor
- Connect MPU6050 or BNO055 to Orange Pi I2C
- Publish IMU data for sensor fusion
- Improve odometry accuracy

### 4. Create Systemd Services
Make robot start automatically on boot:

```bash
# Create service file
sudo nano /etc/systemd/system/my_robot.service

# Add:
[Unit]
Description=My Robot ROS 2 Launch
After=network.target

[Service]
Type=simple
User=robot
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/robot/robot_ws/install/setup.bash && ros2 launch my_bot launch_robot.launch.py"
Restart=always

[Install]
WantedBy=multi-user.target

# Enable service
sudo systemctl daemon-reload
sudo systemctl enable my_robot.service
sudo systemctl start my_robot.service
```

---

## Summary

### Recommended Configuration:

| Component | Specification |
|-----------|--------------|
| **Orange Pi 5 OS** | Ubuntu 22.04 LTS (Armbian) |
| **Orange Pi 5 ROS 2** | Humble Hawksbill (LTS) |
| **Laptop OS** | Ubuntu 24.04 LTS (Current) |
| **Laptop ROS 2** | Jazzy Jalisco (Current) |
| **DDS** | FastDDS or CycloneDDS |
| **Domain ID** | 42 (same on both) |
| **Network** | Static IPs on same subnet |

### Key Benefits:
✅ **Stability:** Ubuntu 22.04 is mature and well-tested on ARM64  
✅ **Compatibility:** ROS 2 Humble has excellent hardware support  
✅ **Long-term Support:** Both Ubuntu 22.04 and ROS 2 Humble supported until 2027  
✅ **Cross-version:** Humble and Jazzy communicate seamlessly over network  
✅ **Orange Pi Support:** Official images and drivers available  
✅ **Community:** Large user base for troubleshooting  

### Quick Start Checklist:

- [ ] Flash Ubuntu 22.04 to Orange Pi 5 SD card
- [ ] Boot Orange Pi 5, configure user and network
- [ ] Set static IP for Orange Pi 5 (e.g., 192.168.1.100)
- [ ] Install ROS 2 Humble on Orange Pi 5
- [ ] Clone and build my_bot package
- [ ] Connect and test RPLidar C1
- [ ] Upload Arduino sketch and connect to Orange Pi
- [ ] Wire motor driver and test motors
- [ ] Configure distributed ROS 2 (domain ID, DDS)
- [ ] Test laptop ↔ Orange Pi communication
- [ ] Launch robot and visualize on laptop
- [ ] Drive robot with teleop from laptop

Good luck with your deployment! 🤖
