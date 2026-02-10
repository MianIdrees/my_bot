# System Architecture Diagrams

This document provides visual representations of the robot's hardware and software architecture.

## Overall System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         DEVELOPMENT LAPTOP                          │
│                    (Ubuntu 24.04 + ROS 2 Jazzy)                     │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────────┐  │
│  │   RViz2      │  │   Gazebo     │  │  teleop_twist_keyboard  │  │
│  │ (Visualization)│  │ (Simulation) │  │   (Manual Control)      │  │
│  └──────────────┘  └──────────────┘  └─────────────────────────┘  │
│         │                 │                        │                │
│         └─────────────────┴────────────────────────┘                │
│                           │                                         │
│                   ROS 2 Topics/TF                                   │
│                           │                                         │
└───────────────────────────┼─────────────────────────────────────────┘
                            │
                      WiFi Network
                   (ROS_DOMAIN_ID=42)
                            │
┌───────────────────────────┼─────────────────────────────────────────┐
│                           │                                         │
│                   ROS 2 DDS Bridge                                  │
│                           │                                         │
│  ┌────────────────────────┴───────────────────────────────────┐    │
│  │                     ORANGE PI 5                             │    │
│  │              (Ubuntu 22.04 + ROS 2 Humble)                  │    │
│  │                                                             │    │
│  │  ┌─────────────────┐  ┌──────────────┐  ┌───────────────┐ │    │
│  │  │ Robot State     │  │ RPLidar C1   │  │ Serial Motor  │ │    │
│  │  │ Publisher       │  │ Node         │  │ Controller    │ │    │
│  │  │ (URDF/TF)       │  │ (sllidar)    │  │ (Python)      │ │    │
│  │  └────────┬────────┘  └──────┬───────┘  └───────┬───────┘ │    │
│  │           │                  │                   │          │    │
│  └───────────┼──────────────────┼───────────────────┼──────────┘    │
│              │                  │                   │                │
│              │            USB   │                   │ Serial         │
│              │            │     │                   │ (115200)       │
└──────────────┼────────────┼─────┼───────────────────┼────────────────┘
               │            │     │                   │
               │      ┌─────▼─────▼───────┐     ┌─────▼──────────┐
               │      │   RPLidar C1      │     │ Arduino Nano   │
               │      │  (Laser Scanner)  │     │ (Motor Driver) │
               │      └───────────────────┘     └────────┬───────┘
               │                                         │
               │                                    GPIO (PWM)
               │                                         │
               │                              ┌──────────▼──────────┐
               │                              │  L298N Motor Driver │
               │                              │  (12V H-Bridge)     │
               │                              └──────┬──────┬───────┘
               │                                     │      │
               │                              ┌──────▼──┐ ┌─▼──────┐
               │                              │  Left   │ │ Right  │
               └──────────────────────────────│  Motor  │ │ Motor  │
                                              └─────────┘ └────────┘
```

## Data Flow Diagram

```
LAPTOP                         NETWORK                      ORANGE PI 5
══════                         ═══════                      ═══════════

┌─────────────┐
│ Keyboard    │
│ Teleoperation│
└──────┬──────┘
       │
       │ Twist (cmd_vel)
       ▼
┌─────────────┐              ┌──────────┐              ┌──────────────┐
│   RViz2     │◄─────────────┤   WiFi   │◄─────────────┤ Robot State  │
│ Visualization│  LaserScan   │   DDS    │    TF Tree   │  Publisher   │
└─────────────┘  TF, Odom    └────┬─────┘              └──────────────┘
                                   │
                                   │ cmd_vel
                                   ▼
                              ┌────────────────┐
                              │ Serial Motor   │
                              │ Controller     │
                              └────────┬───────┘
                                       │
                                       │ "L:0.5,A:0.0"
                                       │ (Serial String)
                                       ▼
                              ┌────────────────┐
                              │ Arduino Nano   │
                              │ Differential   │
                              │ Drive Control  │
                              └────────┬───────┘
                                       │
                                       │ PWM Signals
                                       ▼
                              ┌────────────────┐
                              │ L298N Driver   │
                              │ Motor A, B     │
                              └────────┬───────┘
                                       │
                                       │ 12V DC
                                       ▼
                              ┌────────────────┐
                              │ Encoder Motors │
                              └────────────────┘
```

## Hardware Wiring Schematic

```
╔════════════════════════════════════════════════════════════════════╗
║                         POWER DISTRIBUTION                         ║
╚════════════════════════════════════════════════════════════════════╝

                        ┌───────────────┐
                        │ 12V Battery   │
                        │ or PSU        │
                        │ (3A minimum)  │
                        └───┬───────┬───┘
                            │       │
                        ┌───▼───┐   │
                        │DC-DC  │   │
                        │Buck   │   │
                        │12V→5V │   │
                        └───┬───┘   │
                            │       │
                        ┌───▼───┐   │
                        │Orange │   │
                        │Pi 5   │   │
                        │5V/4A  │   │
                        └───────┘   │
                                    │
                              ┌─────▼──────┐
                              │  L298N     │
                              │  12V Input │
                              └────────────┘

╔════════════════════════════════════════════════════════════════════╗
║                      SIGNAL CONNECTIONS                            ║
╚════════════════════════════════════════════════════════════════════╝

 Orange Pi 5                                        Arduino Nano
┌────────────┐                                     ┌────────────────┐
│            │                                     │                │
│  USB 3.0───┼─────────USB Cable──────────────────┤  USB Mini-B    │
│            │                                     │                │
│            │                                     │  D3 (PWM) ─────┼──┐
│            │                                     │  D4 ───────────┼──┤
│            │                                     │  D5 ───────────┼──┤
│            │         ┌────────────────┐          │  D6 (PWM) ─────┼──┤
│  USB 3.0───┼─────────┤  RPLidar C1    │          │  D7 ───────────┼──┤
│            │   USB   │  Laser Scanner │          │  D8 ───────────┼──┤
└────────────┘         └────────────────┘          │  GND ──────────┼──┤
                                                   │  5V ───────────┼──┤
                                                   └────────────────┘  │
                                                                       │
                                                   ┌────────────────┐  │
                                                   │    L298N       │  │
                                                   │  Motor Driver  │  │
                                                   │                │  │
                                                   │  ENA ◄─────────┼──┘
                                                   │  IN1 ◄─────────┼──┘
                                                   │  IN2 ◄─────────┼──┘
                                                   │  ENB ◄─────────┼──┘
                                                   │  IN3 ◄─────────┼──┘
                                                   │  IN4 ◄─────────┼──┘
                                                   │  GND ◄─────────┼──┘
                                                   │  5V  ◄─────────┼──┘
                                                   │                │
                                                   │  OUT1, OUT2 ───┼──► Left Motor
                                                   │  OUT3, OUT4 ───┼──► Right Motor
                                                   └────────────────┘
```

## Pin Configuration Table

### Arduino Nano to L298N

| Arduino Pin | Function | L298N Pin | Description |
|------------|----------|-----------|-------------|
| D3 (PWM)   | Output   | ENA       | Left motor speed (0-255) |
| D4         | Output   | IN1       | Left motor direction bit 1 |
| D5         | Output   | IN2       | Left motor direction bit 2 |
| D6 (PWM)   | Output   | ENB       | Right motor speed (0-255) |
| D7         | Output   | IN3       | Right motor direction bit 1 |
| D8         | Output   | IN4       | Right motor direction bit 2 |
| GND        | Ground   | GND       | Common ground |
| 5V         | Power    | 5V        | Logic level power |

### Optional: Encoder Connections

| Encoder Signal | Arduino Pin | Description |
|---------------|-------------|-------------|
| Left Encoder A | D2 (INT0)  | Interrupt-capable pin |
| Left Encoder B | D9         | Direction sensing |
| Right Encoder A | D3 (INT1) | Interrupt-capable pin |
| Right Encoder B | D10       | Direction sensing |
| Encoder VCC    | 5V         | Power supply |
| Encoder GND    | GND        | Ground |

## Network Configuration

```
┌──────────────────────────────────────────────────────────────┐
│                    Home WiFi Router                          │
│                    Gateway: 192.168.1.1                      │
│                    Subnet: 192.168.1.0/24                    │
└───────────────┬──────────────────────┬───────────────────────┘
                │                      │
                │ WiFi                 │ WiFi/Ethernet
                │                      │
        ┌───────▼──────────┐   ┌──────▼────────────┐
        │  Orange Pi 5     │   │  Laptop           │
        │  192.168.1.100   │   │  192.168.1.101    │
        │  (Static IP)     │   │  (DHCP or Static) │
        │                  │   │                   │
        │  ROS 2 Humble    │   │  ROS 2 Jazzy      │
        │  ROS_DOMAIN_ID=42│   │  ROS_DOMAIN_ID=42 │
        └──────────────────┘   └───────────────────┘
                │                      │
        ┌───────▼──────────┐   ┌──────▼────────────┐
        │ Topics Published:│   │ Topics Published: │
        │ - /scan          │   │ - /cmd_vel        │
        │ - /tf            │   │                   │
        │ - /joint_states  │   │ Topics Subscribed:│
        │                  │   │ - /scan           │
        │ Topics Subscribed│   │ - /tf             │
        │ - /cmd_vel       │   │ - /joint_states   │
        └──────────────────┘   └───────────────────┘
```

## Software Stack Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    LAPTOP (Ubuntu 24.04)                    │
├─────────────────────────────────────────────────────────────┤
│  Application Layer: RViz2, Gazebo, Nav2, SLAM               │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 Jazzy: Publishers, Subscribers, Services             │
├─────────────────────────────────────────────────────────────┤
│  DDS (FastDDS/CycloneDDS): Data Distribution                │
├─────────────────────────────────────────────────────────────┤
│  Network Layer: TCP/IP, UDP Multicast                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ WiFi/Ethernet
                              │
┌─────────────────────────────▼───────────────────────────────┐
│              ORANGE PI 5 (Ubuntu 22.04)                     │
├─────────────────────────────────────────────────────────────┤
│  Hardware Interface: sllidar_ros2, serial_motor_controller  │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 Humble: robot_state_publisher, joint_state_publisher │
├─────────────────────────────────────────────────────────────┤
│  DDS (FastDDS/CycloneDDS): Data Distribution                │
├─────────────────────────────────────────────────────────────┤
│  USB & Serial Drivers: CP210x (RPLidar), CH340 (Arduino)    │
├─────────────────────────────────────────────────────────────┤
│  Linux Kernel: Device drivers, USB stack                    │
└─────────────────────────────────────────────────────────────┘
                       │                  │
                  USB  │                  │ Serial
                       │                  │
           ┌───────────▼──────┐     ┌────▼────────────┐
           │  RPLidar C1      │     │  Arduino Nano   │
           │  (Sensor)        │     │  (Microcontroller)│
           └──────────────────┘     └─────────────────┘
                                            │
                                         GPIO
                                            │
                                    ┌───────▼────────┐
                                    │  L298N Driver  │
                                    │  (H-Bridge)    │
                                    └───────┬────────┘
                                            │
                                      ┌─────▼─────┐
                                      │  Motors   │
                                      └───────────┘
```

## Message Flow Example: Teleoperation

```
Sequence: User drives robot forward

1. USER presses 'i' key
   ↓
2. teleop_twist_keyboard (Laptop)
   Publishes: Twist{linear.x=0.5, angular.z=0.0} → /cmd_vel
   ↓
3. DDS/FastDDS (Laptop)
   Serializes message, sends via UDP multicast
   ↓
4. WiFi Network
   Transmits UDP packets: 192.168.1.101 → 192.168.1.100
   ↓
5. DDS/FastDDS (Orange Pi)
   Receives, deserializes message
   ↓
6. serial_motor_controller.py (Orange Pi)
   Subscribes to /cmd_vel
   Converts to: "L:0.500,A:0.000\n"
   ↓
7. Serial Port /dev/ttyACM0
   Transmits at 115200 baud
   ↓
8. Arduino Nano Serial RX
   Parses: linearVel=0.500, angularVel=0.000
   ↓
9. driveMotors() function
   Calculates: leftVel=0.500, rightVel=0.500
   Converts to PWM: leftPWM=255, rightPWM=255
   ↓
10. Arduino GPIO Pins
    ENA=255 (HIGH), ENB=255 (HIGH)
    IN1=HIGH, IN2=LOW (forward)
    IN3=HIGH, IN4=LOW (forward)
    ↓
11. L298N Motor Driver
    OUT1-OUT2: +12V (left motor forward)
    OUT3-OUT4: +12V (right motor forward)
    ↓
12. DC Motors
    Both spin forward at full speed
    ↓
13. RESULT: Robot moves forward!
```

## Troubleshooting Flow Chart

```
                    ┌────────────────┐
                    │  Robot not     │
                    │  responding?   │
                    └────────┬───────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
        ┌─────▼─────┐               ┌──────▼──────┐
        │ Is Orange │               │ Are topics  │
        │ Pi 5 on?  │               │ visible?    │
        └─────┬─────┘               └──────┬──────┘
              │                             │
        YES   │   NO                  YES   │   NO
              │                             │
    ┌─────────▼────┐              ┌─────────▼─────────┐
    │ Check USB    │              │ Check network:    │
    │ connections  │              │ - Same subnet?    │
    └─────┬────────┘              │ - Same DOMAIN_ID? │
          │                       │ - Firewall off?   │
    ┌─────▼──────────┐            └─────────┬─────────┘
    │ RPLidar LED on?│                      │
    └─────┬──────────┘                      │
          │                             ┌───▼───────┐
    YES   │   NO                        │ Fix and   │
          │                             │ retry     │
    ┌─────▼───────────┐                 └───────────┘
    │ Motors powered? │
    │ (12V supply)    │
    └─────┬───────────┘
          │
    YES   │   NO
          │
    ┌─────▼────────────┐
    │ Test with serial:│
    │ screen /dev/ttyACM0│
    │ Send: L:0.1,A:0.0│
    └─────┬────────────┘
          │
    WORKS │   FAILS
          │
    ┌─────▼──────────┐         ┌──────────────┐
    │ Issue is in    │         │ Check Arduino│
    │ ROS 2 bridge   │         │ - Uploaded?  │
    │ Check logs     │         │ - Wiring?    │
    └────────────────┘         │ - Motor test?│
                               └──────────────┘
```

---

**Note:** All diagrams are text-based for maximum compatibility. For interactive diagrams, consider using tools like draw.io or PlantUML.
