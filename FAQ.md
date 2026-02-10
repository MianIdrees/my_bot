# Frequently Asked Questions (FAQ)

## General Questions

### Q: Why use different ROS 2 versions on laptop and Orange Pi 5?

**A:** This is the recommended approach for maximum stability and compatibility:

- **Orange Pi 5 (Humble):** Ubuntu 22.04 and ROS 2 Humble have excellent ARM64 support, mature drivers for Rockchip hardware, and long-term support until 2027.
- **Laptop (Jazzy):** Keep your current development environment with the latest features.
- **Compatibility:** ROS 2 Humble and Jazzy communicate perfectly over network using DDS. All message types are compatible.

Alternative: You could use Humble on both, but you'd lose the latest features on your laptop. Or Jazzy on both, but Orange Pi 5 support is less mature.

### Q: Can I use Raspberry Pi 4 instead of Orange Pi 5?

**A:** Yes, but Orange Pi 5 is recommended:

| Feature | Orange Pi 5 | Raspberry Pi 4B |
|---------|-------------|-----------------|
| CPU | 8-core RK3588S (4x A76 + 4x A55) | 4-core Cortex-A72 |
| RAM | Up to 32GB | Up to 8GB |
| USB 3.0 | 4 ports | 2 ports |
| PCIe | M.2 slot available | None |
| Performance | ~3x faster | Baseline |
| Price | Similar | $35-75 |

For this robot, either will work, but Orange Pi 5 has more headroom for advanced features like SLAM and Nav2.

### Q: Do I need encoders on the motors?

**A:** Not required, but highly recommended:

- **Without encoders:** Open-loop control, robot may drift, no odometry feedback
- **With encoders:** Closed-loop control, accurate odometry, better navigation

The Arduino code includes encoder support. If you add them later, just uncomment the encoder feedback section.

### Q: What motor driver can I use besides L298N?

**A:** Several options:

| Driver | Voltage | Current | Pros | Cons |
|--------|---------|---------|------|------|
| L298N | 5-35V | 2A per channel | Cheap, easy | Low efficiency, heats up |
| TB6612FNG | 4.5-13.5V | 1.2A per channel | Efficient, small | Lower current |
| BTS7960 | 5.5-27V | 43A per channel | High power | Overkill for small robot |
| DRV8833 | 2.7-10.8V | 1.5A per channel | Very efficient | Lower voltage |

For this robot (assuming ~2A motors), L298N or TB6612FNG are good choices. Adjust Arduino pins if using different driver.

## Orange Pi 5 Questions

### Q: Can I use eMMC instead of microSD?

**A:** Yes, highly recommended for production use:

- **microSD:** Easier to flash, good for development
- **eMMC:** Faster (100+ MB/s), more reliable, better for production

Flash eMMC the same way as microSD. Orange Pi 5 can boot from either.

### Q: Orange Pi 5 Plus vs Orange Pi 5?

**A:** Orange Pi 5 is sufficient:

- **Orange Pi 5:** 8-core RK3588S, up to 32GB RAM - Perfect for this robot
- **Orange Pi 5 Plus:** Same SoC, M.2 NVMe slot, more I/O - Only if you need storage expansion

Save money and use regular Orange Pi 5 unless you specifically need NVMe storage.

### Q: What power supply do I need for Orange Pi 5?

**A:** USB-C PD (Power Delivery) 5V/4A minimum:

- Official Orange Pi 5 power adapter (recommended)
- USB-C PD laptop charger (must support 5V/3A minimum)
- DC-DC buck converter from battery (12V → 5V @ 4A)

For mobile robot: Use DC-DC buck converter from main battery.

### Q: Can I use Ubuntu 24.04 on Orange Pi 5?

**A:** Possible but not recommended:

- Limited official support
- Some drivers may not work
- More complex setup

Stick with Ubuntu 22.04 (Jammy) for best results. It's supported until 2027.

## Network & ROS 2 Questions

### Q: What's ROS_DOMAIN_ID and why does it matter?

**A:** ROS_DOMAIN_ID isolates ROS 2 networks:

- Range: 0-101 (for localhost), 0-232 (for network)
- Default: 0
- **Why set it:** Prevents interference from other ROS 2 systems on same network

Set the same ID on both machines (e.g., 42) so they can discover each other.

```bash
export ROS_DOMAIN_ID=42
```

### Q: WiFi vs Ethernet for robot communication?

**A:**

| Connection | Latency | Bandwidth | Reliability | Setup |
|------------|---------|-----------|-------------|-------|
| WiFi | 5-20ms | 50-300 Mbps | Good | Easy |
| Ethernet | 1-5ms | 1000 Mbps | Excellent | Requires cable |

For development: WiFi is fine
For production/autonomous: Ethernet is better (lower latency)
For truly mobile: WiFi is only option

### Q: Do I need to configure DDS?

**A:** Usually not required, but can help:

- Default DDS (FastDDS) works for most cases
- Create `fastdds.xml` only if having discovery issues
- Can switch to CycloneDDS if FastDDS has problems:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

### Q: Can I visualize the robot without RViz2?

**A:** Yes, several options:

1. **Foxglove Studio** - Web-based, beautiful UI
2. **PlotJuggler** - For plotting topics
3. **rqt_graph** - Node/topic visualization
4. **Web Video Server** - Stream camera over HTTP

But RViz2 is the standard and most feature-complete.

## Arduino & Motor Control Questions

### Q: Can I use Arduino Uno instead of Nano?

**A:** Yes, they're nearly identical:

- Same ATmega328P microcontroller
- Same code works on both
- Nano is smaller, better for embedded projects
- Uno has easier USB connector (USB-B)

Code works on both without changes.

### Q: Serial communication is unreliable, what to do?

**A:** Several solutions:

1. **Check baud rate:** Must match on both sides (115200)
2. **Add ground wire:** Arduino GND → Orange Pi GND
3. **Use hardware serial:** Already used in code
4. **Add flow control:** If using UART pins (RX/TX)
5. **Shorten USB cable:** < 1.5m for reliability
6. **Add ferrite beads:** Reduce EMI on USB cable

### Q: Can I use I2C or SPI instead of serial?

**A:** Yes, possible:

- **Serial (Current):** Simplest, works well, 115200 baud is fast enough
- **I2C:** More complex, but good for multiple devices
- **SPI:** Fastest, but requires more wires

Stick with serial unless you have specific reasons to change.

### Q: Motors are too fast, how to slow down?

**A:** Multiple approaches:

1. **Limit max velocity in Arduino:**
   ```cpp
   const float MAX_LINEAR_VEL = 0.3;  // Reduce from 0.5
   ```

2. **Scale in ROS 2 node:**
   ```python
   linear = msg.linear.x * 0.5  # 50% speed
   ```

3. **Use teleop with lower max velocities:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3
   ```

### Q: How to add emergency stop button?

**A:** Add to Arduino:

```cpp
#define ESTOP_PIN 11  // Emergency stop button pin

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  // ... rest of setup
}

void loop() {
  if (digitalRead(ESTOP_PIN) == LOW) {  // Button pressed (active low)
    stopMotors();
    Serial.println("EMERGENCY STOP!");
    while(digitalRead(ESTOP_PIN) == LOW) {
      delay(100);  // Wait for button release
    }
  }
  // ... rest of loop
}
```

Wire: Button between pin 11 and GND.

## RPLidar Questions

### Q: My lidar model is A1/A2, not C1. Will it work?

**A:** Yes, with minor changes:

1. **Change baud rate** in launch file:
   ```python
   # For A1/A2:
   serial_baudrate = '115200'
   
   # For C1 (current):
   serial_baudrate = '460800'
   ```

2. **Change scan mode** (optional):
   ```python
   scan_mode = 'Standard'  # or 'Express' for A2
   ```

That's it! The sllidar_ros2 driver supports all RPLidar models.

### Q: Lidar scan is rotated/upside-down, how to fix?

**A:** Two options:

1. **Physically rotate the sensor** to match URDF orientation

2. **Invert in launch file:**
   ```python
   parameters=[{
       'inverted': True,  # Change to True
       # ... other parameters
   }]
   ```

### Q: Lidar connection keeps dropping

**A:** Troubleshooting steps:

1. **Check USB cable:** Use high-quality USB 2.0 cable < 1m
2. **Check power:** Ensure Orange Pi has adequate power supply
3. **Check USB port:** Try different USB port
4. **Add udev rule:** See HARDWARE_DEPLOYMENT.md
5. **Check kernel messages:**
   ```bash
   dmesg | grep ttyUSB
   ```

## Build & Deployment Questions

### Q: How to auto-start robot on boot?

**A:** Create systemd service:

```bash
sudo nano /etc/systemd/system/myrobot.service
```

Add:
```ini
[Unit]
Description=My Robot Launch
After=network.target

[Service]
Type=simple
User=robot
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/robot/robot_ws/install/setup.bash && ros2 launch my_bot launch_robot.launch.py"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable myrobot.service
sudo systemctl start myrobot.service
```

### Q: How to update robot code remotely?

**A:** Several methods:

1. **SSH + Git** (recommended):
   ```bash
   ssh robot@192.168.1.100
   cd ~/robot_ws/src/my_bot
   git pull
   cd ~/robot_ws
   colcon build --symlink-install
   ```

2. **rsync** (faster for large changes):
   ```bash
   rsync -avz --delete ~/robot_ws/src/my_bot/ robot@192.168.1.100:~/robot_ws/src/my_bot/
   ```

3. **CI/CD pipeline** (advanced):
   - GitHub Actions auto-build on commit
   - Auto-deploy to robot via SSH

### Q: Can I run simulation and real robot simultaneously?

**A:** Yes, with different ROS_DOMAIN_IDs:

Terminal 1 (Simulation):
```bash
export ROS_DOMAIN_ID=0
ros2 launch my_bot launch_sim.launch.py
```

Terminal 2 (Real Robot):
```bash
export ROS_DOMAIN_ID=42
ros2 launch my_bot launch_robot.launch.py
```

They won't interfere because they're on different domains.

## Performance & Optimization Questions

### Q: Orange Pi 5 CPU usage is high (>80%)

**A:** Optimization checklist:

1. **Set performance governor:**
   ```bash
   echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   ```

2. **Disable unnecessary services:**
   ```bash
   sudo systemctl disable bluetooth
   sudo systemctl disable avahi-daemon
   ```

3. **Use CycloneDDS:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

4. **Reduce topic rates:** Lower sensor publish rates

5. **Check thermal throttling:**
   ```bash
   cat /sys/class/thermal/thermal_zone0/temp
   ```
   Add heatsink if > 70°C.

### Q: Visualization is laggy in RViz2

**A:** Solutions:

1. **On laptop, check use_sim_time:**
   ```bash
   ros2 param get /rviz2 use_sim_time
   # Should be 'false' for real robot
   ```

2. **Reduce scan density** in lidar config

3. **Lower network priority for non-critical topics:**
   - Use BEST_EFFORT QoS instead of RELIABLE

4. **Use wired Ethernet** instead of WiFi

### Q: Battery life is short

**A:** Power optimization:

1. **Measure current draw:**
   - Orange Pi 5: ~2-3A @ 5V = 10-15W
   - Motors: ~1-2A @ 12V per motor = 24-48W
   - RPLidar: ~0.5A @ 5V = 2.5W
   - Total: ~40-65W

2. **Battery sizing example:**
   - 12V 5Ah battery = 60Wh
   - Runtime: 60Wh / 60W = ~1 hour

3. **Improve efficiency:**
   - Use efficient motor driver (TB6612 vs L298N)
   - Add sleep mode when idle
   - Lower CPU frequency when not needed
   - Use LiPo instead of lead-acid (lighter)

## Safety Questions

### Q: What safety features should I add?

**A:** Essential safety features:

1. **Command timeout** (Already implemented in Arduino - 1 second)
2. **Emergency stop button** (See Arduino code example above)
3. **Battery voltage monitoring** (Add voltage sensor)
4. **Obstacle detection** (Use lidar data)
5. **Tilt sensor** (Detect if robot tips over)
6. **Watchdog timer** (Reset if system hangs)

### Q: How to prevent robot from falling off table?

**A:** Multiple approaches:

1. **Cliff sensors** (IR sensors pointing down)
2. **Lidar-based edge detection** (scan for sudden distance changes)
3. **Height map** (use 3D lidar or camera)
4. **Geofencing** (virtual boundaries in nav stack)

For development: Always test in open space, not on elevated surfaces!

## Troubleshooting

### Q: "Could not open serial port" error

**A:**
```bash
# Check device exists
ls -l /dev/ttyUSB* /dev/ttyACM*

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Reboot
sudo reboot
```

### Q: "Failed to create node" error

**A:**
```bash
# Check ROS 2 installation
ros2 doctor --report

# Source workspace
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# Check Python path
python3 -c "import rclpy; print('OK')"
```

### Q: Robot moves in wrong direction

**A:**
1. **Swap motor wires** at L298N (OUT1 ↔ OUT2)
2. **Or invert in Arduino code** (swap HIGH/LOW in direction logic)
3. **Check wheel orientation** in URDF

## Next Steps

### Q: I've got the basic robot working, what's next?

**A:** Progression path:

1. **Add odometry** (encoder feedback) ✓
2. **Implement SLAM** (map building)
   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```
3. **Add Nav2** (autonomous navigation)
   ```bash
   sudo apt install ros-humble-navigation2
   ```
4. **Add camera** (object detection)
5. **Implement behavior trees** (complex missions)

### Q: Where can I get more help?

**A:**

- **Documentation:** HARDWARE_DEPLOYMENT.md (comprehensive guide)
- **ROS 2 Docs:** https://docs.ros.org/en/humble/
- **ROS Answers:** https://answers.ros.org/
- **Orange Pi Forum:** http://www.orangepi.org/orangepibbsen/
- **GitHub Issues:** Open an issue in this repository

---

**Can't find your question?** Open an issue on GitHub or check the main documentation in [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md).
