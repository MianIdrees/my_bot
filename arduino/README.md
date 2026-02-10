# Arduino Motor Controller Setup Guide

This guide covers uploading and testing the Arduino Nano motor controller firmware for the my_bot differential drive robot.

## Hardware Requirements

- **Arduino Nano** (ATmega328P)
- **L298N Motor Driver Module**
- **USB Mini-B Cable** (for programming)
- **12V DC Motors** (2x, with encoders optional)
- **12V Power Supply** (2-3A minimum)
- **Jumper Wires**

## Wiring Diagram

### Arduino Nano ↔ L298N Motor Driver

```
Arduino Nano          L298N Motor Driver
═════════════         ══════════════════
Pin 3 (PWM)    →      ENA  (Motor A Speed)
Pin 4          →      IN1  (Motor A Dir 1)
Pin 5          →      IN2  (Motor A Dir 2)
Pin 6 (PWM)    →      ENB  (Motor B Speed)
Pin 7          →      IN3  (Motor B Dir 1)
Pin 8          →      IN4  (Motor B Dir 2)
GND            →      GND
5V             →      5V (Logic power)
```

### L298N ↔ Motors

```
L298N                 Motors
═════                 ══════
OUT1, OUT2     →      Left Motor (Motor A)
OUT3, OUT4     →      Right Motor (Motor B)
12V, GND       ←      12V Power Supply
```

### Optional: Encoder Connections

```
Encoder               Arduino Nano
═══════               ════════════
Left Encoder A  →     Pin 2 (INT0)
Left Encoder B  →     Pin 9
Right Encoder A →     Pin 12 (digital pin - polling or pin change interrupt)
Right Encoder B →     Pin 10
Encoder VCC     →     5V
Encoder GND     →     GND
```

**Note about encoder pins:** Arduino Nano only has 2 hardware interrupt pins (D2, D3). Since D3 is used for motor PWM control, the right encoder uses D12 with polling or pin change interrupts. For best performance with encoders:
- Option 1: Use only left encoder (current config)
- Option 2: Move motor PWM to different pins and use D2/D3 for both encoders
- Option 3: Use pin change interrupt library for any pin

## Software Installation

### Method 1: Using Arduino IDE (Recommended for beginners)

1. **Install Arduino IDE**
   ```bash
   # On Ubuntu
   sudo apt update
   sudo apt install -y arduino
   
   # Or download from: https://www.arduino.cc/en/software
   ```

2. **Connect Arduino Nano via USB**
   ```bash
   # Check if detected
   ls -l /dev/ttyUSB* /dev/ttyACM*
   # Should show: /dev/ttyUSB0 or /dev/ttyACM0
   ```

3. **Configure Arduino IDE**
   - Launch Arduino IDE: `arduino`
   - Go to **Tools → Board → Arduino AVR Boards → Arduino Nano**
   - Go to **Tools → Processor → ATmega328P (Old Bootloader)**
     - *Note: Try "ATmega328P" if upload fails*
   - Go to **Tools → Port → /dev/ttyUSB0** (or /dev/ttyACM0)

4. **Open the Sketch**
   - Go to **File → Open**
   - Navigate to: `~/robot_ws/src/my_bot/arduino/differential_drive_controller.ino`
   - The sketch will open in the IDE

5. **Verify Pin Configuration**
   
   Edit these lines if your wiring is different:
   ```cpp
   #define ENA 3   // Left motor speed (PWM)
   #define IN1 4   // Left motor direction 1
   #define IN2 5   // Left motor direction 2
   #define ENB 6   // Right motor speed (PWM)
   #define IN3 7   // Right motor direction 1
   #define IN4 8   // Right motor direction 2
   ```

6. **Verify Robot Parameters**
   
   Edit these to match your robot's URDF:
   ```cpp
   const float WHEEL_SEPARATION = 0.35;  // meters
   const float WHEEL_RADIUS = 0.05;      // meters
   const float MAX_LINEAR_VEL = 0.5;     // m/s
   const float MAX_ANGULAR_VEL = 1.0;    // rad/s
   ```

7. **Upload to Arduino**
   - Click **Verify** (✓) button to compile
   - Click **Upload** (→) button to flash
   - Wait for "Done uploading" message

8. **Verify Upload Success**
   - Open **Tools → Serial Monitor**
   - Set baud rate to **115200**
   - You should see: `"Arduino Differential Drive Controller Ready"`

### Method 2: Using Arduino CLI (Advanced)

```bash
# Install Arduino CLI
sudo apt install -y arduino-cli

# Install Arduino AVR core
arduino-cli core install arduino:avr

# Navigate to sketch directory
cd ~/robot_ws/src/my_bot/arduino

# Compile the sketch
arduino-cli compile --fqbn arduino:avr:nano differential_drive_controller

# Upload to Arduino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano differential_drive_controller

# Monitor serial output (Ctrl+C to exit)
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

## Testing the Controller

### Test 1: Serial Communication Test

Use a serial terminal to manually send commands:

```bash
# Install screen
sudo apt install -y screen

# Connect to Arduino
screen /dev/ttyUSB0 115200

# You should see startup message:
# "Arduino Differential Drive Controller Ready"
# "Waiting for Twist commands: L:linear,A:angular"

# Test commands (type these):
L:0.1,A:0.0      # Move forward slowly
L:0.0,A:0.0      # Stop
L:-0.1,A:0.0     # Move backward slowly
L:0.0,A:0.5      # Rotate counter-clockwise
L:0.0,A:-0.5     # Rotate clockwise

# Exit screen: Press Ctrl+A, then K, then Y
```

### Test 2: ROS 2 Integration Test (on Orange Pi 5)

```bash
# Terminal 1: Start the serial motor controller node
ros2 run my_bot serial_motor_controller.py --ros-args -p serial_port:=/dev/ttyACM0

# Terminal 2: Publish test commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Test 3: Direction Verification

1. **Forward Test**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
   ```
   - Both wheels should turn in the same direction
   - Robot should move forward

2. **Rotation Test**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}"
   ```
   - Wheels should turn in opposite directions
   - Robot should rotate counter-clockwise (when viewed from above)

3. **If directions are wrong:**
   - Option A: Swap motor wires (swap OUT1 with OUT2, or OUT3 with OUT4)
   - Option B: Invert direction in code:
     ```cpp
     // In driveMotors() function, change:
     if (leftVel > 0.01) {
         digitalWrite(IN1, LOW);   // Swap HIGH/LOW
         digitalWrite(IN2, HIGH);
     }
     ```

## Troubleshooting

### Problem: Arduino not detected

**Solution:**
```bash
# Check USB connection
lsusb | grep -i arduino
# Should show: "1a86:7523 QinHeng Electronics CH340"

# Add user to dialout group
sudo usermod -a -G dialout $USER
sudo reboot

# Create udev rule for permissions
sudo nano /etc/udev/rules.d/99-arduino.rules
```

Add this line:
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"
```

Reload udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Problem: Upload fails with "programmer not responding"

**Solutions:**
1. Try different processor selection:
   - Tools → Processor → **ATmega328P** (without "Old Bootloader")
   
2. Try different board:
   - Tools → Board → **Arduino Mini** instead of Nano

3. Check USB cable (must be data cable, not charge-only)

4. Hold reset button during upload

### Problem: Motors not moving

**Check:**
1. **Power Supply**
   - Verify 12V power connected to L298N
   - Check motor driver power LED is ON

2. **Wiring**
   - Verify all connections match diagram
   - Check for loose wires
   - Test motor directly with power supply

3. **Enable Jumpers**
   - Ensure jumpers are present on ENA and ENB pins (if your L298N has them)

4. **Serial Communication**
   - Open serial monitor, verify startup message
   - Send test command: `L:0.3,A:0.0`
   - Check for timeout messages

### Problem: Motors move in wrong direction

**Solution 1: Swap motor wires**
- Swap OUT1 ↔ OUT2 for left motor
- Swap OUT3 ↔ OUT4 for right motor

**Solution 2: Modify code**
- In `driveMotors()` function, swap HIGH/LOW in direction logic

### Problem: Motors jitter or move erratically

**Causes:**
1. **Insufficient power supply** - Use regulated 12V with at least 2A capacity
2. **Bad ground connection** - Ensure all GNDs are connected
3. **EMI interference** - Keep motor wires away from signal wires
4. **PWM frequency** - Arduino default is usually fine, but can be changed if needed

### Problem: Timeout messages appearing

This is normal when no ROS commands are being sent. The Arduino stops motors after 1 second of no commands for safety.

To disable timeout warnings:
```cpp
// In loop(), comment out the Serial.println:
if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (linearVel != 0.0 || angularVel != 0.0) {
      stopMotors();
      // Serial.println("TIMEOUT: Motors stopped");  // Comment this
    }
}
```

## Encoder Support (Optional)

If your motors have encoders, the code already includes basic encoder reading:

1. **Wire encoders** as shown in wiring diagram above

2. **Adjust encoder parameters** in Arduino code:
   ```cpp
   const int ENCODER_PPR = 360;  // Change to your encoder's pulses per revolution
   ```

3. **Enable encoder feedback** in Arduino code:
   ```cpp
   // In loop(), uncomment this line:
   sendEncoderFeedback();
   ```

4. **Read encoder data** from serial:
   - Format: `E:left_count,right_count`
   - Update rate: 20Hz (every 50ms)

5. **Future enhancement**: Create a ROS 2 odometry node that reads encoder feedback and publishes odometry

## Performance Tips

1. **Reduce serial debug output** for better performance:
   - Comment out all `Serial.print()` statements except startup message

2. **Adjust control loop frequency**:
   ```cpp
   delay(10);  // 100Hz - change to 20 for 50Hz if needed
   ```

3. **Use hardware serial** for higher reliability (already used by default)

4. **Add velocity ramping** to prevent sudden motor movements:
   ```cpp
   // Smooth acceleration (add to driveMotors function)
   const float ACCEL_LIMIT = 0.1;  // m/s per loop iteration
   linearVel = constrain(linearVel, prevLinear - ACCEL_LIMIT, prevLinear + ACCEL_LIMIT);
   ```

## Next Steps

After successful Arduino upload and testing:

1. Connect Arduino to Orange Pi 5 via USB
2. Launch complete robot: `ros2 launch my_bot launch_robot.launch.py`
3. Control from laptop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
4. Add encoder odometry for better navigation
5. Implement closed-loop velocity control using encoder feedback

## Additional Resources

- [L298N Motor Driver Datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [Arduino Nano Pinout](https://docs.arduino.cc/hardware/nano)
- [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)

## Support

If you encounter issues not covered here, check:
1. Serial monitor for error messages
2. Multimeter readings on power supply and motor driver
3. Motor driver heat (add heatsink if L298N gets hot)
4. Orange Pi serial connection: `dmesg | grep ttyACM`
