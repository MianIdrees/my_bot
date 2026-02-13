#!/usr/bin/env python3
"""
diff_drive_node.py — ROS2 Serial Bridge for Arduino Motor Controller

This node bridges ROS2 and the Arduino Nano motor controller:
  - Subscribes to /cmd_vel (Twist) → sends PWM commands to Arduino
  - Reads encoder ticks from Arduino → publishes /odom (Odometry) and TF odom→base_link
  - Publishes /joint_states for wheel joint visualization

Differential Drive Kinematics:
  Robot parameters must match the URDF:
    wheel_separation = 0.35 m
    wheel_radius     = 0.05 m

  Forward kinematics (encoders → odometry):
    v_left  = (delta_left_ticks  / ticks_per_rev) * 2π * wheel_radius / dt
    v_right = (delta_right_ticks / ticks_per_rev) * 2π * wheel_radius / dt
    v = (v_right + v_left) / 2
    ω = (v_right - v_left) / wheel_separation

  Inverse kinematics (cmd_vel → motor PWM):
    v_left  = v - ω * wheel_separation / 2
    v_right = v + ω * wheel_separation / 2
    PWM proportional to velocity (with configurable scaling)

Serial Protocol:
  TX to Arduino:  m <left_pwm> <right_pwm>\n
  RX from Arduino: e <left_ticks> <right_ticks>\n
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

import serial


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # ========================== PARAMETERS ==========================
        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('wheel_radius', 0.0325)   # 65mm wheels
        self.declare_parameter('ticks_per_rev', 990.0)    # JGB37-520: 11 PPR × 90:1 gear
        self.declare_parameter('max_motor_speed', 0.37)   # ~110 RPM × π × 0.065m
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0)       # Hz

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Derived constants
        self.meters_per_tick = (2.0 * math.pi * self.wheel_rad) / self.ticks_per_rev
        # PWM scaling: PWM_value = velocity / max_motor_speed * 255
        self.pwm_per_mps = 255.0 / self.max_motor_speed

        # ========================== ODOMETRY STATE ==========================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_odom_time = None

        # Wheel positions for joint_states (radians)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # ========================== SERIAL CONNECTION ==========================
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # ========================== ROS2 INTERFACES ==========================
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for reading serial and publishing odometry
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.update_callback)

        # Watchdog: stop motors if no cmd_vel for 0.5s
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds

        self.get_logger().info(
            f'DiffDriveNode started: port={self.serial_port}, '
            f'wheel_sep={self.wheel_sep}, wheel_rad={self.wheel_rad}, '
            f'ticks_per_rev={self.ticks_per_rev}'
        )

    def connect_serial(self):
        """Attempt to open the serial port to the Arduino."""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.05
            )
            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            # Flush any startup messages
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to motor PWM commands and send to Arduino."""
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s

        # Inverse kinematics: compute wheel velocities
        v_left = v - (w * self.wheel_sep / 2.0)
        v_right = v + (w * self.wheel_sep / 2.0)

        # Convert to PWM (-255 to 255)
        left_pwm = int(v_left * self.pwm_per_mps)
        right_pwm = int(v_right * self.pwm_per_mps)

        # Clamp
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        self.send_motor_command(left_pwm, right_pwm)

    def send_motor_command(self, left_pwm, right_pwm):
        """Send motor command to Arduino via serial."""
        if self.ser is None or not self.ser.is_open:
            return
        cmd = f'm {left_pwm} {right_pwm}\n'
        try:
            with self.serial_lock:
                self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def read_encoder_data(self):
        """Read and parse encoder data from Arduino. Returns (left_ticks, right_ticks) or None."""
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            with self.serial_lock:
                # Read all available lines, use the latest encoder message
                latest = None
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('e '):
                        latest = line

                if latest is None:
                    return None

                parts = latest.split()
                if len(parts) == 3:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    return (left_ticks, right_ticks)
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Serial read error: {e}')

        return None

    def update_callback(self):
        """Main update loop: read encoders, compute odometry, publish."""
        now = self.get_clock().now()

        # Watchdog: stop motors if cmd_vel timeout
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_timeout:
            self.send_motor_command(0, 0)

        # Read encoders
        enc_data = self.read_encoder_data()
        if enc_data is None:
            return

        left_ticks, right_ticks = enc_data

        # Initialize on first reading
        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            return

        # Time delta
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # Compute deltas
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_odom_time = now

        # Distance traveled by each wheel
        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick

        # Update wheel positions (radians) for joint_states
        self.left_wheel_pos += delta_left * (2.0 * math.pi / self.ticks_per_rev)
        self.right_wheel_pos += delta_right * (2.0 * math.pi / self.ticks_per_rev)

        # Differential drive forward kinematics
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_sep

        # Update pose
        if abs(delta_theta) < 1e-6:
            # Straight line
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:
            # Arc
            radius = dist_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta
        # Normalize theta to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        self.v_linear = dist_center / dt
        self.v_angular = delta_theta / dt

        # Publish odometry
        self.publish_odometry(now)

        # Publish joint states
        self.publish_joint_states(now)

        # Publish TF
        if self.publish_tf:
            self.publish_odom_tf(now)

    def publish_odometry(self, stamp):
        """Publish /odom message."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Pose covariance (simple diagonal)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        # Twist
        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.v_angular

        # Twist covariance
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

    def publish_joint_states(self, stamp):
        """Publish /joint_states for wheel joints (needed for URDF visualization)."""
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)

    def publish_odom_tf(self, stamp):
        """Broadcast odom → base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self.theta)

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Clean shutdown: stop motors and close serial."""
        self.get_logger().info('Shutting down — stopping motors')
        self.send_motor_command(0, 0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
