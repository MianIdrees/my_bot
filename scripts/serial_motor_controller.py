#!/usr/bin/env python3

"""
Serial Motor Controller Node for ROS 2
Bridges cmd_vel Twist messages to Arduino via serial port
Compatible with ROS 2 Humble and Jazzy

Author: Generated for my_bot deployment
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialMotorController(Node):
    """
    ROS 2 node that subscribes to cmd_vel and sends motor commands
    to Arduino via serial connection.
    """
    
    def __init__(self):
        super().__init__('serial_motor_controller')
        
        # Declare parameters
        # Note: Device name depends on USB-to-serial chip:
        # - /dev/ttyACM0 for ATmega32u4-based boards or native USB
        # - /dev/ttyUSB0 for CH340/CP2102 USB-to-serial chips (common on cheap Nano clones)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Get parameters
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # Initialize serial connection
        self.serial = None
        self.connect_serial()
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create timer for connection monitoring
        self.timer = self.create_timer(
            self.reconnect_interval,
            self.check_connection
        )
        
        self.get_logger().info('Serial Motor Controller initialized')
        self.get_logger().info(f'Listening on cmd_vel topic')
    
    def connect_serial(self):
        """Establish serial connection to Arduino"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            
            # Wait for Arduino to reset after serial connection
            time.sleep(2)
            
            # Read startup message from Arduino
            if self.serial.in_waiting > 0:
                startup_msg = self.serial.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f'Arduino: {startup_msg}')
            
            self.get_logger().info(
                f'Connected to Arduino on {self.port} at {self.baud} baud'
            )
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial = None
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.serial = None
            return False
    
    def check_connection(self):
        """Periodically check serial connection and attempt reconnect if needed"""
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn('Serial connection lost. Attempting to reconnect...')
            self.connect_serial()
    
    def cmd_vel_callback(self, msg):
        """
        Callback for cmd_vel topic.
        Converts Twist message to serial command and sends to Arduino.
        
        Args:
            msg (Twist): Velocity command with linear.x and angular.z
        """
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn('Serial port not connected. Skipping command.')
            return
        
        try:
            # Extract velocities
            linear = msg.linear.x
            angular = msg.angular.z
            
            # Format command string for Arduino: "L:linear,A:angular\n"
            command = f"L:{linear:.3f},A:{angular:.3f}\n"
            
            # Send to Arduino
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()
            
            # Debug logging (can be disabled for production)
            self.get_logger().debug(
                f'Sent: linear={linear:.3f} m/s, angular={angular:.3f} rad/s'
            )
            
            # Read any feedback from Arduino (optional)
            if self.serial.in_waiting > 0:
                feedback = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if feedback and not feedback.startswith('CMD:'):
                    self.get_logger().info(f'Arduino: {feedback}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial = None  # Trigger reconnection
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {e}')
    
    def destroy_node(self):
        """Clean up serial connection when node is destroyed"""
        if self.serial and self.serial.is_open:
            try:
                # Send stop command before closing
                stop_command = "L:0.000,A:0.000\n"
                self.serial.write(stop_command.encode('utf-8'))
                self.serial.flush()
                time.sleep(0.1)
                self.serial.close()
                self.get_logger().info('Serial connection closed')
            except Exception as e:
                self.get_logger().error(f'Error closing serial: {e}')
        
        super().destroy_node()


def main(args=None):
    """Main function to initialize and spin the node"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = SerialMotorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt, shutting down')
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
        else:
            print(f'Failed to create node: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
