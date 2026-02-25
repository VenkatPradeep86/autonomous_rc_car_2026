#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

SERIAL_PORT = "/dev/arduino_drive"
BAUD_RATE = 115200

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        
        # Rate limiting variables
        self.last_send_time = time.time()
        self.send_interval = 0.05  # Limit to 20Hz (every 50ms)

        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2.0) # Wait for Arduino reset
            self.get_logger().info(f"Connected to {SERIAL_PORT} at {BAUD_RATE} Baud")
        except serial.SerialException as e:
            self.get_logger().error(f"FATAL: Could not open serial port: {e}")
            exit(1)

    def twist_callback(self, msg):
        current_time = time.time()
        
        # Prevent Serial Buffer Overflow by limiting frequency
        if (current_time - self.last_send_time) < self.send_interval:
            return

        # Prepare the command string
        cmd_str = f"x:{msg.linear.x:.2f} z:{msg.angular.z:.2f}\n"

        try:
            self.serial.write(cmd_str.encode())
            # Log the action so you can see it in the terminal
            self.get_logger().info(f"Sending to Arduino -> {cmd_str.strip()}")
            self.last_send_time = current_time
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Write Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down. Sending stop command...")
        # Final safety stop
        if node.serial.is_open:
            node.serial.write("x:0.00 z:0.00\n".encode())
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
