#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
import time

# UPDATE THIS PORT if needed (e.g. /dev/ttyACM0)
SERIAL_PORT = "/dev/arduino_wheel_encoder"
BAUD_RATE = 115200

class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.steering_angle = 0.0
        self.prev_left_dist = 0.0
        self.prev_right_dist = 0.0
        self.prev_time = self.get_clock().now()

        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2) 
            self.get_logger().info(f"Connected to Arduino on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        self.timer = self.create_timer(0.05, self.read_encoder_data)

    def cmd_vel_callback(self, msg: Twist):
        self.steering_angle = msg.angular.z

    def read_encoder_data(self):
        try:
            if self.serial.in_waiting:
                # READ TEXT instead of Binary
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                
                # Check if line looks valid (contains "left_distance")
                if "left_distance" in line:
                    values = self.parse_text_packet(line)
                    if values:
                        self.publish_joint_state(values)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def parse_text_packet(self, line):
        try:
            # Format: "left_distance:1.2, right_distance:1.2, ..."
            data = {}
            parts = line.split(',')
            for part in parts:
                if ':' in part:
                    key, val = part.split(':')
                    data[key.strip()] = float(val)
            
            return {
                "left_distance": data.get("left_distance", 0.0),
                "right_distance": data.get("right_distance", 0.0)
            }
        except ValueError:
            return None

    def publish_joint_state(self, values: dict):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 1e-3

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]

        # Arduino already sends METERS, so no conversion needed
        dist_left = values.get("left_distance", 0.0)
        dist_right = values.get("right_distance", 0.0)

        msg.position = [
            self.steering_angle,
            self.steering_angle,
            dist_left,
            dist_right,
        ]

        v_left = (dist_left - self.prev_left_dist) / dt
        v_right = (dist_right - self.prev_right_dist) / dt

        msg.velocity = [0.0, 0.0, v_left, v_right]
        msg.effort = []

        self.prev_left_dist = dist_left
        self.prev_right_dist = dist_right
        self.prev_time = now
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
