#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time

# UPDATE THIS after plugging in:
# Arduino Micros often appear as /dev/ttyACM0, ACM1, or ACM2
SERIAL_PORT = "/dev/arduino_sensors" 
BAUD_RATE = 115200

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.sensor_names = ["us_left", "us_rear_center", "us_rear_left", "us_rear_right", "us_right"]
        self.publishers_map = {}

        for sensor in self.sensor_names:
            topic_name = f"/{sensor}"
            self.publishers_map[sensor] = self.create_publisher(Range, topic_name, 10)

        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Micro on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Connection Failed: {e}")
            exit(1)

        self.timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_and_publish(line)
        except Exception as e:
            self.get_logger().warn(f"Read Error: {e}")

    def parse_and_publish(self, line):
        try:
            # Parse "key:val, key:val"
            parts = line.split(",")
            for part in parts:
                if ":" in part:
                    key, val = part.split(":")
                    key = key.strip()
                    dist = float(val.strip())
                    
                    if key in self.publishers_map:
                        self.publish_range(self.publishers_map[key], dist, key)
        except ValueError:
            pass

    def publish_range(self, pub, dist, frame_id):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.52
        msg.min_range = 0.02
        msg.max_range = 2.5
        msg.range = dist
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
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
