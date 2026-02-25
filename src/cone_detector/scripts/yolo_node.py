#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.declare_parameter('conf_threshold', 0.25)
        
        pkg_share = get_package_share_directory('cone_detector')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        self.model = YOLO(model_path).to('cpu')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(Image, '/image', self.image_callback, qos)
        self.pub_simple = self.create_publisher(Float32MultiArray, '/yolo/simple_detections', 1)
        self.get_logger().info('YOLO Node Online. Format: [Color Count]')

    def image_callback(self, msg: Image):
        try:
            conf_thresh = self.get_parameter('conf_threshold').get_parameter_value().double_value
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, verbose=False)[0]

            simple_data = []
            blue_count = 0
            yellow_count = 0

            for box in results.boxes:
                if float(box.conf) < conf_thresh:
                    continue

                cls_id = int(box.cls)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = float(0.5 * (x1 + x2)), float(0.5 * (y1 + y2))
                
                simple_data.extend([cx, cy, float(cls_id)])
                
                if cls_id == 0: blue_count += 1
                elif cls_id == 1: yellow_count += 1

            # Publish to MATLAB
            matlab_msg = Float32MultiArray()
            matlab_msg.data = simple_data
            self.pub_simple.publish(matlab_msg)
            
            # Simple, direct terminal output
            self.get_logger().info(f"BLUE: {blue_count} | YELLOW: {yellow_count}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
