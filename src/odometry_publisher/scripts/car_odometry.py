#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from sensor_msgs.msg import JointState

from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

import math


class CarOdometry(Node):
    def __init__(self):
        super().__init__('car_odometry')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Car parameters
        self.L = 0.3   # Wheelbase (m)
        self.dt = 0.1  # Nominal update rate
        self.last_time = self.get_clock().now()

        # Vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Inputs
        self.steering_angle = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

        # Timer
        self.timer = self.create_timer(self.dt, self.publish_odometry)

    def cmd_vel_callback(self, msg: Twist):
        # Steering angle in radians
        self.steering_angle = msg.angular.z

    def joint_state_callback(self, msg: JointState):
        """
        Extract rear wheel velocities from joint_states using URDF names:
          - left_rear_wheel_joint
          - right_rear_wheel_joint
        """
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        if "left_rear_wheel_joint" in name_to_index:
            i = name_to_index["left_rear_wheel_joint"]
            if i < len(msg.velocity):
                self.v_left = msg.velocity[i]

        if "right_rear_wheel_joint" in name_to_index:
            i = name_to_index["right_rear_wheel_joint"]
            if i < len(msg.velocity):
                self.v_right = msg.velocity[i]

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        self.last_time = now

        # Linear velocity from rear wheels
        v = (self.v_left + self.v_right) / 2.0

        # yaw rate
        if abs(self.steering_angle) > 1e-3:
            omega = v / self.L * math.tan(self.steering_angle)
        else:
            omega = 0.0

        # Integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Convert yaw → quaternion
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CarOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
