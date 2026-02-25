# car_odometry

This package provides a odometry node for a car. It estimates the robot’s pose in the `odom` frame using wheel speeds and a steering angle, then publishes both an `Odometry` message and a TF transform.

---

## Overview

**Node name:** `car_odometry`  
**Executable:** `car_odometry.py`

Main functions:
- Subscribes to wheel speeds and steering angle (`cmd_vel.angular.z`).
- Computes linear velocity, yaw rate, and integrates a simple kinematic model.
- Publishes:
  - `nav_msgs/Odometry` on `/odom`
  - TF transform: `odom` → `base_footprint`

---

## Subscribed Topics
- **`cmd_vel`** (`geometry_msgs/Twist`) — steering angle from `angular.z`  
- **`joint_states`** (`sensor_msgs/msg/JointState`) — wheel speeds  
 

Linear velocity is computed as the wheel-speed average. Yaw rate follows a basic Ackermann model:

ω = (v / L) · tan(δ)

---

## Published Outputs
### `/odom` (nav_msgs/Odometry)
- frame_id: "odom"
- child_frame_id: "base_footprint"
- Pose: x, y, θ
- Velocity: linear.x = v, angular.z = ω

### TF
- Transform from `odom` to `base_footprint` with the same pose and timestamp as `/odom`.

---

## Parameters

- Wheelbase: `L = 0.3` m  
- Timer period: `dt = 0.1` s  
- Initial pose: `x = 0`, `y = 0`, `theta = 0`

---

## Dependencies

- ROS 2 packages: `rclpy`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`
- Python: `tf_transformations`, `math`, `time`
