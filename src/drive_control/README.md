## 📦 drive_control

`drive_control` is a ROS2 node that interfaces high-level velocity commands with the car's motor and steering hardware. It subscribes to `/cmd_vel` and sends control signals over USB serial to a microcontroller.

---

### 🔧 Features

- **Topic:** `/cmd_vel`
- **Message:** `geometry_msgs/msg/Twist` (or custom)
- **Interface:** USB Serial (`/dev/ttyACM*`)
- **Protocol:** ASCII commands (e.g., `x:6, z:2\n`)
- **Command:**  
  ```bash
  ros2 run drive_control drive_node.py
  ```

---

### 📌 Function

Acts as the **actuator interface layer** in the driving stack—bridging ROS2 tools like `rqt_robot_steering` with hardware for real-time motion control.
