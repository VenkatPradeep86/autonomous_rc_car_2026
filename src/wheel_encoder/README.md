## 📦 wheel_encoder

`wheel_encoder` captures real-time velocity data from optical wheel encoders, aiding in odometry and dead-reckoning localization. It interfaces with an Arduino to receive processed rear-wheel distances and velocites readings and front wheel steering angle.

---

### 🔧 Features

- **Publishes:**  `/joint_states`
- **Message Type:** `sensor_msgs/msg/JointState Message`
- **Continuous output** at configurable frequency

---

### 📡 Published Topic

- `/joint_states`

---

### 🚀 Run the Node

```bash
ros2 run wheel_encoder wheel_encoder_node.py
```

---
