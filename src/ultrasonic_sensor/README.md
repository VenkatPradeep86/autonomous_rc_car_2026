## 📦 ultrasonic_sensor

`ultrasonic_sensor` integrates HC-SR04 ultrasonic range sensors into the car's perception system, enabling short-range obstacle detection during low-speed or close-quarters navigation.

---

### 🔧 Features

- **Publishes real-time range data**
- **Message type:** `std_msgs/msg/Float32`
- **No subscriptions**

---

### 📡 Published Topics

- `/us_right`  
- `/us_left`  
- `/us_rear_center`  
- `/us_rear_left`  
- `/us_rear_right`

---

### 🚀 Run the Node

```bash
ros2 run ultrasonic_sensor ultrasonic_node.py
```

---
