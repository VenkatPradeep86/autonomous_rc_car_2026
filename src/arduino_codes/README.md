## 📦 arduino_codes

The `arduino_codes` package contains firmware for various microcontroller-based subsystems in the car. These programs interface with hardware components such as motor drivers, steering servos, ultrasonic sensors, and wheel encoders.

---

### 🔧 Key Functionalities

- **Motor and Steering Control** – Parses serial commands and drives actuators
- **Ultrasonic Sensor Handling** – Measures distances using HC-SR04 and sends data via serial
- **Wheel Encoder Processing** – Measures and calculates distances for odometry

---

### 💡 Usage Overview

Each Arduino sketch targets a specific subsystem:
- **drive_arduino.ino** – Controls direction and motor speed
- **ultrasonic.ino** – Reads distances and sends them to ROS
- **wheel_encoder.ino** – Processes pulses and calculates distances

---

### 🧪 Integration with ROS2

These sketches work in tandem with the following ROS2 nodes:
| Arduino Code        | ROS2 Node           | Communication |
|---------------------|---------------------|----------------|
| `drive_arduino.ino`   | `drive_control`     | USB Serial     |
| `ultrasonic.ino` | `ultrasonic_sensor` | USB Serial     |
| `wheeel_encoder.ino`| `wheel_encoder`     | USB Serial     |

---
