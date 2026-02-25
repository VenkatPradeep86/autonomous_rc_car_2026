## 📦 car_description

`car_description` defines the complete car model using **URDF** for use in visualization, simulation, and motion planning tasks in ROS2. It ensures consistent frame alignment across all subsystems by describing the car’s physical structure and sensor placement.

---

### 🔧 Key Components

- **URDF File** – Describes chassis, wheels, joints, and sensor mounts  
- **Meshes** – 3D models for visual representation  
- **Launch File** – Publishes the model via `robot_state_publisher` and Publishes the joint states via `joint_state_publisher`

---

### 🔁 ROS2 Integration

- Provides static transforms (`tf2`) for all links
- Provides dynamic transforms (`tf`) for wheels
- Enables accurate visualization and frame tracking
- Supports alignment of LiDAR, camera, ultrasonic, and encoder data

---

### 🚀 Launch the Model

```bash
ros2 launch car_description display.launch.py
```
