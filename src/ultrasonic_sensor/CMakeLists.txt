cmake_minimum_required(VERSION 3.5)
project(ultrasonic_sensor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python script
install(
  PROGRAMS
  scripts/ultrasonic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()

