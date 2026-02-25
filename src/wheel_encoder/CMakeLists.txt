cmake_minimum_required(VERSION 3.5)
project(wheel_encoder)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python script properly
install(
  PROGRAMS
  scripts/wheel_encoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

