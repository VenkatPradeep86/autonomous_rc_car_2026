cmake_minimum_required(VERSION 3.5)
project(drive_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python script
install(
  PROGRAMS
  scripts/drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()

