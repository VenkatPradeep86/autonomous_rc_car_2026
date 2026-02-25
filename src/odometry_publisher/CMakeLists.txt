cmake_minimum_required(VERSION 3.5)
project(odometry_publisher)

# Find Python3 and required ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
  scripts/car_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

