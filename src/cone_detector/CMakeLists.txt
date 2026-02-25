cmake_minimum_required(VERSION 3.10)
project(cone_detector)

find_package(ament_cmake REQUIRED)
# Pure-Python node: no build-time need to find these, but harmless if you want:
# find_package(vision_msgs REQUIRED)

install(PROGRAMS
  scripts/yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()

