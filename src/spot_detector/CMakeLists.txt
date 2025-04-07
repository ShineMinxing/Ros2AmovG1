cmake_minimum_required(VERSION 3.8)
project(spot_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(spot_detector_node src/spot_detector_node.cpp)
ament_target_dependencies(spot_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  OpenCV
)

install(TARGETS spot_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
