cmake_minimum_required(VERSION 3.8)
project(video_play)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_play_node src/video_play_node.cpp)
ament_target_dependencies(video_play_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
)

install(TARGETS video_play_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
