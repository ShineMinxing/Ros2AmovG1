cmake_minimum_required(VERSION 3.8)
project(g1_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(g1_camera_node src/g1_camera_node.cpp)
ament_target_dependencies(g1_camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
  image_transport
  compressed_image_transport
)

install(TARGETS g1_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
