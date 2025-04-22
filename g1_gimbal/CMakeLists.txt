cmake_minimum_required(VERSION 3.8)
project(g1_gimbal)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(src/serial)

include_directories(
  include
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64-linux)

add_executable(g1_gimbal_node src/g1_gimbal_node.cpp)
target_link_libraries(g1_gimbal_node
  serial
  AMOV_Gimbal
)
ament_target_dependencies(g1_gimbal_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS g1_gimbal_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
