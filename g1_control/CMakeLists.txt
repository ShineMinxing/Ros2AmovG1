cmake_minimum_required(VERSION 3.5)
project(g1_control)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 编译可执行文件
add_executable(g1_control_node src/g1_control_node.cpp)

# 设置目标依赖
ament_target_dependencies(
  g1_control_node
  rclcpp
  std_msgs
  tf2_geometry_msgs
  nav_msgs
)

# 安装
install(
  TARGETS g1_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
