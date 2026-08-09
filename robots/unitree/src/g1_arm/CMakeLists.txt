cmake_minimum_required(VERSION 3.8)
project(g1_arm)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # <-- 添加这一行
#find_package(nav_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(tf2 REQUIRED)
find_package(unitree_sdk2)
include_directories(/usr/local/include/ddscxx)

# 创建可执行文件
add_executable(g1_getcmd_node src/getcmd.cpp)
#target_link_libraries(g1_getvel_node unitree_sdk2)

# 链接依赖（添加std_msgs）
ament_target_dependencies(g1_getcmd_node
  rclcpp
  std_msgs  # <-- 添加这一行
  rosidl_typesupport_introspection_cpp
#  tf2_ros
#  tf2
)

add_executable(g1_pubcmd_node src/pubcmd.cpp)
ament_target_dependencies(g1_pubcmd_node
  rclcpp
  std_msgs
  rosidl_typesupport_introspection_cpp
)
target_link_libraries(g1_pubcmd_node unitree_sdk2)

# 安装可执行文件
install(TARGETS g1_getcmd_node 
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS g1_pubcmd_node 
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs  # <-- 添加这一行
  tf2_ros
  tf2
)

ament_package()
