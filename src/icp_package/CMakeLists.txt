cmake_minimum_required(VERSION 3.8)
project(icp_package)

set(CMAKE_CXX_STANDARD 17)

# Отключаем warning о политике CMP0074
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# TF2 зависимости
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Создаем executable обычным способом
add_executable(ICP_node src/ICP_node.cpp)

# Явно указываем зависимости
ament_target_dependencies(ICP_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

target_link_libraries(ICP_node
)

install(TARGETS ICP_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()