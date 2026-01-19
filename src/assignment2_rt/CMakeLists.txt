cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generazione Interfacce (Msg e Srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetAvgVel.srv"
  DEPENDENCIES std_msgs
)

# 2. Nodo Controller
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

# FIX DEPRECATION: Sintassi corretta per linkare i messaggi custom
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(controller_node "${cpp_typesupport_target}")

# 3. Nodo Safety
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

# FIX DEPRECATION: Sintassi corretta per linkare i messaggi custom
target_link_libraries(safety_node "${cpp_typesupport_target}")

# Installazione Eseguibili
install(TARGETS
  controller_node
  safety_node
  DESTINATION lib/${PROJECT_NAME})

# MODIFICA: Installazione di TUTTE le cartelle (Launch + Assets Grafici/Config)
install(DIRECTORY
  config
  launch
  meshes
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()