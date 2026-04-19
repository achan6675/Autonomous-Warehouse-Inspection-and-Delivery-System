################################################################################
# Minimum version, project name, compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(project_2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

################################################################################
# Message and Service generation
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ItemEntry.msg"
  "srv/GetTextFile.srv"
  "srv/MoveTo.srv"
  "srv/GetBattery.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${GZ_SIM_INCLUDE_DIRS}
)

################################################################################
# Executables
################################################################################
add_executable(inspection_robot_node 
src/inspection_robot_node.cpp
src/navigator.cpp)

add_executable(battery_life_node 
src/battery_life_node.cpp)

add_executable(image_processing_node 
src/image_processing_node.cpp)

add_executable(user_client 
src/user_client_node.cpp   
src/text_file_generator.cpp)

add_executable(robot_location_manager_node
src/robot_location_manager_node.cpp)

################################################################################
# Link dependencies
################################################################################
ament_target_dependencies(inspection_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  rosidl_default_runtime
)

ament_target_dependencies(battery_life_node
  rclcpp
  sensor_msgs
  rosidl_default_runtime
)

ament_target_dependencies(image_processing_node
  rclcpp
  sensor_msgs
  rosidl_default_runtime
)

ament_target_dependencies(user_client
  rclcpp
  sensor_msgs
  rosidl_default_runtime
)

ament_target_dependencies(robot_location_manager_node
  rclcpp
  sensor_msgs
  rosidl_default_runtime
)

################################################################################
# rosidl interfaces linkage
################################################################################
rosidl_target_interfaces(inspection_robot_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(battery_life_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(image_processing_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(user_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(robot_location_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

################################################################################
# Installation
################################################################################
install(TARGETS  
  inspection_robot_node
  battery_life_node
  image_processing_node
  user_client
  robot_location_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

################################################################################
# Export and package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  rosidl_default_runtime
)

ament_package()

