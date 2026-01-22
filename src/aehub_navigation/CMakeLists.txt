cmake_minimum_required(VERSION 3.8)
project(aehub_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Position Registry (C++)
add_library(position_registry SHARED
  src/position_registry.cpp
)

ament_target_dependencies(position_registry
  rclcpp
  geometry_msgs
  yaml-cpp
  tf2
  tf2_geometry_msgs
)

ament_export_include_directories(include)
ament_export_libraries(position_registry)
ament_export_dependencies(
  rclcpp
  yaml-cpp
)

# Nav2 Action Client (C++) - library, not executable
add_library(nav2_action_client SHARED
  src/nav2_action_client.cpp
)

ament_target_dependencies(nav2_action_client
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# Install
install(TARGETS
  position_registry
  nav2_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(nav2_action_client)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

# Python scripts
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# Install Python node entrypoints as executables (ament_cmake package)
# NOTE: We install with extension-less names so `ros2 run` and launch files can use
# `executable:=navigation_integrated_node` (without `.py`).
install(PROGRAMS
  src/${PROJECT_NAME}/navigation_integrated_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_integrated_node
)
install(PROGRAMS
  src/${PROJECT_NAME}/mqtt_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_status_publisher
)
install(PROGRAMS
  src/${PROJECT_NAME}/initial_pose_from_symovo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME initial_pose_from_symovo
)
install(PROGRAMS
  src/${PROJECT_NAME}/symovo_map_fetcher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME symovo_map_fetcher
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()

