cmake_minimum_required(VERSION 3.8)
project(aehub_robot_readiness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(aehub_nav2_readiness REQUIRED)
find_package(aehub_msgs REQUIRED)

# RobotReadinessGate library
add_library(robot_readiness_gate SHARED
  src/robot_readiness_gate.cpp
  src/odom_watcher.cpp
  src/cmd_vel_watcher.cpp
  src/safety_watcher.cpp
)

target_include_directories(robot_readiness_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_readiness_gate
  rclcpp
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  aehub_nav2_readiness
  aehub_msgs
)

# Robot readiness monitor node (telemetry adapter)
add_executable(robot_readiness_monitor_node
  src/robot_readiness_monitor_node.cpp
)
target_link_libraries(robot_readiness_monitor_node robot_readiness_gate)
ament_target_dependencies(robot_readiness_monitor_node
  rclcpp
  diagnostic_msgs
)

# Install library
install(TARGETS robot_readiness_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS robot_readiness_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch/config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(robot_readiness_gate)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  aehub_nav2_readiness
  aehub_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Test executable
  ament_add_gtest(test_robot_readiness_gate
    test/test_robot_readiness_gate.cpp
  )

  target_include_directories(test_robot_readiness_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_robot_readiness_gate
    robot_readiness_gate
  )

  ament_target_dependencies(test_robot_readiness_gate
    rclcpp
    geometry_msgs
    nav_msgs
    aehub_nav2_readiness
    aehub_msgs
  )

  # Set timeout for tests
  set_tests_properties(test_robot_readiness_gate PROPERTIES
    TIMEOUT 60
  )
endif()

ament_package()
