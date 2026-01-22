cmake_minimum_required(VERSION 3.8)
project(aehub_nav2_readiness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Nav2ReadinessGate library
add_library(nav2_readiness_gate SHARED
  src/nav2_readiness_gate.cpp
  src/world_snapshot.cpp
  src/lifecycle_sensor.cpp
  src/action_sensor.cpp
  src/tf_sensor.cpp
)

target_include_directories(nav2_readiness_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_readiness_gate
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install library
install(TARGETS nav2_readiness_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(nav2_readiness_gate)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test helpers library
  add_library(test_helpers SHARED
    test/test_helpers.cpp
  )

  ament_target_dependencies(test_helpers
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    tf2_ros
  )

  target_include_directories(test_helpers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # Test executable
  ament_add_gtest(test_readiness_gate
    test/test_readiness_gate.cpp
  )

  target_include_directories(test_readiness_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  target_link_libraries(test_readiness_gate
    nav2_readiness_gate
    test_helpers
  )

  ament_target_dependencies(test_readiness_gate
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2_ros
  )

  # Set timeout for tests
  set_tests_properties(test_readiness_gate PROPERTIES
    TIMEOUT 60
  )
endif()

ament_package()
