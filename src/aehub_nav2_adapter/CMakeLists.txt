cmake_minimum_required(VERSION 3.8)
project(aehub_nav2_adapter)

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
find_package(nav2_msgs REQUIRED)
find_package(aehub_nav2_readiness REQUIRED)

# Include directories
include_directories(include)

# Nav2AdapterNode library
add_library(nav2_adapter_node SHARED
  src/nav2_adapter_node.cpp
)

ament_target_dependencies(nav2_adapter_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav2_msgs
  aehub_nav2_readiness
)

# Install library
install(TARGETS nav2_adapter_node
  DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(nav2_adapter_node)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav2_msgs
  aehub_nav2_readiness
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
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
    rclcpp_action
    nav2_msgs
    geometry_msgs
    aehub_nav2_readiness
  )

  target_include_directories(test_helpers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # Test executables
  # Smoke test
  add_executable(smoke_test_nav2_adapter
    test/smoke_test_nav2_adapter.cpp
  )

  target_link_libraries(smoke_test_nav2_adapter
    nav2_adapter_node
  )

  ament_target_dependencies(smoke_test_nav2_adapter
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
  )

  # Install test executable
  install(TARGETS smoke_test_nav2_adapter
    DESTINATION lib/${PROJECT_NAME}
  )

  # Add smoke test (manual - requires Nav2 stack)
  # NOTE:
  # Do NOT register this as a ctest test: it requires a running Nav2 stack and will fail in CI.
  # The executable is still installed so it can be run manually:
  #   ros2 run aehub_nav2_adapter smoke_test_nav2_adapter

  # Test matrix test suite
  # Integration-style matrix (DDS-dependent). Build it, but disable in ctest by default.
  ament_add_gtest(test_nav2_adapter_matrix
    test/test_nav2_adapter_matrix.cpp
  )
  target_link_libraries(test_nav2_adapter_matrix
    nav2_adapter_node
    test_helpers
  )
  ament_target_dependencies(test_nav2_adapter_matrix
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    aehub_nav2_readiness
  )
  target_include_directories(test_nav2_adapter_matrix PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_nav2_adapter_matrix PROPERTIES
    DISABLED TRUE
    LABELS "manual;flaky;dds"
    TIMEOUT 300
  )

  # Deterministic unit tests (no DDS graph dependency)
  ament_add_gtest(test_nav2_adapter_unit
    test/test_nav2_adapter_unit.cpp
  )
  target_link_libraries(test_nav2_adapter_unit
    nav2_adapter_node
  )
  ament_target_dependencies(test_nav2_adapter_unit
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
  )
endif()

ament_package()