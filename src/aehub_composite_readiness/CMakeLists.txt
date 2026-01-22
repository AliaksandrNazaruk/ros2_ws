cmake_minimum_required(VERSION 3.8)
project(aehub_composite_readiness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aehub_nav2_readiness REQUIRED)

# CompositeReadinessGate library
add_library(composite_readiness_gate SHARED
  src/composite_readiness_gate.cpp
)

target_include_directories(composite_readiness_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(composite_readiness_gate
  rclcpp
  aehub_nav2_readiness
)

# Install library
install(TARGETS composite_readiness_gate
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
ament_export_libraries(composite_readiness_gate)
ament_export_dependencies(
  rclcpp
  aehub_nav2_readiness
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Test executable
  ament_add_gtest(test_composite_readiness_gate
    test/test_composite_readiness_gate.cpp
  )

  target_include_directories(test_composite_readiness_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_composite_readiness_gate
    composite_readiness_gate
  )

  ament_target_dependencies(test_composite_readiness_gate
    rclcpp
    aehub_nav2_readiness
  )

  # Set timeout for tests
  set_tests_properties(test_composite_readiness_gate PROPERTIES
    TIMEOUT 60
  )
endif()

ament_package()
