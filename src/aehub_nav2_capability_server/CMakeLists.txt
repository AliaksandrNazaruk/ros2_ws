cmake_minimum_required(VERSION 3.8)
project(aehub_nav2_capability_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(aehub_msgs REQUIRED)
find_package(aehub_nav2_adapter REQUIRED)
find_package(aehub_nav2_readiness REQUIRED)
find_package(aehub_robot_readiness REQUIRED)
find_package(aehub_composite_readiness REQUIRED)

# Capability server implementation library (so tests can link it)
add_library(nav2_capability_server_lib
  src/nav2_capability_server_node.cpp
)

target_include_directories(nav2_capability_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_capability_server_lib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav2_msgs
  action_msgs
  diagnostic_msgs
  aehub_msgs
  aehub_nav2_adapter
  aehub_nav2_readiness
  aehub_robot_readiness
  aehub_composite_readiness
)

add_executable(aehub_nav2_capability_server
  src/main.cpp
)
target_link_libraries(aehub_nav2_capability_server nav2_capability_server_lib)

ament_target_dependencies(aehub_nav2_capability_server
  rclcpp
)

install(TARGETS aehub_nav2_capability_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_stack_smoke
    test/test_stack_smoke.cpp
  )

  ament_target_dependencies(test_stack_smoke
    rclcpp
    aehub_nav2_readiness
    aehub_robot_readiness
    aehub_composite_readiness
  )

  ament_add_gtest(test_capability_gating
    test/test_capability_gating.cpp
  )
  ament_target_dependencies(test_capability_gating
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    lifecycle_msgs
    aehub_msgs
    aehub_nav2_readiness
    aehub_nav2_adapter
  )
  target_link_libraries(test_capability_gating
    nav2_capability_server_lib
  )
endif()

ament_package()

