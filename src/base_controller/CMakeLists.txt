cmake_minimum_required(VERSION 3.8)
project(base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aehub_msgs REQUIRED)

# Find OpenSSL for HTTPS support
find_package(OpenSSL REQUIRED)

# Include directories
include_directories(include)

# AsyncDriverClient library
add_library(async_driver_client
  src/async_driver_client.cpp
)
ament_target_dependencies(async_driver_client
  rclcpp
  geometry_msgs
)
if(OpenSSL_FOUND)
  target_link_libraries(async_driver_client OpenSSL::SSL OpenSSL::Crypto)
  target_compile_definitions(async_driver_client PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)
  message(STATUS "Enabled OpenSSL support for async_driver_client")
else()
  message(WARNING "OpenSSL not found - async_driver_client will not support HTTPS")
endif()

# Symovo JSON extraction helper (no ROS dependencies)
add_library(symovo_json_extract
  src/symovo_json_extract.cpp
)

# Symovo safety mapping helper (pure, unit-testable)
add_library(symovo_safety_mapping
  src/symovo_safety_mapping.cpp
)
target_link_libraries(symovo_safety_mapping symovo_json_extract)

# Base controller node executable
add_executable(base_controller_node
  src/base_controller_node.cpp
)
ament_target_dependencies(base_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  aehub_msgs
)
target_link_libraries(base_controller_node async_driver_client symovo_json_extract symovo_safety_mapping)
if(OpenSSL_FOUND)
  target_compile_definitions(base_controller_node PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)
endif()

# Symovo API probe node (manual test utility)
add_executable(symovo_api_probe_node
  src/symovo_api_probe_node.cpp
)
ament_target_dependencies(symovo_api_probe_node
  rclcpp
)
if(OpenSSL_FOUND)
  target_link_libraries(symovo_api_probe_node OpenSSL::SSL OpenSSL::Crypto)
  target_compile_definitions(symovo_api_probe_node PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)
endif()

# Install
install(TARGETS
  async_driver_client
  symovo_json_extract
  symovo_safety_mapping
  base_controller_node
  symovo_api_probe_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_symovo_json_extract
    test/test_symovo_json_extract.cpp
  )
  target_link_libraries(test_symovo_json_extract symovo_json_extract)

  ament_add_gtest(test_symovo_safety_mapping
    test/test_symovo_safety_mapping.cpp
  )
  target_link_libraries(test_symovo_safety_mapping symovo_safety_mapping)
endif()

ament_package()
