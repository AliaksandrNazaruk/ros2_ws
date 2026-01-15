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
  src/${PROJECT_NAME}/navigation_command_handler.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_command_handler
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

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  # ---------------------------------------------------------------------------
  # Lint (explicit config and scope)
  # ---------------------------------------------------------------------------
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME ament_flake8_custom
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "${CMAKE_CURRENT_SOURCE_DIR}/setup.py"
  )

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(
    TESTNAME ament_pep257_custom
    --convention google
    --add-ignore
      D100 D101 D102 D103 D104 D105 D107
      D200 D204 D205 D212
      D400 D401 D413 D415 D417
      D411
    --exclude
      test build install log setup.py
  )

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(TESTNAME ament_lint_cmake_custom)

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(TESTNAME ament_cpplint_custom)

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(TESTNAME ament_uncrustify_custom)

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(TESTNAME ament_cppcheck_custom)

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(TESTNAME ament_xmllint_custom)

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(
    TESTNAME ament_copyright_custom
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )

  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)

  # Unit tests
  ament_add_pytest_test(test_position_registry
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_position_registry.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_navigation_state_manager
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_navigation_state_manager.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_command_validation
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_command_validation.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_mqtt_status_publisher
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_mqtt_status_publisher.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_broker_config_provider
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_broker_config_provider.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_mqtt_connection_manager
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_mqtt_connection_manager.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )

  # Integration tests (will use launch_testing in future)
  # For now, add them as pytest tests
  ament_add_pytest_test(test_navigation_command_flow
    "${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_navigation_command_flow.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 120
  )

  ament_add_pytest_test(test_state_machine
    "${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_state_machine.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 120
  )
endif()

ament_package()

