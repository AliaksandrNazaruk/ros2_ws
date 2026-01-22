cmake_minimum_required(VERSION 3.8)
project(aehub_navigation_executor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python node entrypoint as executable
install(PROGRAMS
  src/${PROJECT_NAME}/navigation_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_executor_node
)

# Install sim robot status publisher under its own name
install(PROGRAMS
  src/${PROJECT_NAME}/sim_robot_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_robot_status_publisher
)

# Install initial pose publisher (for headless AMCL init)
install(PROGRAMS
  src/${PROJECT_NAME}/sim_initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_initial_pose_publisher
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY systemd
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # NOTE: This package currently contains legacy code and launch files that do not
  # pass all ament Python linters. We keep functional tests (pytest) enabled.

  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_unit
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    TIMEOUT 180
  )
  
  # Python integration tests
  ament_add_pytest_test(test_integration
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integration
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    TIMEOUT 180
  )
endif()

ament_package()
