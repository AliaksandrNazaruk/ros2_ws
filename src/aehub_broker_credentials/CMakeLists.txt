cmake_minimum_required(VERSION 3.5)
project(aehub_broker_credentials)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python node entrypoint as executable
install(PROGRAMS
  src/${PROJECT_NAME}/broker_credentials_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME broker_credentials_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_broker_client
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_broker_client.py
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
  )
endif()

ament_package()
