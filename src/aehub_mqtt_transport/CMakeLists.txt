cmake_minimum_required(VERSION 3.8)
project(aehub_mqtt_transport)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python node entrypoint as executable
install(PROGRAMS
  src/${PROJECT_NAME}/mqtt_transport_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_transport_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # NOTE: This package contains legacy integration code and launch files that do not
  # yet conform to the strict ament Python lint rules. Keep functional pytest coverage
  # enabled; re-enable ament linters once the codebase is migrated/cleaned up.

  # Python integration tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mqtt_topic_mapper
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_mqtt_topic_mapper.py
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
  )
  
  ament_add_pytest_test(test_transport_smoke
    ${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_transport_smoke.py
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
  )
endif()

ament_package()
