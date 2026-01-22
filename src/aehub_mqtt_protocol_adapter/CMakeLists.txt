cmake_minimum_required(VERSION 3.8)
project(aehub_mqtt_protocol_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python node entrypoint as executable
install(PROGRAMS
  src/${PROJECT_NAME}/mqtt_protocol_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_protocol_adapter_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # NOTE: This package is currently a thin legacy wrapper; keep it buildable and runnable
  # while we migrate to the new MQTT transport stack. We intentionally do not run ament
  # Python linters here (flake8/pep257) until the migration is complete.
endif()

ament_package()

