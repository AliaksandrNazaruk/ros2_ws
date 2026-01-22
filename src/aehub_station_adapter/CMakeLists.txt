cmake_minimum_required(VERSION 3.8)
project(aehub_station_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(PROGRAMS
  src/${PROJECT_NAME}/station_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME station_adapter
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()

