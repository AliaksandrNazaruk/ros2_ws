cmake_minimum_required(VERSION 3.8)
project(aehub_symovo_map_mirror)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/${PROJECT_NAME}/symovo_map_contract_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME symovo_map_contract_probe
)
install(PROGRAMS
  src/${PROJECT_NAME}/symovo_map_mirror_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME symovo_map_mirror
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_symovo_map_geometry test/test_symovo_map_geometry.py)
  ament_add_pytest_test(test_symovo_api_client_parsing test/test_symovo_api_client_parsing.py)
  ament_add_pytest_test(test_symovo_map_mirror_integration test/test_symovo_map_mirror_integration.py)
endif()

ament_package()

