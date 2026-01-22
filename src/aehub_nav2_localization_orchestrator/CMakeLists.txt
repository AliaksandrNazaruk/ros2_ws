cmake_minimum_required(VERSION 3.8)
project(aehub_nav2_localization_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/${PROJECT_NAME}/nav2_localization_orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_localization_orchestrator
)

if(BUILD_TESTING)
  ament_add_pytest_test(
    test_nav2_localization_orchestrator_integration
    test/test_nav2_localization_orchestrator_integration.py
    TIMEOUT 120
  )
endif()

ament_package()

