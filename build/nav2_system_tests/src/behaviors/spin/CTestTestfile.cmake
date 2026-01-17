# CMake generated Testfile for 
# Source directory: /home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/spin
# Build directory: /home/boris/ros2_ws/build/nav2_system_tests/src/behaviors/spin
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_spin_behavior "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/boris/ros2_ws/build/nav2_system_tests/test_results/nav2_system_tests/test_spin_behavior.xml" "--package-name" "nav2_system_tests" "--generate-result-on-success" "--env" "TEST_DIR=/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/spin" "--command" "/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/spin/test_spin_behavior.launch.py")
set_tests_properties(test_spin_behavior PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/boris/ros2_ws/build/nav2_system_tests/src/behaviors/spin" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/spin/CMakeLists.txt;1;ament_add_test;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/spin/CMakeLists.txt;0;")
