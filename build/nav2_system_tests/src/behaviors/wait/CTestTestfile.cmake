# CMake generated Testfile for 
# Source directory: /home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/wait
# Build directory: /home/boris/ros2_ws/build/nav2_system_tests/src/behaviors/wait
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_wait_behavior "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/boris/ros2_ws/build/nav2_system_tests/test_results/nav2_system_tests/test_wait_behavior.xml" "--package-name" "nav2_system_tests" "--generate-result-on-success" "--env" "TEST_EXECUTABLE=/home/boris/ros2_ws/build/nav2_system_tests/src/behaviors/wait/test_wait_behavior_node" "BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_behavior.xml" "--command" "/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/wait/test_wait_behavior_launch.py")
set_tests_properties(test_wait_behavior PROPERTIES  TIMEOUT "180" WORKING_DIRECTORY "/home/boris/ros2_ws/build/nav2_system_tests/src/behaviors/wait" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/wait/CMakeLists.txt;12;ament_add_test;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behaviors/wait/CMakeLists.txt;0;")
