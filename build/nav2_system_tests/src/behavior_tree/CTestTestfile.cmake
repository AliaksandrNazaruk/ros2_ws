# CMake generated Testfile for 
# Source directory: /home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behavior_tree
# Build directory: /home/boris/ros2_ws/build/nav2_system_tests/src/behavior_tree
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_behavior_tree_node "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/boris/ros2_ws/build/nav2_system_tests/test_results/nav2_system_tests/test_behavior_tree_node.gtest.xml" "--package-name" "nav2_system_tests" "--output-file" "/home/boris/ros2_ws/build/nav2_system_tests/ament_cmake_gtest/test_behavior_tree_node.txt" "--command" "/home/boris/ros2_ws/build/nav2_system_tests/src/behavior_tree/test_behavior_tree_node" "--gtest_output=xml:/home/boris/ros2_ws/build/nav2_system_tests/test_results/nav2_system_tests/test_behavior_tree_node.gtest.xml")
set_tests_properties(test_behavior_tree_node PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/boris/ros2_ws/build/nav2_system_tests/src/behavior_tree/test_behavior_tree_node" TIMEOUT "60" WORKING_DIRECTORY "/home/boris/ros2_ws/build/nav2_system_tests/src/behavior_tree" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behavior_tree/CMakeLists.txt;3;ament_add_gtest;/home/boris/ros2_ws/src/navigation2/nav2_system_tests/src/behavior_tree/CMakeLists.txt;0;")
subdirs("../../gtest")
