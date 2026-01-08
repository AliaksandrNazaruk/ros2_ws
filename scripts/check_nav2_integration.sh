#!/bin/bash

# Script to verify Nav2 integration with base_controller
# Checks that all required topics and transforms are available

echo "=== Nav2 Integration Check ==="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
if [ -f /home/boris/ros2_ws/install/setup.bash ]; then
    source /home/boris/ros2_ws/install/setup.bash
fi

echo "1. Checking ROS2 topics..."
echo ""

# Check if base_controller is publishing /odom
echo "Checking /odom topic (from base_controller):"
if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    echo "  ✓ /odom topic exists"
    ros2 topic info /odom 2>/dev/null | head -5
else
    echo "  ✗ /odom topic NOT found - base_controller may not be running"
fi
echo ""

# Check if /cmd_vel exists (published by Nav2, consumed by base_controller)
echo "Checking /cmd_vel topic (Nav2 -> base_controller):"
if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    echo "  ✓ /cmd_vel topic exists"
    ros2 topic info /cmd_vel 2>/dev/null | head -5
else
    echo "  ✗ /cmd_vel topic NOT found - Nav2 controller may not be running"
fi
echo ""

# Check if /scan exists (for obstacle avoidance)
echo "Checking /scan topic (for obstacle avoidance):"
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "  ✓ /scan topic exists"
    ros2 topic info /scan 2>/dev/null | head -5
else
    echo "  ⚠ /scan topic NOT found - obstacle avoidance may not work"
    echo "    (This is optional if you're not using laser scan)"
fi
echo ""

# Check TF transforms
echo "2. Checking TF transforms..."
echo "Checking odom -> base_link transform:"
if ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10 | grep -q "At time"; then
    echo "  ✓ odom -> base_link transform exists"
else
    echo "  ✗ odom -> base_link transform NOT found - base_controller may not be publishing TF"
fi
echo ""

# Check Nav2 nodes
echo "3. Checking Nav2 nodes..."
NAV2_NODES=("controller_server" "planner_server" "bt_navigator" "lifecycle_manager_navigation")
for node in "${NAV2_NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "  ✓ $node is running"
    else
        echo "  ✗ $node is NOT running"
    fi
done
echo ""

# Check base_controller node
echo "4. Checking base_controller node..."
if ros2 node list 2>/dev/null | grep -q "base_controller"; then
    echo "  ✓ base_controller node is running"
    ros2 node info /base_controller 2>/dev/null | grep -E "(Subscribers|Publishers)" | head -10
else
    echo "  ✗ base_controller node is NOT running"
fi
echo ""

echo "=== Summary ==="
echo "Required for Nav2 integration:"
echo "  - base_controller publishes /odom: $(ros2 topic list 2>/dev/null | grep -q '^/odom$' && echo 'YES' || echo 'NO')"
echo "  - base_controller publishes TF (odom->base_link): $(ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5 | grep -q 'At time' && echo 'YES' || echo 'NO')"
echo "  - Nav2 publishes /cmd_vel: $(ros2 topic list 2>/dev/null | grep -q '^/cmd_vel$' && echo 'YES' || echo 'NO')"
echo "  - /scan available (optional): $(ros2 topic list 2>/dev/null | grep -q '^/scan$' && echo 'YES' || echo 'NO')"
echo ""

