#!/bin/bash
# Manual checklist verification script for aehub_navigation_executor

set -e

WORKSPACE=/home/boris/ros2_ws
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=========================================="
echo "1. Building package..."
echo "=========================================="
cd "$WORKSPACE"
colcon build --packages-select aehub_navigation_executor
source "$WORKSPACE/install/setup.bash"

echo ""
echo "=========================================="
echo "2. Checking lifecycle behavior..."
echo "=========================================="

# Start executor
echo "Starting executor..."
ros2 launch aehub_navigation_executor navigation_executor.launch.py > /tmp/executor.log 2>&1 &
EXECUTOR_PID=$!
sleep 5

# Check state
echo "Current state:"
ros2 lifecycle get /navigation_executor || echo "Node not ready yet"
sleep 2

# Test transitions
echo -e "\nTesting deactivate:"
ros2 lifecycle set /navigation_executor deactivate && sleep 1
ros2 lifecycle get /navigation_executor

echo -e "\nTesting activate:"
ros2 lifecycle set /navigation_executor activate && sleep 1
ros2 lifecycle get /navigation_executor

echo -e "\nTesting double activate (should fail):"
ros2 lifecycle set /navigation_executor activate && sleep 1 || echo "✓ Correctly rejected duplicate activate"

# Cleanup
pkill -P $EXECUTOR_PID || true
wait $EXECUTOR_PID 2>/dev/null || true
sleep 2

echo ""
echo "=========================================="
echo "3. Checking ROS topics..."
echo "=========================================="

# Start executor again
ros2 launch aehub_navigation_executor navigation_executor.launch.py > /tmp/executor_topics.log 2>&1 &
EXECUTOR_PID=$!
sleep 8

echo "Available topics:"
ros2 topic list | grep aehub || echo "No aehub topics found"

echo -e "\nExpected topics:"
echo "  /aehub/commands/navigate_to (input)"
echo "  /aehub/commands/cancel (input)"
echo "  /aehub/events/ack (output)"
echo "  /aehub/events/result (output)"
echo "  /aehub/events/state (output)"

# Cleanup
pkill -P $EXECUTOR_PID || true
wait $EXECUTOR_PID 2>/dev/null || true
sleep 2

echo ""
echo "=========================================="
echo "4. Smoke test: sending command..."
echo "=========================================="

# Start executor
ros2 launch aehub_navigation_executor navigation_executor.launch.py > /tmp/executor_smoke.log 2>&1 &
EXECUTOR_PID=$!
sleep 8

# Send command
CMD_ID="11111111-1111-1111-1111-111111111111"
echo "Sending navigateTo command: $CMD_ID"
ros2 topic pub --once /aehub/commands/navigate_to std_msgs/msg/String "{
  data: '{\"command_id\": \"$CMD_ID\", \"x\": 1.0, \"y\": 2.0, \"timestamp\": 1768840000}'
}"

sleep 2

echo -e "\nChecking for ACK:"
timeout 3 ros2 topic echo /aehub/events/ack --once 2>&1 | head -20 || echo "No ACK received"

# Cleanup
pkill -P $EXECUTOR_PID || true
wait $EXECUTOR_PID 2>/dev/null || true
sleep 2

echo ""
echo "=========================================="
echo "5. Testing deduplication..."
echo "=========================================="

# Start executor
ros2 launch aehub_navigation_executor navigation_executor.launch.py > /tmp/executor_dedup.log 2>&1 &
EXECUTOR_PID=$!
sleep 8

CMD_ID="22222222-2222-2222-2222-222222222222"

echo "Sending first command: $CMD_ID"
ros2 topic pub --once /aehub/commands/navigate_to std_msgs/msg/String "{
  data: '{\"command_id\": \"$CMD_ID\", \"x\": 1.0, \"y\": 2.0, \"timestamp\": 1768840001}'
}"

sleep 2

echo -e "\nSending duplicate command: $CMD_ID"
ros2 topic pub --once /aehub/commands/navigate_to std_msgs/msg/String "{
  data: '{\"command_id\": \"$CMD_ID\", \"x\": 1.0, \"y\": 2.0, \"timestamp\": 1768840002}'
}"

sleep 2

echo -e "\nChecking events (should see replay):"
timeout 5 ros2 topic echo /aehub/events/ack --once 2>&1 | head -20 || echo "No events"

# Cleanup
pkill -P $EXECUTOR_PID || true
wait $EXECUTOR_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "Checklist verification complete!"
echo "=========================================="
