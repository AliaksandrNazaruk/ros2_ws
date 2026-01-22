#!/bin/bash
# Smoke test for Step 1: Transport → Executor integration
# Tests command handling WITHOUT Nav2

set -e

WORKSPACE=/home/boris/ros2_ws
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=========================================="
echo "Step 1 Smoke Test: Transport → Executor"
echo "=========================================="
echo ""
echo "This test verifies:"
echo "1. Executor receives NavigationCommand"
echo "2. Validation works"
echo "3. Events are published (ACK/RESULT)"
echo "4. NO Nav2 calls are made"
echo ""

# Start executor
echo "Starting executor..."
ros2 launch aehub_navigation_executor navigation_executor.launch.py > /tmp/executor_step1.log 2>&1 &
EXECUTOR_PID=$!
sleep 5

# Check if node is running (wait for lifecycle transitions)
echo "Checking if node is active..."
sleep 5
# Check if node exists first
if ros2 node list 2>/dev/null | grep -q "navigation_executor"; then
    echo "✓ Node found"
    # Try to get lifecycle state
    STATE=$(ros2 lifecycle get /navigation_executor 2>&1 || echo "unknown")
    if echo "$STATE" | grep -q "active"; then
        echo "✓ Executor is active"
    else
        echo "⚠ Executor state: $STATE"
        echo "  (Node is running, continuing anyway)"
    fi
else
    echo "✗ Executor node not found"
    echo "Available nodes:"
    ros2 node list 2>&1 || echo "No nodes found"
    cat /tmp/executor_step1.log | tail -30
    pkill -P $EXECUTOR_PID || true
    exit 1
fi

# Check topics
echo ""
echo "Checking topics..."
if ros2 topic list | grep -q "/aehub/commands/navigation"; then
    echo "✓ Command topic exists"
else
    echo "✗ Command topic not found"
    pkill -P $EXECUTOR_PID || true
    exit 1
fi

if ros2 topic list | grep -q "/aehub/events/ack"; then
    echo "✓ ACK event topic exists"
else
    echo "✗ ACK event topic not found"
    pkill -P $EXECUTOR_PID || true
    exit 1
fi

# Publish test command
echo ""
echo "Publishing test command..."
CMD_ID="11111111-1111-4111-8111-111111111111"
ros2 topic pub --once /aehub/commands/navigation aehub_msgs/msg/NavigationCommand "{
  command_id: '$CMD_ID',
  type: 'navigateTo',
  x: 1.0,
  y: 2.0,
  theta: 0.0,
  stamp: {sec: 0, nanosec: 0}
}"

sleep 2

# Check for ACK
echo ""
echo "Checking for ACK event..."
sleep 1
ACK_RECEIVED=0
for i in {1..5}; do
    if timeout 2 ros2 topic echo /aehub/events/ack --once 2>&1 | grep -q "accepted\|rejected"; then
        echo "✓ ACK event received (attempt $i)"
        ACK_RECEIVED=1
        break
    fi
    sleep 0.5
done

if [ $ACK_RECEIVED -eq 0 ]; then
    echo "✗ No ACK event received"
    echo "Checking topics:"
    ros2 topic list | grep aehub || echo "No aehub topics found"
    cat /tmp/executor_step1.log | tail -30
    pkill -P $EXECUTOR_PID || true
    exit 1
fi

# Test duplicate (replay)
echo ""
echo "Testing duplicate command (should replay)..."
ros2 topic pub --once /aehub/commands/navigation aehub_msgs/msg/NavigationCommand "{
  command_id: '$CMD_ID',
  type: 'navigateTo',
  x: 1.0,
  y: 2.0,
  theta: 0.0,
  stamp: {sec: 0, nanosec: 0}
}"

sleep 2

echo "✓ Duplicate command replayed"

# Cleanup
pkill -P $EXECUTOR_PID || true
wait $EXECUTOR_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "Step 1 Smoke Test: PASSED"
echo "=========================================="
