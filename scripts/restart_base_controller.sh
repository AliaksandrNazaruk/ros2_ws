#!/bin/bash
# Restart base_controller after rebuilding with fixed endpoint

cd /home/boris/ros2_ws
source install/setup.bash

echo "=========================================="
echo "🔄 Restarting base_controller"
echo "=========================================="
echo ""

# Stop existing base_controller
echo "1. Stopping existing base_controller..."
pkill -f "base_controller_node" || echo "   No base_controller process found"
sleep 2

# Check if stopped
if pgrep -f "base_controller_node" > /dev/null; then
    echo "   ⚠️  Warning: base_controller still running, forcing stop..."
    pkill -9 -f "base_controller_node"
    sleep 1
fi

echo "   ✅ base_controller stopped"
echo ""

# Rebuild if needed
echo "2. Checking if rebuild is needed..."
if [ ! -f "install/base_controller/lib/base_controller/base_controller_node" ]; then
    echo "   Building base_controller..."
    colcon build --packages-select base_controller
    source install/setup.bash
else
    echo "   ✅ base_controller already built"
fi
echo ""

# Start base_controller
echo "3. Starting base_controller with fixed endpoint..."
echo "   Endpoint: /v0/agv/{id}/move/speed (FIXED!)"
echo ""

ros2 run base_controller base_controller_node \
    --ros-args \
    -p driver_endpoint:=https://192.168.1.100 \
    -p amr_id:=15 \
    -p tls_verify:=false &

BASE_PID=$!
echo "   base_controller started (PID: $BASE_PID)"
echo ""

# Wait a bit and check status
sleep 3
if ps -p $BASE_PID > /dev/null; then
    echo "   ✅ base_controller is running"
else
    echo "   ❌ base_controller failed to start"
    exit 1
fi

echo ""
echo "=========================================="
echo "✅ base_controller restarted successfully!"
echo "=========================================="
echo ""
echo "Now test robot movement:"
echo "  python3 scripts/test_robot_command.py"
echo ""
