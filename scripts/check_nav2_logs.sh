#!/bin/bash
# Check Nav2 logs for planner errors

cd /home/boris/ros2_ws
source install/setup.bash

echo "=========================================="
echo "🔍 Checking Nav2 Logs for Planner Errors"
echo "=========================================="
echo ""

echo "1. Checking for planner errors..."
timeout 10 ros2 topic echo /rosout 2>&1 | grep -iE "planner|compute.*path|path.*error|path.*failed|costmap.*error|tf.*error" | head -30 &
sleep 5
pkill -f "ros2 topic echo /rosout"

echo ""
echo "2. Checking planner server status..."
ros2 lifecycle get /planner_server 2>&1

echo ""
echo "3. Checking controller server status..."
ros2 lifecycle get /controller_server 2>&1

echo ""
echo "4. Checking for TF errors in Nav2..."
timeout 5 ros2 topic echo /rosout 2>&1 | grep -iE "tf.*error|transform.*error|frame.*not.*exist" | head -10 &
sleep 3
pkill -f "ros2 topic echo /rosout"

echo ""
echo "=========================================="
echo "✅ Log check complete"
echo "=========================================="
