#!/bin/bash

# Start Nav2 Testing Server

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f /home/boris/ros2_ws/install/setup.bash ]; then
    source /home/boris/ros2_ws/install/setup.bash
fi

# Change to server directory
cd /home/boris/ros2_ws/scripts/nav2_test_server

# Start server using system Python (not virtual environment)
echo "Starting Nav2 Testing Server..."
echo "Dashboard: http://localhost:8000/dashboard"
echo "API Docs: http://localhost:8000/docs"
echo "Using system Python: $(which python3)"
echo ""

python3 -m uvicorn main:app \
  --host 0.0.0.0 \
  --port 8000 \
  --proxy-headers \
  --forwarded-allow-ips="*"

