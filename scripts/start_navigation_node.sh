#!/bin/bash

# Start navigation_integrated_node
# This node handles MQTT commands and sends goals to Nav2

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f /home/boris/ros2_ws/install/setup.bash ]; then
    source /home/boris/ros2_ws/install/setup.bash
fi

# Get API key from .env file
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/nav2_test_server/.env"

if [ -f "$ENV_FILE" ]; then
    # Extract API key from .env
    API_KEY=$(grep "CONFIG_SERVICE_API_KEY" "$ENV_FILE" | cut -d '=' -f2 | tr -d ' ')
else
    echo "Warning: .env file not found at $ENV_FILE"
    echo "Using default empty API key (will fail if Config Service requires auth)"
    API_KEY=""
fi

echo "Starting navigation_integrated_node..."
echo "Config Service URL: http://localhost:7900"
echo "Robot ID: robot_001"
echo ""

# Start navigation node
python3 -m aehub_navigation.navigation_integrated_node \
    --ros-args \
    -p robot_id:=robot_001 \
    -p config_service_url:=http://localhost:7900 \
    -p config_service_api_key:="$API_KEY"

