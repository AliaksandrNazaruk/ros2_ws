#!/bin/bash
# Configure Nav2 to work with odom frame instead of map frame

cd /home/boris/ros2_ws
source install/setup.bash

echo "=========================================="
echo "🔧 Configuring Nav2 for odom frame"
echo "=========================================="
echo ""

CONFIG_FILE="/home/boris/ros2_ws/config/nav2_symovo_params.yaml"
BACKUP_FILE="${CONFIG_FILE}.backup_$(date +%Y%m%d_%H%M%S)"

# Create backup
cp "$CONFIG_FILE" "$BACKUP_FILE"
echo "✅ Backup created: $BACKUP_FILE"
echo ""

# Check current configuration
echo "Current global_frame in global_costmap:"
grep -A 2 "global_costmap:" "$CONFIG_FILE" | grep "global_frame" || echo "  Not found"

echo ""
echo "Current global_frame in bt_navigator:"
grep -A 2 "bt_navigator:" "$CONFIG_FILE" | grep "global_frame" || echo "  Not found"

echo ""
echo "⚠️  WARNING: This script will modify the configuration file."
echo "   Make sure to test navigation after changes."
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 1
fi

# Modify global_frame from map to odom
echo ""
echo "Modifying global_frame from 'map' to 'odom'..."
sed -i 's/global_frame: map/global_frame: odom/g' "$CONFIG_FILE"

echo "✅ Configuration updated"
echo ""
echo "Changes made:"
echo "- global_costmap.global_frame: map -> odom"
echo "- bt_navigator.global_frame: map -> odom (if present)"
echo ""
echo "⚠️  IMPORTANT: Restart Nav2 nodes for changes to take effect!"
echo "   Run: ros2 lifecycle set /planner_server shutdown"
echo "   Then restart the launch file"
echo ""
