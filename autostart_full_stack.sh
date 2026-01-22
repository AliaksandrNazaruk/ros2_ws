#!/usr/bin/env bash
set -euo pipefail

# Autostart: full MQTT navigation stack (Symovo + Nav2).
#
# Usage:
#   ./autostart_full_stack.sh [ros2 launch args...]
# Example:
#   ./autostart_full_stack.sh symovo_endpoint:=https://192.168.1.100 amr_id:=15 tls_verify:=false

WS="/home/boris/ros2_ws"
cd "$WS"

# Don't run the ROS graph as root (will create duplicates + /root/.ros).
if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  echo "ERROR: do not run this script with sudo/root. Run as user 'boris'." >&2
  exit 1
fi

# Best-effort cleanup to avoid duplicate stacks.
pkill -9 -f "ros2 launch aehub_broker_credentials broker_credentials.launch.py" 2>/dev/null || true
pkill -9 -f "full_mqtt_navigation_stack.launch.py" 2>/dev/null || true
pkill -9 -f "symovo_nav2.launch.py" 2>/dev/null || true
pkill -9 -f "static_tf_base_laser" 2>/dev/null || true
pkill -9 -f "static_tf_map_odom" 2>/dev/null || true
pkill -9 -f "broker_credentials_node" 2>/dev/null || true
pkill -9 -f "mqtt_transport_node" 2>/dev/null || true
pkill -9 -f "mqtt_protocol_adapter_node" 2>/dev/null || true
pkill -9 -f "navigation_executor_node" 2>/dev/null || true
pkill -9 -f "aehub_nav2_capability_server" 2>/dev/null || true
pkill -9 -f "nav2_adapter" 2>/dev/null || true
pkill -9 -f "base_controller_node" 2>/dev/null || true
pkill -9 -f "symovo_scan_converter" 2>/dev/null || true
pkill -9 -f "symovo_scan_converter.py" 2>/dev/null || true
pkill -9 -f "symovo_map_mirror" 2>/dev/null || true
pkill -9 -f "nav2_localization_orchestrator" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "behavior_server" 2>/dev/null || true
pkill -9 -f "waypoint_follower" 2>/dev/null || true
pkill -9 -f "velocity_smoother" 2>/dev/null || true

# Remove stale stack lock file (safe because we killed old stack processes above).
rm -f /tmp/symovo_nav2_stack.robot_001.lock 2>/dev/null || true
rm -f /tmp/base_controller.lock 2>/dev/null || true

# FastDDS SHM can get stuck if previous runs were started with sudo/root.
# Try to clean stale SHM port files. If they are root-owned, this will silently fail
# and you should run the sudo cleanup command below once:
#   sudo rm -f /dev/shm/fastrtps_* /dev/shm/ros2_*
rm -f /dev/shm/fastrtps_* /dev/shm/ros2_* 2>/dev/null || true

# ROS setup scripts may reference unset variables; avoid nounset failures.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# FastDDS shared memory transport often fails on embedded systems and can cause service timeouts.
# Disable SHM to make lifecycle/service calls more reliable.
export RMW_FASTRTPS_USE_SHM=0

# Optional centralized env file (API keys, robot_id, etc.)
# Don't fail if it's root-readable only.
if [[ -r /etc/aehub/aehub_stack.env ]]; then
  # shellcheck disable=SC1091
  source /etc/aehub/aehub_stack.env
fi

# Ensure ROS env vars from env file are exported to child processes.
# (Sourcing sets shell vars but does not export them by default.)
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID
fi
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION
fi

# Preflight: without symovo_bridge scan converter, /scan will have no publishers and Nav2 won't fully activate.
if ! ros2 pkg prefix symovo_bridge >/dev/null 2>&1; then
  echo "WARNING: package 'symovo_bridge' is not installed/built. /scan will be missing; Nav2 navigation may stay inactive." >&2
  echo "         Build it:  cd /home/boris/ros2_ws && colcon build --packages-select symovo_bridge && source install/setup.bash" >&2
fi

exec ros2 launch aehub_navigation_executor full_mqtt_navigation_stack.launch.py "$@"

