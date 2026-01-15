#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-aehub_nav2.service}"

need_sudo() {
  if command -v sudo >/dev/null 2>&1; then
    echo "sudo"
  else
    echo ""
  fi
}

SUDO="$(need_sudo)"

usage() {
  cat <<'EOF'
Usage: aehub_nav2_ctl.sh <command>

Commands:
  start       Start the service
  stop        Stop the service
  restart     Restart the service
  status      Show service status (no pager)
  logs        Follow journal logs
  nodes       Show current ros2 node list (uses /opt/ros/jazzy + workspace overlay)

Environment:
  SERVICE_NAME   systemd unit name (default: aehub_nav2.service)
EOF
}

cmd="${1:-}"
case "$cmd" in
  start)
    $SUDO systemctl start "$SERVICE_NAME"
    ;;
  stop)
    $SUDO systemctl stop "$SERVICE_NAME"
    ;;
  restart)
    $SUDO systemctl restart "$SERVICE_NAME"
    ;;
  status)
    $SUDO systemctl status --no-pager "$SERVICE_NAME" || true
    ;;
  logs)
    $SUDO journalctl -u "$SERVICE_NAME" -f
    ;;
  nodes)
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source /home/boris/ros2_ws/install/setup.bash
    ros2 node list || true
    ;;
  ""|-h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage >&2
    exit 2
    ;;
esac

