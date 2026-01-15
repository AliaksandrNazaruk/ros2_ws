#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/boris/ros2_ws"
UNIT_SRC="$REPO_ROOT/scripts/aehub_nav2.service"
ENV_EXAMPLE_SRC="$REPO_ROOT/config/aehub_nav2.env.example"

UNIT_DST="/etc/systemd/system/aehub_nav2.service"
ENV_DST="/etc/aehub_nav2.env"

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "Missing unit file: $UNIT_SRC" >&2
  exit 1
fi

echo "Installing systemd unit to: $UNIT_DST"
if ! sudo -n true 2>/dev/null; then
  echo "ERROR: sudo requires a password in this environment." >&2
  echo "Run this script in an interactive terminal and enter sudo password, or install manually:" >&2
  echo "  sudo cp -f \"$UNIT_SRC\" \"$UNIT_DST\"" >&2
  echo "  sudo cp -f \"$ENV_EXAMPLE_SRC\" \"$ENV_DST\"   # if missing" >&2
  echo "  sudo systemctl daemon-reload" >&2
  echo "  sudo systemctl enable aehub_nav2.service" >&2
  exit 2
fi

sudo cp -f "$UNIT_SRC" "$UNIT_DST"

if [[ ! -f "$ENV_DST" ]]; then
  echo "Creating env file: $ENV_DST (from example)"
  sudo cp -f "$ENV_EXAMPLE_SRC" "$ENV_DST"
  sudo chmod 600 "$ENV_DST"
  echo "IMPORTANT: edit $ENV_DST and set CONFIG_SERVICE_API_KEY (and other params)."
else
  echo "Env file already exists: $ENV_DST (leaving as-is)"
fi

echo "Reloading systemd daemon"
sudo systemctl daemon-reload

echo "Enabling service"
sudo systemctl enable aehub_nav2.service

echo "Done. Use:"
echo "  sudo systemctl start aehub_nav2.service"
echo "  sudo systemctl status aehub_nav2.service --no-pager"
echo "  sudo journalctl -u aehub_nav2.service -f"

