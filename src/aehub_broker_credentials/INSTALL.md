# Installation Guide

## Quick Start

```bash
# 1. Install service
sudo cp docs/examples/aehub-broker-credentials.service.example \
        /etc/systemd/system/aehub-broker-credentials.service

# 2. Enable and start
sudo systemctl daemon-reload
sudo systemctl enable aehub-broker-credentials.service
sudo systemctl start aehub-broker-credentials.service

# 3. Verify
systemctl status aehub-broker-credentials.service
ros2 lifecycle get /broker_credentials_node
```

## Production Installation (systemd)

### Step 1: Configure Packaged YAML

Configure credentials in the packaged YAML:

- `share/aehub_broker_credentials/config/broker_credentials.yaml`

### Step 2: Install Systemd Service

```bash
# Copy service file
sudo cp docs/examples/aehub-broker-credentials.service.example \
        /etc/systemd/system/aehub-broker-credentials.service

# Edit if needed (adjust User, Group, paths)
sudo nano /etc/systemd/system/aehub-broker-credentials.service

# Reload systemd
sudo systemctl daemon-reload
```

### Step 3: Enable and Start

```bash
sudo systemctl enable aehub-broker-credentials.service
sudo systemctl start aehub-broker-credentials.service
```

### Step 4: Verify

```bash
# Check service status
systemctl status aehub-broker-credentials.service

# Check ROS node
ros2 node list | grep broker

# Check lifecycle state
ros2 lifecycle get /broker_credentials_node

# Check published topic
ros2 topic echo /aehub/mqtt/broker_config
```

### Step 5: Configure Dependencies

Other services (e.g., `aehub-stack.service`) should depend on this service:

```ini
[Unit]
After=aehub-broker-credentials.service
Requires=aehub-broker-credentials.service
```

## Development Installation

For manual testing, see README.md "Manual Launch" section.

## Troubleshooting

### Service fails to start

1. Check logs: `journalctl -u aehub-broker-credentials.service -n 50`
2. Verify packaged YAML is present/configured: `ros2 pkg prefix aehub_broker_credentials`
3. Verify ROS2 workspace: `source /home/boris/ros2_ws/install/setup.bash && ros2 pkg list | grep aehub_broker_credentials`

### Node not visible in ROS graph

1. Check ROS_DOMAIN_ID matches
2. Verify service is running: `systemctl status aehub-broker-credentials.service`
3. Check lifecycle state: `ros2 lifecycle get /broker_credentials_node`
