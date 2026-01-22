# aehub_broker_credentials

Lifecycle infrastructure node for fetching and publishing MQTT broker configuration.

**Status**: ✅ PRODUCTION-GRADE, FROZEN (bugfix only)  
**Quick Install**: See [INSTALL.md](INSTALL.md) for systemd service setup

## Responsibility

This node:
- Fetches MQTT broker configuration from Config Service (`/api/v1/config/broker`)
- Validates configuration data
- Detects configuration changes (via SHA256 hash)
- Publishes configuration to ROS topic `/aehub/mqtt/broker_config`

## Non-Responsibility

This node MUST NOT:
- Connect to MQTT broker
- Handle TLS setup
- Reconnect MQTT connections
- Retry MQTT operations
- Interpret MQTT commands
- Interact with Nav2
- Handle business logic

## ROS API

### Publishes

- **Topic**: `/aehub/mqtt/broker_config`
- **Type**: `aehub_msgs/BrokerConfig`
- **QoS**: 
  - Reliability: `RELIABLE`
  - Durability: `TRANSIENT_LOCAL`
  - Depth: `1`

**Important**: Late-joining subscribers (e.g., `mqtt_transport_node`) will receive the last published configuration immediately upon subscription.

### Message Fields

```
string broker
uint16 broker_port
string mqtt_user
string mqtt_password
bool mqtt_use_tls
bool mqtt_tls_insecure
builtin_interfaces/Time fetched_at
```

## Parameters

| Parameter | Type | Default | Required | Description |
|-----------|------|---------|----------|-------------|
| `config_service_url` | string | (from packaged YAML) | No* | Base URL of Config Service (e.g., `https://mqtt.techvisioncloud.pl`) |
| `api_key` | string | (from packaged YAML) | No* | API key for authentication (X-API-Key header) |

\* Not required if defaults are configured in packaged YAML (`share/aehub_broker_credentials/config/broker_credentials.yaml`)
| `poll_interval_sec` | double | `5.0` | No | Polling interval in seconds |
| `request_timeout_sec` | double | `2.0` | No | HTTP request timeout in seconds |
| `fail_on_startup` | bool | `false` | No | Fail activation if initial fetch fails |

## Lifecycle States

| State | Behavior |
|-------|----------|
| `unconfigured` | No HTTP client, no timer |
| `inactive` | HTTP client exists, timer stopped, **no publishing** |
| `active` | HTTP client + timer + periodic polling + publishing |
| `deactivated` | Timer stopped, **no publishing** |
| `cleanup` | Resources released |
| `shutdown` | No side effects |

**Invariant**: In `inactive` and below states, **no publishing** occurs.

## Error Handling

- **Config Service unavailable**: Node continues running, logs WARN, retries on next poll
- **Bad JSON / validation error**: Node continues running, logs ERROR, retries on next poll
- **TLS / HTTP error**: Node continues running, logs WARN/ERROR, retries on next poll
- **Initial fetch failure** (if `fail_on_startup=true`): Activation fails, node remains in `inactive` state

This behavior ensures the infrastructure node does not crash the system.

## Usage

### Configuration

Credentials can be provided in three ways (in order of precedence):

1. **Launch arguments** (highest priority)
2. **Packaged YAML defaults**: `share/aehub_broker_credentials/config/broker_credentials.yaml`
3. **Environment variables** (for systemd services)

**Note**: If defaults are configured in packaged YAML, you can launch without any arguments:

```bash
# Use packaged YAML defaults (no arguments needed):
ros2 launch aehub_broker_credentials broker_credentials.launch.py
```

#### Launch Arguments (Optional - to override defaults)

```bash
ros2 launch aehub_broker_credentials broker_credentials.launch.py \
  config_service_url:=http://localhost:7900 \
  api_key:=your-api-key-here
```

#### Command Line (Direct)

```bash
ros2 run aehub_broker_credentials broker_credentials_node \
  --ros-args \
  -p config_service_url:=http://localhost:7900 \
  -p api_key:=your-api-key-here

# In another terminal:
ros2 lifecycle set /broker_credentials_node configure
ros2 lifecycle set /broker_credentials_node activate

# Check published configuration:
ros2 topic echo /aehub/mqtt/broker_config
```

### Production Setup

#### Systemd Service (Recommended for Production)

**Step 1: Configure packaged YAML**

Edit credentials in:

- `share/aehub_broker_credentials/config/broker_credentials.yaml`

**Step 2: Create systemd service**

```bash
sudo cp docs/examples/aehub-broker-credentials.service.example \
        /etc/systemd/system/aehub-broker-credentials.service
```

**Step 3: Edit service file** (adjust User, Group, paths as needed)

**Step 4: Enable and start**

```bash
sudo systemctl daemon-reload
sudo systemctl enable aehub-broker-credentials.service
sudo systemctl start aehub-broker-credentials.service
```

**Step 5: Verify**

```bash
systemctl status aehub-broker-credentials.service
ros2 node list | grep broker
ros2 lifecycle get /broker_credentials_node
```

**Important**: Other services (e.g., `aehub-stack.service`) should depend on this service:

```ini
[Unit]
After=aehub-broker-credentials.service
Requires=aehub-broker-credentials.service
```

#### Manual Launch (Development)

```bash
# Use packaged YAML defaults (recommended - no arguments needed):
ros2 launch aehub_broker_credentials broker_credentials.launch.py

# Override defaults via launch args (optional):
ros2 launch aehub_broker_credentials broker_credentials.launch.py \
  config_service_url:=http://localhost:7900 \
  api_key:=your-api-key-here
```

**Security Note**: Never commit API keys to git. Use secure configuration management.

## Architecture

This node follows the **separation of concerns** principle:

- **broker_client.py**: Minimal HTTP client (no retries, no circuit breaker)
- **broker_credentials_node.py**: Lifecycle orchestration, validation, change detection

The node is designed as a **pure infrastructure component** that provides configuration to downstream nodes (e.g., `mqtt_transport_node`) without any business logic.

## Version

**Current version**: `0.1.0`

**Status**: FROZEN (bugfix only)

The ROS API contract (topic, message, QoS) is fixed and should not change without version bump.
