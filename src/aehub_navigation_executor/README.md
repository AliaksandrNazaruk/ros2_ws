# aehub_navigation_executor

Business-layer Nav2 command executor with idempotency.

**Status**: ✅ SKELETON COMPLETE  
**Version**: 0.1.0

## Responsibility

This node:
- Receives navigation commands from ROS (received via MQTT transport)
- Validates and deduplicates commands by `command_id`
- Manages Nav2 action lifecycle
- Publishes events (ack / result / state)
- Provides strictly idempotent command processing

## Non-Responsibility

This node MUST NOT:
- Connect to MQTT (handled by `aehub_mqtt_transport`)
- Read YAML mapping (handled by `aehub_mqtt_transport`)
- Manage broker credentials (handled by `aehub_broker_credentials`)
- Handle transport / retry / TLS

## Architecture Position

```
MQTT Broker
   ↓
aehub_mqtt_transport   (transport layer)
   ↓  ROS topics
aehub_navigation_executor   ← YOU ARE HERE
   ↓
Nav2 Action Server (/navigate_to_pose)
```

## ROS API

### Input Topics (Commands)

| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/aehub/commands/navigate_to` | `std_msgs/String` (JSON) | RELIABLE | Navigation command |
| `/aehub/commands/cancel` | `std_msgs/String` (JSON) | RELIABLE | Cancel active command |

### Output Topics (Events)

| Topic | Type | Purpose |
|-------|------|---------|
| `/aehub/events/ack` | `std_msgs/String` | Ack: received / accepted / rejected |
| `/aehub/events/result` | `std_msgs/String` | Terminal result |
| `/aehub/events/state` | `std_msgs/String` | Current state |

### Nav2 Action

| Action | Type |
|--------|------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

## Command Contract

## Lifecycle

`navigation_executor_node` is a **ROS2 LifecycleNode**.

The provided launch file performs **automatic configure + activate** on start (no manual `ros2 lifecycle set ...`).

## Duplicate Protection (production)

ROS graph-level duplicate node names are possible if a second instance is launched manually.
For production, prefer the provided systemd unit example which uses a `flock` lock.

## systemd

An example unit file is shipped in `share/aehub_navigation_executor/systemd/`.
Typical install flow:

```bash
sudo cp $(ros2 pkg prefix aehub_navigation_executor)/share/aehub_navigation_executor/systemd/aehub-navigation-executor.service.example \
  /etc/systemd/system/aehub-navigation-executor.service
sudo systemctl daemon-reload
sudo systemctl enable --now aehub-navigation-executor.service
```

### Common Fields (ALL commands)
```json
{
  "command_id": "uuid-v4",
  "timestamp": 1768840168
}
```

**Protocol Invariant**: `command_id` MUST be a valid UUID v4 string (RFC 4122 format).
- Non-UUID identifiers (e.g., ULID, numeric IDs) will be rejected by validation
- This is a protocol requirement: deduplication relies on UUID uniqueness guarantees

### navigateTo
```json
{
  "command_id": "...",
  "timestamp": ...,
  "target_id": "dock_A"
}
```
OR
```json
{
  "command_id": "...",
  "timestamp": ...,
  "x": 1.0,
  "y": 2.0,
  "theta": 1.57
}
```

### cancel
```json
{
  "command_id": "...",
  "timestamp": ...
}
```

## State Machine

### Internal States
- `IDLE` → `NAVIGATING` → `SUCCEEDED` | `ABORTED` | `CANCELED` | `ERROR` → `IDLE`
- `IDLE` → `CANCELING` → `CANCELED` → `IDLE`
- `NAVIGATING` → `CANCELING` → `CANCELED` → `IDLE`

### Public States (Events)
- `idle`, `navigating`, `canceling`, `succeeded`, `aborted`, `canceled`, `error`

## Deduplication Rules

- **Storage**: In-memory map `command_id → outcome`
- **Behavior**:
  - First time `command_id`: Execute
  - Duplicate `navigateTo`: Replay ack/result
  - Duplicate `cancel`: Replay canceled
  - Duplicate across topics: Still duplicate
- **TTL**: Configurable (default: 10 min)

## Lifecycle States

| State | Behavior |
|-------|----------|
| `unconfigured` | No subscriptions, no ActionClient |
| `inactive` | ActionClient created, **no subscriptions** |
| `active` | Subscribed to commands, ready to execute |
| `deactivated` | Stop accepting new commands, **do NOT cancel active goal** |
| `cleanup` | Clear caches, reset state |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rate_limit_sec` | double | `1.0` | Rate limit for navigateTo commands (seconds) |
| `command_cache_ttl_sec` | double | `600.0` | Command cache TTL (seconds, default: 10 min) |

## Architecture Principles

1. **Single Responsibility**: Only business logic for command execution
2. **Determinism**: One `command_id` → one logical result
3. **Idempotency**: Repeated command with same `command_id` never triggers second action
4. **Stateless transport, stateful business**: Command state stored only here
5. **ROS-native**: LifecycleNode, ActionClient, explicit QoS

## Package Structure

```
aehub_navigation_executor/
├── src/aehub_navigation_executor/
│   ├── navigation_executor_node.py      # LifecycleNode (entrypoint)
│   ├── command_validator.py              # validation + dedup
│   ├── command_state_machine.py          # states & transitions
│   ├── nav2_action_client.py             # thin Nav2 wrapper
│   └── event_publisher.py                # ack/result abstraction
├── test/
│   ├── unit/
│   │   ├── test_command_validator.py
│   │   └── test_state_machine.py
│   └── integration/
│       └── test_navigation_executor_flow.py
└── launch/
    └── navigation_executor.launch.py
```

## Usage

### Launch

```bash
ros2 launch aehub_navigation_executor navigation_executor.launch.py
```

### Lifecycle Management

```bash
# Configure
ros2 lifecycle set /navigation_executor configure

# Activate
ros2 lifecycle set /navigation_executor activate

# Deactivate
ros2 lifecycle set /navigation_executor deactivate
```

## Acceptance Criteria

- ✅ Duplicate command never triggers second Nav2 goal
- ✅ Cancel without goal never errors
- ✅ Replay produces identical output
- ✅ Node survives restart without crash
- ✅ Lifecycle transitions clean

## Version

**Current version**: `0.1.0`

**Status**: SKELETON COMPLETE (ready for implementation)
