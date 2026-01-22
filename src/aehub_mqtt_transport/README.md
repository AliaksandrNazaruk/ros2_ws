# aehub_mqtt_transport

Pure transport layer between ROS2 and MQTT broker.

**Status**: ✅ SKELETON COMPLETE, LOGIC IMPLEMENTED  
**Version**: 0.1.0

## Responsibility

This node:
- Consumes MQTT broker configuration from ROS topic `/aehub/mqtt/broker_config`
- Manages MQTT connection lifecycle (connect, disconnect, reconnect)
- Provides ROS ↔ MQTT bridge (transport layer only)

### Module Responsibility Split

The package is organized into focused modules with clear boundaries:

- **`mqtt_transport_node.py`**: Lifecycle orchestration, ROS integration, BrokerConfig handling
- **`mqtt_client.py`**: Thin wrapper over paho-mqtt client, isolates MQTT client creation and lifecycle
- **`backoff.py`**: Exponential backoff policy for retry logic, testable in isolation
- **`topic_mapper.py`**: MQTT ↔ ROS topic mapping (placeholder for future implementation)

This separation enables:
- ✅ Unit testing of individual components
- ✅ Code reuse across modules
- ✅ Maintainability and readability

## Non-Responsibility

This node MUST NOT:
- Parse JSON
- Know about Nav2
- Store credentials
- Handle business logic
- Pull HTTP
- Manage system lifecycle

## ROS API

### Subscribes

- **Topic**: `/aehub/mqtt/broker_config`
- **Type**: `aehub_msgs/BrokerConfig`
- **QoS**: `RELIABLE` + `TRANSIENT_LOCAL` (latched)

**Behavior**: Node can start without broker_config. Stays INACTIVE until config received.

### Publishes (TODO - to be implemented)

- **Topic**: `/aehub/mqtt/in/<mqtt_topic>` (sanitized)
- **Type**: `std_msgs/String`
- **Payload**: Raw MQTT message as UTF-8 string

### Subscribes (TODO - to be implemented)

- **Topic**: `/aehub/mqtt/out/<mqtt_topic>`
- **Type**: `std_msgs/String`
- **Payload**: Raw string to publish to MQTT

## Lifecycle States

| State | Behavior |
|-------|----------|
| `unconfigured` | No subscriptions, no MQTT client |
| `inactive` | Subscribed to broker_config, **no MQTT connection** |
| `active` | MQTT connected (if broker_config available) |
| `deactivated` | MQTT disconnected, **no publishing** |
| `cleanup` | Resources released |

**Invariant**: MQTT does NOT connect until BrokerConfig is received.

## Parameters

| Parameter | Type | Default | Required | Description |
|-----------|------|---------|----------|-------------|
| `robot_id` | string | `""` | Yes | Robot ID for MQTT topics |
| `mqtt_connect_timeout_sec` | double | `5.0` | No | MQTT connection timeout |
| `mqtt_reconnect_initial_sec` | double | `1.0` | No | Initial reconnect delay |
| `mqtt_reconnect_max_sec` | double | `30.0` | No | Maximum reconnect delay |

## MQTT Contract

### Default Topics

- **Subscribe**: `aroc/robot/{robot_id}/#` (QoS=1)
- **Publish**: `aroc/robot/{robot_id}/status/transport` (QoS=1, retain=false)

### Connection Settings

- **Clean session**: `true`
- **Keepalive**: `30s`
- **Reconnect**: Infinite, exponential backoff

## Behavior on BrokerConfig Change

When new `BrokerConfig` is received:
1. Compare with current config (hash-based)
2. If changed:
   - Disconnect current MQTT client
   - Reconnect with new credentials
   - Resubscribe to topics
3. ROS topics are NOT recreated

**Guarantee**: Only one MQTT client per lifecycle.

## Error Handling

| Scenario | Behavior |
|----------|----------|
| Broker unavailable | Retry forever with exponential backoff |
| TLS error | Retry + status=ERROR |
| No broker_config | Stay INACTIVE, wait |
| Invalid broker_config | Reject + status |

**Node never crashes. Never.**

## Usage

### Launch

```bash
ros2 launch aehub_mqtt_transport mqtt_transport.launch.py \
  robot_id:=fahrdummy-01-local
```

### Systemd Service

See `docs/examples/aehub-mqtt-transport.service.example`

**Dependencies**: Requires `aehub-broker-credentials.service` to be running.

## Architecture

```
Config Service (HTTP)
        ↓
broker_credentials_node
        ↓   (ROS msg, latched)
mqtt_transport_node
        ↓   (ROS topics)
command / telemetry / nav layers
```

This architecture is:
- ✅ Scalable
- ✅ Testable
- ✅ Production-grade
- ✅ No duplicates (systemd prevents)
- ✅ No hidden dependencies

## Architecture Notes

**NOTE:**  
Command deduplication is intentionally NOT implemented in `TopicMapper`.
Transport layer must remain stateless with respect to command semantics.
Deduplication is responsibility of upper-layer business nodes.

**Why:**
- Transport layer should not know about `command_id`
- Transport should only map topics ↔ topics
- Business logic (deduplication, validation, state) belongs to upper layers
- This keeps the transport layer testable, maintainable, and extensible

## Version

**Current version**: `0.1.0`

**Status**: IN DEVELOPMENT
