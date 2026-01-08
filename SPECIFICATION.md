# AE.HUB Robot Navigation – Specification

## 1. Overview
The robot and hub communicate exclusively via MQTT. This document defines topics, payload schemas, module responsibilities, state transitions, and operational instructions for the robot-side navigation node.

- Single integrated ROS2 node: `navigation_integrated_node`
- External broker config is fetched from a centralized Config Service
- TLS-first security; reconnect and resubscribe supported without node restart
- Deterministic per-command outcome is published via dedicated events channel

## 2. MQTT Topics and QoS
- Commands
  - `aroc/robot/{robot_id}/commands/navigateTo` (QoS 1, retained=false)
  - `aroc/robot/{robot_id}/commands/cancel` (QoS 1, retained=false)
- Command events (authoritative outcome per command_id)
  - `aroc/robot/{robot_id}/commands/events` (QoS 1, retained=false)
- Status (global telemetry; not tied to single command outcome)
  - `aroc/robot/{robot_id}/status/navigation` (QoS 1, retained=false)

## 3. JSON Schemas
All payloads include `schema_version: "1.0"` and `robot_id`.

### 3.1 NavigateTo (command)
```json
{
  "schema_version": "1.0",
  "robot_id": "string",
  "command_id": "uuid",
  "timestamp": "ISO-8601 Z",
  "target_id": "string",
  "priority": "normal|high|emergency"
}
```
Validation:
- `command_id` must be UUIDv4
- `target_id` must exist in Position Registry
- Required fields must be present and typed

### 3.2 Cancel (command)
```json
{
  "schema_version": "1.0",
  "robot_id": "string",
  "command_id": "uuid",
  "timestamp": "ISO-8601 Z"
}
```

### 3.3 Command Events (authoritative, per command_id)
Topic: `aroc/robot/{robot_id}/commands/events`

Common fields:
```json
{
  "schema_version": "1.0",
  "robot_id": "string",
  "timestamp": "ISO-8601 Z",
  "event_type": "ack|result",
  "command_id": "uuid",
  "target_id": "string|null"
}
```

ACK event:
```json
{
  "event_type": "ack",
  "ack_type": "received|accepted|rejected",
  "reason": "string (optional)"
}
```

RESULT event:
```json
{
  "event_type": "result",
  "result_type": "succeeded|aborted|canceled|error",
  "error_code": "string (optional)",
  "error_message": "string (optional)"
}
```

Ordering guarantees for each `command_id`:
1) `ack(received)` at ingress
2) `ack(accepted|rejected)` after validation/goal response
3) Terminal `result(...)` exactly once

### 3.4 Navigation Status (global telemetry)
Topic: `aroc/robot/{robot_id}/status/navigation`
```json
{
  "schema_version": "1.0",
  "robot_id": "string",
  "timestamp": "ISO-8601 Z",
  "status": "idle|navigating|paused|error|canceling|succeeded|aborted",
  "target_id": "string|null",
  "command_id": "uuid|null",
  "progress_percent": "number|null",
  "eta_seconds": "number|null",
  "current_position": { "x": 0.0, "y": 0.0, "theta": 0.0 },
  "error_code": "string|null",
  "error_message": "string|null"
}
```
Notes:
- Status is telemetry; it must not override per-command outcome.
- Status never publishes `N/A` values; falls back to safe defaults.

## 4. Config Service API
- Endpoint: `/api/v1/config/broker` (GET, requires `X-API-Key`)
- TLS CA endpoint (optional): `/api/v1/config/certificates/ca` (GET, `X-API-Key`)
BrokerConfig fields:
- `broker` (host), `broker_port` (int), `mqtt_use_tls` (bool), `mqtt_tls_insecure` (bool)
- `mqtt_user` (optional), `mqtt_password` (optional)

## 5. Modules and Responsibilities

### 5.1 MQTTConnectionManager
- Connects (async), manages callbacks (connect/disconnect/message)
- TLS configuration (fail-closed or insecure fallback per setting)
- Persists subscriptions across reconnects
- Exponential backoff for `reconnect()`:
  - `reconnect_backoff_enabled` (default: true)
  - `reconnect_backoff_initial_s` (1.0), `reconnect_backoff_max_s` (60.0)
  - `reconnect_backoff_multiplier` (2.0), `reconnect_max_attempts` (10; 0=∞)

### 5.2 BrokerConfigProvider
- Fetch initial broker config
- Poll for changes and invoke `on_config_changed(new_config)`
- Circuit breaker semantics (retry with intervals)

### 5.3 MQTTCommandEventPublisher
- Publishes per-command `ack` and `result` events (QoS 1)
- Single source of truth for command outcomes

### 5.4 MQTTStatusPublisher
- Publishes global navigation status (QoS 1)
- Enforces non-empty status (defaults to `idle` when needed)

### 5.5 CommandValidator
- Validates command structure and types
- Ensures `target_id` exists via Position Registry
- Deduplicates processed `command_id` with TTL

### 5.6 CommandRateLimiter
- Enforces minimum interval between commands (global key)
- `min_interval_seconds` is configurable via ROS parameter

### 5.7 NavigationStateManager
- Manages high-level states: `IDLE`, `NAVIGATING`, `CANCELING`, `ERROR`, etc.
- Notifies `on_state_change` → status publisher update + publish

### 5.8 NavigationActionClient (Nav2)
- Sends `NavigateToPose` goals
- Handles feedback/result, maps to events and status updates
- Cancels active goal upon cancel command

### 5.9 Error Handling
- Centralized error handler emits:
  - Status with `NavigationState.ERROR` + `error_code/message`
  - Command `result(error)` via events channel
- Sample error codes: `NAV_INVALID_COMMAND`, `NAV_INVALID_TARGET`, `NAV_RATE_LIMIT_EXCEEDED`, `NAV_SERVER_UNAVAILABLE`, `NAV_GOAL_REJECTED`

## 6. Node Integration Flow
1) Fetch broker config from Config Service
2) Connect MQTT (async), set `_mqtt_ready=True` in `on_connect`
3) Subscribe to `commands/navigateTo` and `commands/cancel`
4) On command:
   - Publish `ack(received)` (events)
   - Rate-limit check (global key)
   - Validation (structure, `target_id` existence, dedup)
   - If rejected: `ack(rejected)` + `result(error)`; update status to `error`
   - If accepted: `ack(accepted)`, send Nav2 goal
   - On Nav2 result: publish terminal `result` (`succeeded|aborted|canceled|error`)
   - Status publishes reflect global navigation state and progress

## 7. Security
- TLS with CA from Config Service (fail-closed option)
- Username/password optional; API key required for Config Service
- No retained messages on commands/events/status
- Recommend broker-side ACLs per `robot_id`

## 8. Dev/Testing Instructions
### 8.1 Unique Robot ID (dev only)
- Enable `auto_unique_robot_id` ROS param or set `AEHUB_AUTO_UNIQUE_ROBOT_ID=1`
  - Resulting `robot_id` format: `{hostname}-{base}-{hash}`

### 8.2 Fake Hub and Tests
- Fake Hub: `scripts/fake_hub.py` subscribes to status and events; caches first error per `command_id`
- Extended tests: `scripts/test_chain_extended.py`
  - Use env `ROBOT_ID=<unique>` when running tests
  - Tests await `commands/events` for deterministic outcomes

### 8.3 Runbook (example)
```bash
# Start node (example; ensure API key and config URL are set)
ros2 run aehub_navigation navigation_integrated_node \
  --ros-args \
  -p robot_id:=robot_001 \
  -p config_service_url:=http://localhost:7900 \
  -p config_service_api_key:=<API_KEY> \
  -p rate_limit_interval:=0.1

# Run extended tests against shared broker with unique robot_id
ROBOT_ID=myhost-robot_001-abc123 python3 scripts/test_chain_extended.py
```

## 9. Operational Considerations
- Reconnect backoff to protect broker during outages
- Do not tie UI decisions to status-only; always consume events for verdicts
- Publish initial status after (re)connect
- Avoid duplicate node instances (warn if detected)

## 10. Glossary
- ACK: Acknowledgment of command ingest/acceptance
- RESULT: Terminal outcome per command
- Telemetry Status: Periodic/global state for UI


