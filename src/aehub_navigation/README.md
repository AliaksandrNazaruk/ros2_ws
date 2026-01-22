# DEPRECATED: aehub_navigation (legacy monolith)

This package is **legacy** and **deprecated**.

It mixes multiple architectural layers in one place (Infrastructure + Command Orchestration + Navigation Capability + safety control), which violates the clean architecture rules fixed in:
- `docs/ARCHITECTURE_CLEAN.md`
- `docs/SRS_NAVIGATION.md`

## Do not build new features here

All new work MUST target the clean split:

- **Infrastructure**
  - `aehub_broker_credentials` (HTTP → `/aehub/mqtt/broker_config`)
  - `aehub_mqtt_transport` (MQTT ↔ ROS transport)
- **Command Orchestration**
  - `aehub_navigation_executor` (FSM + idempotency; never `/cmd_vel`)
- **Navigation Capability**
  - `aehub_nav2_capability_server` (sole owner of Nav2 lifecycle, readiness, STOP, `/cmd_vel`)

## Why deprecated

The legacy integrated node(s) here:
- duplicate contracts vs `aehub_msgs`
- duplicate MQTT connection logic vs transport layer
- own STOP burst and `/cmd_vel` (unsafe ownership)
- combine readiness decisions with business logic

They are kept only for migration/compatibility until the new stack is complete.

