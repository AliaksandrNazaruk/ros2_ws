### MQTT-Controlled Navigation Stack (Production Specification)

### 1) Purpose
This document specifies the **production** stack that enables robot navigation control over MQTT, backed by ROS 2 and a **replaceable navigation backend**.

**Goals**
- Accept navigation commands via MQTT and execute them via Nav2.
- Emit deterministic command events (`ack`, `state`, `result`) via MQTT.
- Enforce safety and readiness gating before motion is commanded.
- Run in the **root ROS namespace** (no `/robot/<id>/...` ROS namespaces); `robot_id` is used for MQTT topic routing.

**Non-goals**
- Defining the robot’s motor controller implementation details.
- Simulation, SITL, Gazebo, loopback, or any non-production profiles (explicitly excluded).

---

### 2) High-Level Architecture

**Command flow**
- MQTT Broker → `mqtt_transport_node` → `mqtt_protocol_adapter_node`
- Protocol adapter → `aehub_navigation_executor` (business logic, idempotency)
- Executor → **capability action** `capabilities/navigation/execute` (`aehub_msgs/action/NavigationExecute`)
- Capability implementation (profile-dependent):
  - **Profile A (ROS-owned Nav2)**: `aehub_nav2_capability_server` → Nav2 action `/navigate_to_pose`
  - **Profile B (Machine-owned)**: `navigation_backend_server` → Symovo REST (Transport engine)
- Terminal outcome → capability server → executor → adapter → transport → MQTT events

**Readiness model**
- Navigation execution is gated by a **composite readiness** decision:
  - **Readiness sources are backend-specific**:
    - Profile A: Nav2 lifecycle/action/TF evidence + robot interface evidence
    - Profile B: Symovo REST (AMR `state_flags`, transport busy/idle); ROS topics are *observability only*

---

### 2.1 Backend profiles (production)

#### Profile A: ROS-owned Nav2 backend (legacy / optional)
- AROC Connector owns Nav2 bringup and interacts with Nav2 via ROS actions/services.
- Readiness uses ROS introspection (lifecycle, TF, action availability) + robot interface signals.

#### Profile B: Machine-owned backend (recommended when Nav2 is internal and inaccessible)
- AROC Connector **does not own Nav2** and does not rely on `/cmd_vel`, `/odom`, TF, or lifecycle control as a source of truth.
- The machine HTTP API is the **source of truth** for readiness, busy/idle, and navigation outcome.
- `navigateTo` is **addressed by `target_id`** (predefined mapping `target_id -> station_id` in Symovo).

---

### 2.2 Optional admin node: Station Adapter (positions management)

If the deployment allows editing stations on the machine, can run an **admin/ops** node to manage Symovo Stations:
- **Package**: `aehub_station_adapter`
- **Node**: `station_adapter` (Lifecycle)
- **API**: Symovo Station CRUD
  - `GET /v0/station`, `GET /v0/station/{id}`
  - `POST /v0/station`, `PUT /v0/station/{id}`, `DELETE /v0/station/{id}`
- **ROS services** (JSON payloads for flexibility):
  - `stations/list` (`aehub_msgs/srv/ListStations`)
  - `stations/get` (`aehub_msgs/srv/GetStation`)
  - `stations/create` (`aehub_msgs/srv/CreateStation`)
  - `stations/update` (`aehub_msgs/srv/UpdateStation`)
  - `stations/delete` (`aehub_msgs/srv/DeleteStation`)

This node is **not** required for normal navigation operation; it exists to simplify provisioning and maintenance of the station database.

---

### 3) Runtime Components (ROS 2 Nodes)

### 3.1 MQTT Layer

#### 3.1.1 `broker_credentials_node` (`aehub_broker_credentials`)
**Type**: Lifecycle node  
**Responsibility**
- Poll a configuration service to obtain broker connection data (host/port/TLS/user/password).
- Publish the latest broker config on a ROS topic.

**Key parameters**
- `config_service_url` (string)
- `api_key` (string)
- `broker_config_topic` (string, default: `infra/mqtt/broker_config`)

#### 3.1.2 `mqtt_transport_node` (`aehub_mqtt_transport`)
**Type**: Lifecycle node  
**Responsibility**
- Subscribe to `BrokerConfig` and connect to the MQTT broker.
- Subscribe to MQTT command topics for the configured `robot_id`.
- Publish MQTT events (outbound) produced by downstream nodes.
- Enforce TLS validation using a CA certificate when TLS is enabled.

**Key parameters**
- `robot_id` (string) — used in MQTT topic routing
- `broker_config_topic` (string)
- `mqtt_ca_cert_path` (string) — path to CA certificate

#### 3.1.3 `mqtt_protocol_adapter_node` (`aehub_mqtt_protocol_adapter`)
**Type**: Lifecycle node  
**Responsibility**
- Translate MQTT payloads ↔ internal ROS command/event structures.
- Own the mapping between MQTT topics/payload schemas and ROS-side command routing.

**Key parameters**
- `robot_id` (string)

---

### 3.2 Business Layer

#### 3.2.1 `aehub_navigation_executor` (`aehub_navigation_executor`)
**Type**: Lifecycle node  
**Responsibility**
- Validate incoming navigation commands (including command identifiers).
- Execute navigation by calling the capability action.
- Emit `ack/state/result` events and maintain a coherent state machine for clients.
- Provide **idempotency** for repeated deliveries of the same command ID.

**Key parameters**
- `robot_id` (string)
- `capability_action_name` (string, default: `capabilities/navigation/execute`)

---

### 3.3 Capability Layer (Nav2 API + Readiness)

#### 3.3.1 Capability action (stable internal contract)
**Action name**: `capabilities/navigation/execute`  
**Type**: `aehub_msgs/action/NavigationExecute`

The executor depends on this action contract. The **implementation is replaceable**.

#### 3.3.2 Profile A implementation: `aehub_nav2_capability_server`
**Type**: Lifecycle node  
**Responsibility**
- Bridge goals to Nav2 (`/navigate_to_pose`) and map Nav2 outcome back to AEHub.
- Use ROS-based readiness gating (Nav2 lifecycle/action/TF evidence + robot interface evidence).

#### 3.3.3 Profile B implementation: `navigation_backend_server` (`aehub_navigation_backend`)
**Type**: Lifecycle node  
**Responsibility**
- Execute `navigateTo(target_id)` via Symovo REST:
  - create/start Transport with `GoToStationStep`
  - monitor transport state to produce `succeeded|canceled|error`
- Compute readiness from Symovo AMR `state_flags` (fail-closed on HTTP errors).
- Publish UI telemetry (`aehub_msgs/msg/NavigationStatus`) on `status/navigation`.

---

### 3.4 Navigation Stack (Nav2)

**Nav2 nodes (typical)**
- `map_server`, `amcl`
- `controller_server`, `planner_server`
- `bt_navigator`, `recoveries_server`
- `waypoint_follower`, `velocity_smoother` (as configured)
- lifecycle managers: `lifecycle_manager_localization`, `lifecycle_manager_navigation`

**Contract**
- Nav2 exposes the action server `/navigate_to_pose`.
- Nav2 publishes `/cmd_vel` during navigation; the robot base must consume it.

---

### 3.5 Robot Interface (Production)

#### 3.5.1 Base motion and odometry
There must be a production component responsible for:
- Publishing `/odom` (`nav_msgs/msg/Odometry`)
- Publishing TF for local motion (`odom -> base_*`)
- Consuming `/cmd_vel` (`geometry_msgs/msg/Twist`)

#### 3.5.2 Robot status / safety
There must be a production component responsible for publishing:
- `/robot/status` (`aehub_msgs/msg/RobotStatus`)
  - including **validity flags** and safety-relevant values (motors enabled, e-stop state, driver connected, etc.)

This is required for safety gating and readiness decisions.

---

### 4) ROS Interfaces (Topics, TF, Actions)

### 4.1 Actions

#### 4.1.1 Module capability action
- **Name**: `capabilities/navigation/execute`
- **Type**: `aehub_msgs/action/NavigationExecute`
- **Fields (conceptual)**
  - `command_id` (UUIDv4 string)
  - `target_id` (routing key; required in Profile B)
  - `x`, `y`, `theta` (pose fields; used in Profile A, ignored in Profile B)

#### 4.1.2 Nav2 action
- **Name**: `/navigate_to_pose`
- **Type**: Nav2’s standard NavigateToPose action

### 4.2 Topics

**Required for robot readiness**
- `/robot/status` (`aehub_msgs/msg/RobotStatus`) — must be fresh (non-stale)
- `/odom` (`nav_msgs/msg/Odometry`) — must be present and fresh
- `/cmd_vel` (`geometry_msgs/msg/Twist`) — must have consumers (the robot base)

**Required for Nav2 readiness**
- `/map` (if using map-based localization)
- `/amcl_pose` (if using AMCL localization)
- `/tf` and `/tf_static`

### 4.3 TF Frame Requirements
The following TF connectivity must exist for correct Nav2 operation and readiness checks:
- `map -> odom`
- `odom -> base_link` (or `odom -> base_footprint -> base_link`)

**Invariant**
- TF must form a single connected tree for frames used by navigation.
- Duplicate publishers for the same transform edge should be avoided.

---

### 5) MQTT Contract

### 5.1 Topic routing
MQTT topics are scoped by `robot_id`:
- Commands: `aroc/robot/<robot_id>/commands/#`
- Events: `aroc/robot/<robot_id>/events` (and optionally subtopics under `events/#` depending on deployment)

### 5.2 Command schema (navigateTo)
**Topic**: `aroc/robot/<robot_id>/commands/navigateTo`  
**Payload (minimum)**
- `command_id`: UUIDv4 string
- `target_id`: string (required in **machine-owned** profile)

**Optional fields (for pose-based backends / compatibility)**
- `x`: number (default 0.0 if omitted)
- `y`: number (default 0.0 if omitted)
- `theta`: number (default 0.0 if omitted)

### 5.3 UI telemetry topic (navigation status)
**Topic**: `aroc/robot/<robot_id>/status/navigation`  
**Payload schema**: `aehub.mqtt.status.navigation.v1`  
**Source**: `aehub_msgs/msg/NavigationStatus` published by backend server.

### 5.3 Event types and semantics
Events are emitted as a stream; consumers should not assume a single message.

**`ack`**
- `received`: command accepted for processing (syntactically valid)
- `accepted`: capability accepted for execution
- `rejected`: rejected with a reason (e.g., invalid command format)

**`state`**
- `public_state`: e.g. `ready`, `executing`, `done`
- `internal_state`: implementation-specific (e.g. `idle`, `navigating`)

**`result`**
- `success` OR `error` with `reason`

### 5.4 Error reason conventions (examples)
- `invalid_command_id_format`
- `busy: active_navigation_in_progress`
- `not_ready: <SUMMARY>`

The reason string is intended to be human- and machine-consumable; it should preserve key context for diagnosis.

---

### 6) Lifecycle and Startup Requirements

**Core invariants**
- MQTT transport must not attempt TLS without a valid CA path.
- Capability implementation must be lifecycle-managed reliably.
- Readiness must be evaluated from the **correct source of truth** for the selected profile:
  - Profile A: ROS introspection + robot interface signals
  - Profile B: machine HTTP API (AMR flags + transport state)

**Recommended order**
- Bring up robot interface signals (`/odom`, TF, `/robot/status`) early.
- Bring up Nav2 and allow it to reach `active`.
- Then accept navigation execution goals (capability server readiness will reflect this).

---

### 7) Security

**MQTT**
- TLS is supported and expected for production deployments.
- The CA certificate used to verify the broker must be configured (`mqtt_ca_cert_path`).

**Broker credentials**
- Broker credentials are fetched from the config service and must be protected in transit and at rest.
- API keys should be provided via environment or secured configuration files; avoid committing secrets.

---

### 8) Observability and Diagnostics

**Diagnostics topics**
- `/diagnostics` (Nav2 lifecycle managers and other diagnostic producers)
- `health/aehub_nav2_capability_server` (`diagnostic_msgs/msg/DiagnosticArray`)

**Operational checks**
- Lifecycle state via `*/get_state` services for lifecycle nodes (more robust than relying only on graph name discovery).
- Presence and freshness of `/robot/status` and `/odom`.
- TF connectivity checks for `map->odom` and `odom->base_link`.

---

### 9) Common Failure Modes (Production)

**Readiness failures**
- Robot:
  - missing/stale `/robot/status`
  - motors disabled / e-stop active (or unknown validity treated as unsafe)
  - missing/stale `/odom`
  - no consumers for `/cmd_vel`
- Nav2:
  - lifecycle nodes not active
  - missing TF connectivity (`map`/`odom`/`base_link`)
  - action `/navigate_to_pose` unavailable

**MQTT transport failures**
- TLS verification errors (missing/wrong CA cert)
- broker credentials missing/invalid

---

### 10) Reference Launch Entrypoints (Production)

The production stack is typically started by a launch that includes:
- MQTT credentials + transport + protocol adapter
- Navigation executor
- Capability server + lifecycle manager
- Robot interface nodes (base controller / status publishers)
- Nav2 bringup (map/localization/navigation)

Concrete entrypoints may vary by deployment, but the contracts above must hold.

