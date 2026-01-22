# SRS: Global Requirements for AE.HUB ROS 2 Stack
Version: 0.1  
Status: **Normative document (MUST/SHALL)**  
Purpose: ensure agents/developers write code "like top-tier ROS 2 robotics engineers" and **do not violate responsibility boundaries**, ensuring **easy module replaceability**.

---

## 1. Terms and fundamental definitions

- **ROS 2**: middleware (transport/synchronization/tooling), **not an architecture**.
- **Architecture**: a layered responsibility model, contracts, and invariants for resource ownership.
- **Application Layer / Orchestrator**: scenario business logic, FSM, idempotency.
- **Capability Layer (Service)**: a robot capability exposed as a service with **a single API**.
- **ROS Interface Layer**: thin adapters/drivers; minimal logic.
- **Hardware Implementation**: "hardware" implementation (real/sim) as a swappable part **within a single graph**.
- **Ready/Health**: node readiness/health; **ACTIVE == ready for operation**.

---

## 2. Goals

- **G1**: enforce strict responsibility boundaries and prevent "Swiss Army knife" nodes.
- **G2**: enable replaceability of Nav2/simulation/hardware **without rewriting Application logic**.
- **G3**: ensure production readiness: Lifecycle + Health + Watchdog + graceful degradation.
- **G4**: enable multi-robot scaling: everything under `/robot/<id>/...`.

## 3. Non-Goals

- **NG1**: "subscribe to Nav2 topics and implement logic based on messages" — forbidden.
- **NG2**: "a separate simulator system with a different graph" — forbidden.
- **NG3**: "one monolithic ROS node that does everything" — forbidden.

---

## 4. Reference layer model (mandatory)

### 4.0. Directionality contract: "layers do not control adjacent layers"

- **INV-DIR-01 (MUST)**: no layer **controls** an adjacent layer (no "layer manager").
- **INV-DIR-02 (MUST)**: interaction direction is strictly:
  - **observe upwards** (events/telemetry/state)
  - **command downwards** (intents/requests/goals)
- **INV-DIR-03 (MUST)**: readiness = **gate/validator**, not a controller:
  - readiness performs a **snapshot check** and returns a result (READY/NOT_READY + summary)
  - readiness does not publish commands, does not change other nodes' lifecycle, does not "fix" the system
- **INV-DIR-04 (MUST)**: "Nav2 = black box" means:
  - Nav2 is treated as an **external system**, not a "dependency you can rely on via topics"
  - business decisions are not made based on Nav2 topics

### 4.1. Layers

1) **Application Layer (Orchestrator / FSM)**
- Contains: FSM, idempotency, policies (rate-limit, dedup), scenarios.
- Does not contain: Nav2 topics, `/cmd_vel`, hardware topics.

2) **Capability Layer (Services)**
- Contains: a single capability API (Actions/Services), readiness/STOP policies, internal timeouts.
- May use Nav2/other dependencies internally, but exposes **only its contract** outward.

3) **ROS Interface Layer**
- Contains: transport/edge-protocol adapters, drivers.
- Allowed: minimal format transformation, schema validation at the boundary.
- Forbidden: FSM, dedup, business policies.

4) **Hardware / Simulation**
- Implementations of the same interface (real vs sim), selected via launch arguments.

### 4.2. "Thin ROS layer" invariant
- **INV-THIN-01 (MUST)**: each ROS node must be **either** an adapter/driver, **or** a capability service, **or** an orchestrator. Mixing is forbidden.

### 4.3. Navigation (domain 6-layer model)

This model is mandatory for the navigation domain and clarifies responsibility boundaries.

#### L1 — Command / Intent Layer (external world)
- **Responsibility**: *what we want to do*, not "how".
- **Components**: MQTT/HTTP/REST/UI/Mission Planner.
- **Commands**: `navigate_to(pose)`, `cancel`, `pause`, `resume`.
- **Forbidden (MUST NOT)**: check TF/odom, know about Nav2 lifecycle, depend on Nav2 topics.

#### L2 — Nav2Adapter (Application / Orchestration Layer)
- **Responsibility**: a single control point for navigation; the FSM is the single source of truth.
- **Contains**: navigation FSM, contract with L1, event publication, idempotency/dedup policy.
- **Dependencies (strictly downward)**:
  - readiness gates (L3) **only via the final READY/NOT_READY result**
  - action client to capability/Nav2 (via the contract)
- **Invariants**:
  - **INV-NAVAD-01 (MUST)**: never send a goal if any gate != READY
  - **INV-NAVAD-02 (MUST)**: never wait for readiness inside lifecycle callbacks (only snapshot checks in the operational loop)
  - **INV-NAVAD-03 (MUST)**: the FSM publishes a single event stream upward; consumers do not read "internal details"

#### L3 — Readiness Gates (Domain Validation Layer)
- **Responsibility**: deterministic readiness validation (snapshot checks), without control.
- **Nav2ReadinessGate** checks (approximately): Nav2 node lifecycles, presence of action `/navigate_to_pose`, TF `map→odom→base_link`, required localization/map sources.
  - **Forbidden**: to know about robot hardware, to own `/cmd_vel`.
- **RobotReadinessGate** checks physical readiness (approximately): odom, presence of subscribers on the cmd_vel interface, motors enabled, e-stop.
  - **Forbidden**: to know about Nav2, Nav2 lifecycle.
- **Composition**:
  - **INV-GATE-01 (MUST)**: `ready = nav2_gate && robot_gate`
  - **INV-GATE-02 (MUST)**: Nav2Adapter sees **only the final result** (READY/NOT_READY + brief summary); details are available separately as telemetry/debug.

#### L4 — Nav2 Stack (External System)
- **Responsibility**: external navigation system (not our code).
- **Contract**: Actions + Lifecycle + TF + required topics.
- **Forbidden**: adding adapter logic or readiness logic there.

#### L5 — Motion Execution Layer (Robot Control)
- **Components**: base_controller, cmd_vel_mux, motor drivers, safety controller.
- **Contract**: accepts `/cmd_vel`, publishes `/odom`, ensures safety stop.
- **Invariant**: RobotReadinessGate is an observer, not a controlling controller.

#### L6 — Hardware / Drivers
- **Components**: motor drivers, encoders, safety relays, e-stop.
- **Forbidden**: ROS logic and Nav2 logic.

#### Trace (normative flow)

`[Mission/Intent] → [Nav2Adapter FSM] → (if READY) [Action Client] → [Nav2 Stack] → [/cmd_vel] → [Base Controller] → [Robot]`

---

## 5. Nav2: integration rules (strict)

- **INV-NAV2-01 (MUST)**: Nav2 is used **only** via Actions + Lifecycle.
- **INV-NAV2-02 (MUST NOT)**: no business logic on Nav2 topics (`/odom`, `/amcl_pose`, `/cmd_vel`, etc.).
- **INV-NAV2-03 (MUST)**: outwardly, Nav2 is treated as a **black box** behind a capability API.

---

## 6. Contracts and "topic hell": hard rules

### 6.1. Single-input-topic rule
- **INV-IO-01 (MUST)**: a component (node) must have **no more than 1 input topic** for its domain.
- **INV-IO-02 (SHOULD)**: one output event stream per domain (a single event stream).

### 6.2. No "raw" topics upwards
- **INV-RAW-01 (MUST NOT)**: do not expose "raw" transport/internal topics as a capability interface.
- **INV-RAW-02 (MUST)**: only aggregated/strictly-typed statuses/events go upward.

---

## 7. Namespace and naming (mandatory)

- **INV-NS-01 (MUST)**: everything must live under the namespace: `/robot/<robot_id>/...`.
- **INV-NS-02 (MUST)**: use **relative** topic names in code (no leading `/`).
- **INV-NS-03 (MUST NOT)**: no global `/odom`, `/scan`, `/cmd_vel` in upper-layer interfaces (drivers/local implementations may have their own names, but bringup must be namespaced).

---

## 8. Lifecycle / Health / Watchdog (production minimum)

### 8.1. Lifecycle is mandatory
- **INV-LC-01 (MUST)**: all key nodes must be LifecycleNodes.
- **INV-LC-02 (MUST)**: ACTIVE means "ready to serve requests".
- **INV-LC-03 (MUST)**: a node must correctly release resources in deactivate/cleanup.

### 8.2. Health
- **INV-HLTH-01 (MUST)**: each key node publishes health (`diagnostic_msgs/DiagnosticArray` or equivalent).
- **INV-HLTH-02 (MUST)**: health reflects readiness / dependency errors / lifecycle status.

### 8.3. Watchdog + degradation
- **INV-WD-01 (MUST)**: there must be timeouts for external dependencies (MQTT/HTTP/Nav2).
- **INV-WD-02 (MUST)**: degradation must be explicit (not "silence"): events/health must show the problem.

---

## 9. Sim/Real: one launch, one graph

- **INV-SIM-01 (MUST)**: Simulation = hardware implementation, not a separate system.
- **INV-SIM-02 (MUST)**: **one bringup**, **one graph**, differences only via parameters:
  - `use_sim_time:=true|false`
  - `hardware:=sim|real`
- **INV-SIM-03 (MUST NOT)**: different graphs for sim/real are forbidden.

---

## 10. Navigation as a service (NavigationService API)

### 10.1. Capability model
The Navigation capability exposes **one API** outward:
- Action: `capabilities/navigation/execute` (`aehub_msgs/action/NavigationExecute`)
- (optional) Service: `capabilities/navigation/get_state`
- (optional) Service: `capabilities/navigation/reset`

### 10.2. Ownership of critical resources
- **INV-OWN-01 (MUST)**: `/cmd_vel` is published **only** by the Navigation Capability (within the namespace).
- **INV-OWN-02 (MUST)**: STOP semantics (burst/safety policy) belong **only** to the Capability.
- **INV-OWN-03 (MUST)**: readiness gates are **observers** and are checked *on demand* (gate snapshot), but do not control the system.
- **INV-OWN-04 (MUST)**: the orchestrator never publishes `/cmd_vel` and does not "help" Nav2.

---

## 11. Transport / Protocol boundary (MQTT)

### 11.1. Transport node (pure transport)
- **INV-TR-01 (MUST)**: transport does not parse JSON and does not know commands/events.
- **INV-TR-02 (MUST)**: transport bridges only the typed envelope:
  - `infra/mqtt/in` (`aehub_msgs/MqttEnvelope`)
  - `infra/mqtt/out` (`aehub_msgs/MqttEnvelope`)

### 11.2. Protocol edge adapter (the only place for JSON)
- **INV-PA-01 (MUST)**: JSON is allowed only in the protocol adapter.
- **INV-PA-02 (MUST)**: the protocol adapter has:
  - one input: `infra/mqtt/in` (`MqttEnvelope`)
  - one output: `infra/mqtt/out` (`MqttEnvelope`)
  - one subscription to domain events: `events/navigation` (`NavigationEvent`)
- **INV-PA-03 (MUST)**: the protocol adapter does not do FSM/dedup/policies — only schema handling and routing.

---

## 12. Normative interfaces (target names)

All names below are assumed to be **relative** and will live under `/robot/<id>/...` via launch.

- `infra/mqtt/broker_config` (`aehub_msgs/BrokerConfig`)
- `infra/mqtt/in` (`aehub_msgs/MqttEnvelope`)
- `infra/mqtt/out` (`aehub_msgs/MqttEnvelope`)
- `commands/navigation` (`aehub_msgs/NavigationCommand`)
- `events/navigation` (`aehub_msgs/NavigationEvent`)
- `capabilities/navigation/execute` (`aehub_msgs/action/NavigationExecute`)
- `health/<node>` (`diagnostic_msgs/DiagnosticArray` or equivalent)

---

## 13. Anti-patterns (forbidden)

- Subscribing to Nav2 topics and making decisions "from messages".
- The orchestrator publishing `/cmd_vel` or performing STOP.
- A node with 5–10 input topics "because it's convenient".
- Different launch / different graphs for sim and real.
- Global topic names without `/robot/<id>/...`.

---

## 14. "Project is healthy" checklist (Definition of Done)

- You can remove Nav2 and replace it with a mock capability without changing the orchestrator.
- The FSM (Application Layer) starts and is testable without Nav2 ROS topics.
- Sim/Real: one bringup, one graph.
- No business logic on topics.
- The owner of each critical topic is explicitly defined (especially `/cmd_vel`).
- Each key node has: Lifecycle + Health + Watchdog.
