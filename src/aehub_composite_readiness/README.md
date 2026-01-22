# aehub_composite_readiness

**CompositeReadinessGate** - Aggregates multiple `ReadinessGate` instances into a single readiness decision.

## Purpose

`CompositeReadinessGate` is a **pure orchestrator/aggregator** that:

- ❗ Does **NOT** manage gates
- ❗ Does **NOT** publish
- ❗ Does **NOT** block executors
- ❗ Does **NOT** know details about Nav2/robot
- ✔ Only **aggregates** results from multiple gates

## Supported Gates

| Gate | Required |
|------|----------|
| `Nav2ReadinessGate` | ✅ |
| `RobotReadinessGate` | ✅ |
| `MissionReadinessGate` | ⭕ (optional) |

## Aggregation Rules

### Level Aggregation

```
if ANY gate == NOT_READY  → NOT_READY
else if ANY gate == DEGRADED → DEGRADED
else                      → READY
```

### Failures

- `failures = union` of all failures from sub-gates
- Source of each failure is preserved

### Capabilities

- Capability is `READY` **only if ALL gates** confirm it (AND logic)

## Usage

```cpp
#include "aehub_composite_readiness/composite_readiness_gate.hpp"
#include "aehub_nav2_readiness/nav2_readiness_gate.hpp"
#include "aehub_robot_readiness/robot_readiness_gate.hpp"

auto node = std::make_shared<rclcpp::Node>("my_node");

// Create individual gates
auto nav2_gate = std::make_shared<aehub::nav2::Nav2ReadinessGate>(node);
auto robot_gate = std::make_shared<aehub::robot::RobotReadinessGate>(node);

// Create composite gate
auto composite = std::make_shared<aehub::readiness::CompositeReadinessGate>();

// Add gates
composite->addGate({"nav2", nav2_gate, true});
composite->addGate({"robot", robot_gate, true});

// Use composite gate
auto report = composite->check();
if (report.isReady()) {
  // System is ready for navigation
}
```

## Integration with Nav2Adapter

```cpp
nav2_adapter->setReadinessGate(composite);
```

## Invariants

- **INV-CRG-01**: Contains no check logic, only aggregation
- **INV-CRG-02**: `check()` does NOT call blocking APIs except `check()`
- **INV-CRG-03**: Repeated `check()` with unchanged inputs → identical result
- **INV-CRG-04**: `changed()` does NOT modify state

## Thread Safety

- Thread-safe (mutex-protected)
- `check()` is the only mutating operation
- `changed()` is pure comparison
- `waitUntilReady()` uses polling (NOT for lifecycle callbacks)

## Building

```bash
cd /home/boris/ros2_ws
colcon build --packages-select aehub_composite_readiness
source install/setup.bash
```

## Running Tests

```bash
colcon test --packages-select aehub_composite_readiness
colcon test-result --verbose
```
