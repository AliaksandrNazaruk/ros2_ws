# aehub_robot_readiness

**RobotReadinessGate** - Determines if physical robot is ready for motion.

## Purpose

`RobotReadinessGate` is responsible **exclusively** for checking the physical robot's readiness to move.

- ❗ It does **NOT** know about Nav2
- ❗ It does **NOT** know about tasks/goals
- ❗ It does **NOT** control the robot
- ✔ It only **observes** and **evaluates** readiness

## Architecture Position

```
[Mqtt / App Layer]
        ↓
[Navigation Executor / FSM]
        ↓
[ Readiness Gates ]
   ├── Nav2ReadinessGate
   └── RobotReadinessGate   <-- THIS
        ↓
[ Nav2Adapter ]
        ↓
[ Nav2 ]
```

## Invariant

👉 `navigateToPose()` is allowed **ONLY IF**:
- `Nav2ReadinessGate == READY` **AND**
- `RobotReadinessGate == READY`

## Responsibilities

### FR-1: Robot motion readiness evaluation
Gate MUST determine whether robot is physically capable of motion.

### FR-2: Pure observer
Gate MUST NOT:
- Publish commands
- Change parameters
- Control hardware
- Block executors

### FR-3: Deterministic result
For the same system state → result MUST be identical.

## Checks

### Mandatory checks (blocking)

| Check | Description |
|-------|-------------|
| `ODOM_PRESENT` | `/odom` seen at least once |
| `ODOM_FRESH` | age < threshold (default: 1.0s) |
| `CMD_VEL_AVAILABLE` | someone subscribes to `/cmd_vel` |
| `MOTORS_ENABLED` | from robot status |
| `E_STOP` | must be inactive |

## Readiness Semantics

- **READY**: odom OK, cmd_vel connected, motors enabled, no estop
- **NOT_READY**: any hard failure
- **DEGRADED**: optional checks failed (battery low etc.)

## Usage

```cpp
#include "aehub_robot_readiness/robot_readiness_gate.hpp"

auto node = std::make_shared<rclcpp::Node>("my_node");
auto gate = std::make_unique<aehub::robot::RobotReadinessGate>(node);

// Check readiness (non-blocking)
auto report = gate->check();
if (report.isReady()) {
  // Robot is ready for motion
}

// Wait until ready (blocking, with timeout)
if (gate->waitUntilReady(std::chrono::seconds(10))) {
  // Robot became ready within timeout
}
```

## Configuration

The gate uses ROS2 parameters for safety state (motors, e-stop):

```yaml
robot:
  motors_enabled: true   # Motors enabled state
  estop_active: false    # Emergency stop active (true = unsafe)
```

In production, these would be read from robot status topics. The current implementation uses parameters for flexibility and testing.

## Test Matrix

| Test ID | Case | Expected |
|---------|------|----------|
| RG_01 | no odom → NOT_READY | ✅ |
| RG_02 | stale odom → NOT_READY | ✅ |
| RG_03 | no cmd_vel consumer → NOT_READY | ✅ |
| RG_04 | estop active → NOT_READY | ✅ |
| RG_05 | all OK → READY | ✅ |

## Building

```bash
cd /home/boris/ros2_ws
colcon build --packages-select aehub_robot_readiness
source install/setup.bash
```

## Running Tests

```bash
colcon test --packages-select aehub_robot_readiness
colcon test-result --verbose
```
