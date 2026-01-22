# Nav2Adapter Test Matrix

Complete test coverage for Nav2Adapter following SRS requirements.

## Test Organization

Tests are organized by category:
- **A. Lifecycle** - Lifecycle state transitions
- **B. Navigation Flow** - Goal execution outcomes
- **C. Cancel Semantics** - Cancel operation behavior
- **D. Fault Injection** - Error handling and recovery
- **E. Concurrency** - Thread safety and race conditions
- **F. Performance** - Latency requirements

## Test Implementation

### Test Suite Location
- Unit/Integration tests: `test/test_nav2_adapter_matrix.cpp` (Google Test)
- Test helpers: `test/test_helpers.hpp/cpp` (FakeNav2ActionServer, EventCollector)
- Smoke test: `test/smoke_test_nav2_adapter.cpp` (standalone executable)

### Running Tests

```bash
# Build tests
cd ~/ros2_ws
colcon build --packages-select aehub_nav2_adapter
source install/setup.bash

# Run all tests
colcon test --packages-select aehub_nav2_adapter
colcon test-result --verbose

# Run specific test suite
ros2 run aehub_nav2_adapter test_nav2_adapter_matrix

# Run smoke test (requires Nav2 stack)
ros2 run aehub_nav2_adapter smoke_test_nav2_adapter
```

## Test Matrix

### A. Lifecycle

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-LC-01 | TC-LC-01 | Integration | Configure → Activate | ACTIVE | ✅ Implemented |
| NAV-LC-02 | TC-LC-02 | Integration | Activate without Nav2 | FAIL | ✅ Implemented |
| NAV-LC-03 | TC-LC-03 | Integration | Deactivate during goal | Goal canceled | ✅ Implemented |
| NAV-LC-04 | TC-LC-04 | Integration | Cleanup clears state | No active goal | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_LC_01_*, TC_LC_02_*, TC_LC_03_*, TC_LC_04_*)

### B. Navigation Flow

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-GO-01 | TC-GO-01 | Integration | Send valid goal | onAccepted | ✅ Implemented |
| NAV-GO-02 | TC-GO-02 | Integration | Goal success | onSucceeded | ✅ Implemented |
| NAV-GO-03 | TC-GO-03 | Integration | Goal aborted | onFailed | ✅ Implemented |
| NAV-GO-04 | TC-GO-04 | Integration | Goal canceled by Nav2 | onCanceled | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_GO_01_*, TC_GO_02_*, TC_GO_03_*, TC_GO_04_*)

### C. Cancel Semantics

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-CAN-01 | TC-CAN-01 | Unit | Cancel no goal | OK | ✅ Implemented |
| NAV-CAN-02 | TC-CAN-02 | Integration | Cancel active goal | onCanceled | ✅ Implemented |
| NAV-CAN-03 | TC-CAN-03 | Integration | Double cancel | No duplicate events | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_CAN_01_*, TC_CAN_02_*, TC_CAN_03_*)

### D. Fault Injection

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-FT-01 | TC-FT-01 | Integration | Server not available | onFailed | ✅ Implemented |
| NAV-FT-02 | TC-FT-02 | Integration | Nav2 crash mid-goal | onFailed | ✅ Implemented |
| NAV-FT-03 | TC-FT-03 | Integration | Restart Nav2 | Safe failure | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_FT_01_*, TC_FT_02_*, TC_FT_03_*)

**Note**: TC-FT-02 and TC-FT-03 test restart detection via `checkServerHealth()`. Full restart detection requires Nav2 lifecycle manager integration (tested in smoke test).

### E. Concurrency

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-CON-01 | TC-CON-01 | Stress | Rapid goal+cancel | No deadlock | ✅ Implemented |
| NAV-CON-02 | TC-CON-02 | Stress | Shutdown during callback | Safe exit | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_CON_01_*, TC_CON_02_*)

**Note**: TC-CON-01 performs 10 rapid goal+cancel cycles to verify thread safety and absence of deadlocks.

### F. Performance

| SRS-ID | Test-ID | Type | Description | Expected | Status |
|--------|---------|------|-------------|----------|--------|
| NAV-PERF-01 | TC-PERF-01 | Benchmark | Goal send latency | < 100ms | ✅ Implemented |
| NAV-PERF-02 | TC-PERF-02 | Benchmark | Cancel latency | < 50ms | ✅ Implemented |

**Test File**: `test/test_nav2_adapter_matrix.cpp` (TC_PERF_01_*, TC_PERF_02_*)

**Note**: Performance thresholds are targets, not hard limits. Actual performance depends on system load.

## Test Helpers

### FakeNav2ActionServer

C++ fake Nav2 action server for testing:
- Control goal acceptance/rejection
- Control goal success/abort
- Simulate goal delay
- Cancel goals programmatically

**Usage**:
```cpp
auto fake_server = std::make_shared<FakeNav2ActionServer>();
fake_server->setShouldAcceptGoals(true);
fake_server->setShouldSucceedGoals(true);
fake_server->setGoalDelaySec(0.2);
```

### EventCollector

Thread-safe event collector for testing callbacks:
- Collect all adapter events (accepted, succeeded, failed, canceled)
- Wait for specific events with timeout
- Check event presence

**Usage**:
```cpp
auto collector = std::make_shared<EventCollector>();
// Set callbacks to collector methods
// Wait for event
EXPECT_TRUE(collector->waitForEvent("accepted", command_id, 2000ms));
```

## Test Coverage

### Coverage by Category

- **Lifecycle**: 4/4 tests (100%)
- **Navigation Flow**: 4/4 tests (100%)
- **Cancel Semantics**: 3/3 tests (100%)
- **Fault Injection**: 3/3 tests (100%)
- **Concurrency**: 2/2 tests (100%)
- **Performance**: 2/2 tests (100%)

**Total**: 18/18 tests (100%)

### Coverage by Type

- **Unit tests**: 1 test (TC-CAN-01)
- **Integration tests**: 15 tests
- **Stress tests**: 2 tests (TC-CON-01, TC-CON-02)
- **Benchmark tests**: 2 tests (TC-PERF-01, TC-PERF-02)

## Test Execution

### Prerequisites

- ROS2 Jazzy/Humble installed
- Google Test (gtest) - provided by ament_cmake_gtest
- FakeNav2ActionServer (included in test helpers)

### Running Individual Tests

```bash
# Run specific test
ros2 run aehub_nav2_adapter test_nav2_adapter_matrix --gtest_filter=TC_LC_01_ConfigureActivate

# Run all lifecycle tests
ros2 run aehub_nav2_adapter test_nav2_adapter_matrix --gtest_filter=TC_LC_*

# Run all cancel tests
ros2 run aehub_nav2_adapter test_nav2_adapter_matrix --gtest_filter=TC_CAN_*
```

### Expected Results

All tests should pass with:
- ✅ Lifecycle transitions work correctly
- ✅ Events are emitted for all scenarios
- ✅ Cancel is idempotent
- ✅ No deadlocks or race conditions
- ✅ Performance targets met
- ✅ Safe shutdown without crashes

## Known Limitations

1. **TC-FT-02/TC-FT-03**: Full Nav2 restart detection requires lifecycle manager integration. Tests verify health check logic, but full restart scenario requires external Nav2 stack.

2. **TC-PERF-01/TC-PERF-02**: Performance tests measure best-case latency. Real-world performance may vary based on system load.

3. **Stress tests**: TC-CON-01 performs 10 iterations. For production validation, consider increasing to 100+ iterations.

## Future Enhancements

- [ ] Add more stress test iterations (100+ cycles)
- [ ] Add memory leak detection
- [ ] Add valgrind/asan integration
- [ ] Add coverage reporting
- [ ] Add CI/CD integration
- [ ] Add performance regression tracking
