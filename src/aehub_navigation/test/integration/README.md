# Critical Production Bug Tests

## Test Strategy

These tests are designed by a Senior SDET to catch **real production bugs** that would cause incidents in a deployed system. They follow strict principles:

### Principles

1. **No placeholder tests** - Every test must catch a realistic bug
2. **Behavioral verification** - Tests verify behavior, not implementation
3. **Failure value** - Each test must be able to fail on realistic bugs
4. **Production incidents** - Tests are written as if incidents already happened

### Test Categories

#### 1. MQTT Reconnection Race Conditions (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- MQTT broker restarts
- Network interruptions
- Config Service updates broker configuration

**Bugs caught:**
- Commands lost during reconnection window
- Multiple commands lost during concurrent reconnection
- `_mqtt_ready` flag not reset after successful reconnection

**Tests:**
- `test_command_lost_during_reconnection_window`
- `test_concurrent_commands_during_reconnection`
- `test_reconnection_success_after_command_rejection`

#### 2. Nav2 Server Unavailability (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- Nav2 lifecycle nodes fail
- Nav2 crashes during navigation
- Network partition between nodes

**Bugs caught:**
- Goal sent when server becomes unavailable
- Goal handle memory leak after server crash
- State corruption when server unavailable

**Tests:**
- `test_goal_sent_when_server_becomes_unavailable`
- `test_server_unavailable_after_goal_accepted`

#### 3. Concurrent Command Handling (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- Multiple users send commands simultaneously
- Rapid command sequences from automation
- Race conditions in rate limiting

**Bugs caught:**
- Concurrent commands with different IDs both processed
- Rate limiting bypassed by concurrent commands
- State corruption from concurrent operations

**Tests:**
- `test_concurrent_commands_with_different_ids`
- `test_rate_limiting_bypass_with_concurrent_commands`

#### 4. Status Publishing Failures (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- MQTT publish fails
- Network issues during status update
- MQTT broker rejects message

**Bugs caught:**
- Status publish failure during command processing
- Status publish failure during rejection
- User never notified of command status

**Tests:**
- `test_status_publish_failure_during_command_processing`
- `test_status_publish_failure_during_rejection`

#### 5. Position Registry Failures (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- Position registry temporarily unavailable
- Invalid position IDs
- Registry throws exceptions

**Bugs caught:**
- Position registry returns None, no retry
- Registry exception crashes node
- Invalid position not properly handled

**Tests:**
- `test_position_registry_returns_none`
- `test_position_registry_exception`

#### 6. State Consistency After Failures (`test_critical_production_bugs.py`)

**Real-world scenarios:**
- Goal rejected but state not reset
- Cancel called but state not cleared
- Partial failures leave inconsistent state

**Bugs caught:**
- State corruption after goal rejection
- State not reset after cancel
- Inconsistent state blocks future commands

**Tests:**
- `test_state_consistency_after_goal_rejection`
- `test_state_consistency_after_cancel`

#### 7. MQTT Connection Edge Cases (`test_mqtt_connection_edge_cases.py`)

**Real-world scenarios:**
- Connection timeout with delayed callback
- Multiple simultaneous connection attempts
- Resource leaks from uncleaned connections

**Bugs caught:**
- Connection timeout race condition
- Callback called after timeout check
- Memory leaks from uncleaned clients
- CA certificate temp files not cleaned

**Tests:**
- `test_connection_timeout_before_callback`
- `test_callback_called_after_timeout_check`
- `test_multiple_simultaneous_connection_attempts`
- `test_old_client_not_cleaned_on_reconnect`
- `test_ca_certificate_cleanup_on_reconnect`

#### 8. Subscription Management (`test_mqtt_connection_edge_cases.py`)

**Real-world scenarios:**
- Subscriptions lost on reconnection
- Subscription failures not logged
- Commands lost after reconnection

**Bugs caught:**
- Subscriptions not restored after reconnection
- Subscription failures not logged
- Commands lost silently

**Tests:**
- `test_subscriptions_lost_on_reconnection`
- `test_subscription_failure_during_reconnection`

## Running Tests

```bash
# Run all critical bug tests
pytest src/aehub_navigation/test/integration/test_critical_production_bugs.py -v

# Run MQTT edge case tests
pytest src/aehub_navigation/test/integration/test_mqtt_connection_edge_cases.py -v

# Run with coverage
pytest src/aehub_navigation/test/integration/ --cov=aehub_navigation --cov-report=html
```

## Test Maintenance

### When to Add Tests

Add a test when:
1. A production incident occurs
2. A bug is found during code review
3. A new failure mode is identified
4. A race condition is discovered

### When to Remove Tests

Remove a test when:
1. The bug it catches is impossible (architecture change)
2. The test becomes flaky (not deterministic)
3. The test no longer provides value (bug already fixed and prevented)

### Test Quality Checklist

Before committing a test, verify:
- [ ] Test can realistically fail on a bug
- [ ] Test verifies behavior, not implementation
- [ ] Test has clear failure value
- [ ] Test is deterministic (no flakiness)
- [ ] Test is fast (< 1 second)
- [ ] Test is isolated (no external dependencies)

## Known Issues

### Current Limitations

1. **Mock limitations**: Some tests use mocks that may not perfectly simulate real behavior
2. **Timing sensitivity**: Some race condition tests may be timing-dependent
3. **ROS2 context**: Tests require ROS2 context initialization

### Future Improvements

1. Integration tests with real MQTT broker
2. Integration tests with real Nav2 server
3. Stress tests for high concurrency
4. Long-running tests for memory leaks
5. Network partition simulation

## Contributing

When adding new tests:
1. Follow the Given/When/Then structure
2. Document the bug the test would catch
3. Explain the production incident scenario
4. Ensure the test can realistically fail
5. Verify the test is deterministic

