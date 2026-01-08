# Navigation Node Testing

This directory contains comprehensive tests for the navigation integrated node.

## Test Structure

```
test/
├── unit/                          # Unit tests (fast, isolated)
│   ├── test_navigation_action_client.py
│   ├── test_mqtt_command_event_publisher.py
│   ├── test_command_validator.py
│   ├── test_command_rate_limiter.py
│   ├── test_error_handler.py
│   ├── test_mqtt_connection_manager.py
│   ├── test_mqtt_status_publisher.py
│   ├── test_broker_config_provider.py
│   ├── test_navigation_state_manager.py
│   ├── test_position_registry.py
│   └── ...
├── integration/                   # Integration tests (require ROS2)
│   ├── test_fake_hub_integration.py
│   ├── test_navigation_integration.py
│   ├── test_critical_production_bugs.py
│   └── ...
└── README.md                       # This file
```

## Running Tests

### Quick Start

```bash
# Run all tests using the test runner script
python3 scripts/run_all_tests.py

# Run with coverage report
python3 scripts/run_all_tests.py --coverage

# Run only unit tests
python3 scripts/run_all_tests.py --unit-only

# Run only integration tests
python3 scripts/run_all_tests.py --integration-only

# Run only fake hub tests
python3 scripts/run_all_tests.py --fake-hub-only
```

### Using pytest directly

```bash
# Run all unit tests
pytest src/aehub_navigation/test/unit/ -v

# Run all integration tests
pytest src/aehub_navigation/test/integration/ -v

# Run with coverage
pytest src/aehub_navigation/test/ --cov=aehub_navigation --cov-report=html

# Run specific test file
pytest src/aehub_navigation/test/unit/test_navigation_action_client.py -v

# Run specific test
pytest src/aehub_navigation/test/unit/test_navigation_action_client.py::TestNavigationActionClient::test_send_goal_success -v
```

### Fake Hub End-to-End Tests

```bash
# Run fake hub tests directly
python3 scripts/test_chain_extended.py

# Run fake hub tests via pytest (requires ENABLE_FAKE_HUB_TESTS=1)
ENABLE_FAKE_HUB_TESTS=1 pytest src/aehub_navigation/test/integration/test_fake_hub_integration.py -v
```

## Prerequisites

### For Unit Tests
- Python 3.8+
- pytest
- pytest-cov (for coverage)
- rclpy (ROS2 Python package)

### For Integration Tests
- All unit test prerequisites
- ROS2 environment sourced
- Navigation node dependencies

### For Fake Hub Tests
- All integration test prerequisites
- Config Service running on http://localhost:7900
- MQTT broker accessible
- Navigation node running (optional, for full end-to-end)

## Test Coverage

To generate coverage report:

```bash
# Using test runner
python3 scripts/run_all_tests.py --coverage

# Using pytest directly
pytest src/aehub_navigation/test/ --cov=aehub_navigation --cov-report=html --cov-report=term-missing

# View HTML report
open htmlcov/index.html  # macOS
xdg-open htmlcov/index.html  # Linux
```

## Test Categories

### Unit Tests
- **Fast**: Run in < 1 second each
- **Isolated**: No external dependencies
- **Mocked**: Use mocks for external services
- **Coverage**: Test individual modules in isolation

### Integration Tests
- **Realistic**: Test with ROS2 context
- **Focused**: Test specific integration scenarios
- **Critical**: Catch production bugs
- **Edge Cases**: Test error conditions and race conditions

### Fake Hub Tests
- **End-to-End**: Test complete system from MQTT to Nav2
- **Real Services**: Use real MQTT broker and Config Service
- **Specification**: Verify compliance with SPECIFICATION.md
- **Comprehensive**: Test multiple scenarios in sequence

## Writing New Tests

### Unit Test Template

```python
import pytest
from unittest.mock import Mock, MagicMock
from aehub_navigation.your_module import YourClass

class TestYourClass:
    @pytest.fixture
    def instance(self):
        return YourClass()
    
    def test_feature(self, instance):
        # Arrange
        # Act
        result = instance.method()
        # Assert
        assert result == expected
```

### Integration Test Template

```python
import pytest
import rclpy
from aehub_navigation.navigation_integrated_node import NavigationIntegratedNode

@pytest.fixture
def rclpy_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

def test_integration_scenario(rclpy_context):
    # Test integration scenario
    pass
```

## Test Maintenance

### When to Add Tests
1. New feature added
2. Bug found and fixed
3. Edge case discovered
4. Production incident occurred

### When to Update Tests
1. API changes
2. Specification changes
3. Test becomes flaky
4. Coverage gap identified

### Test Quality Checklist
- [ ] Test is deterministic (no flakiness)
- [ ] Test is fast (< 1 second for unit, < 10 seconds for integration)
- [ ] Test is isolated (no side effects)
- [ ] Test has clear assertions
- [ ] Test name describes what it tests
- [ ] Test follows AAA pattern (Arrange, Act, Assert)

## Known Issues

### Flaky Tests
- Some integration tests may be timing-dependent
- Fake hub tests require external services

### Coverage Gaps
- Some error paths may not be fully covered
- Edge cases in reconnection logic

## CI/CD Integration

Tests can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions
- name: Run Unit Tests
  run: pytest src/aehub_navigation/test/unit/ -v

- name: Run Integration Tests
  run: pytest src/aehub_navigation/test/integration/ -v

- name: Generate Coverage
  run: pytest src/aehub_navigation/test/ --cov=aehub_navigation --cov-report=xml
```

## Troubleshooting

### Tests fail with "rclpy not initialized"
- Ensure ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`

### Fake hub tests fail with "Config Service not available"
- Start Config Service: `docker-compose up config-service`
- Or skip fake hub tests: `python3 scripts/run_all_tests.py --skip-fake-hub`

### Tests timeout
- Increase timeout in test: `@pytest.mark.timeout(60)`
- Check for deadlocks or infinite loops

### Coverage report not generated
- Install pytest-cov: `pip install pytest-cov`
- Use `--coverage` flag with test runner

## References

- [SPECIFICATION.md](../../../../SPECIFICATION.md) - System specification
- [pytest documentation](https://docs.pytest.org/)
- [ROS2 testing guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-ROS2.html)

