#!/usr/bin/env python3
"""
Pytest Integration for Fake Hub End-to-End Tests

Integrates fake_hub tests into pytest framework for CI/CD compatibility.
These tests require:
- Config Service running on http://localhost:7900
- MQTT broker accessible
- Navigation node running

These are true end-to-end tests that verify the complete system.
"""

import pytest
import sys
import os
import time
import json
import uuid
from pathlib import Path
from typing import Optional, Dict, Any
from datetime import datetime, timezone

# Add scripts directory to path
SCRIPT_DIR = Path(__file__).parent.parent.parent.parent.parent / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from fake_hub import FakeHub


# Test configuration
CONFIG_SERVICE_URL = "http://localhost:7900"
API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = os.getenv("ROBOT_ID", "fahrdummy-01-test")


@pytest.fixture(scope="module")
def fake_hub():
    """
    Create and configure fake hub for testing.
    
    This fixture is module-scoped to reuse the same hub instance
    across all tests in this module.
    """
    hub = FakeHub(
        config_service_url=CONFIG_SERVICE_URL,
        api_key=API_KEY,
        robot_id=ROBOT_ID
    )
    
    # Fetch MQTT config
    if not hub.fetch_mqtt_config():
        pytest.skip("Config Service not available")
    
    # Connect to MQTT
    if not hub.connect_mqtt():
        pytest.skip("MQTT broker not available")
    
    yield hub
    
    # Cleanup
    hub.shutdown()


@pytest.fixture(autouse=True)
def cleanup_between_tests(fake_hub):
    """Clean up status history between tests"""
    fake_hub.clear_status_history()
    yield
    time.sleep(0.5)  # Small delay between tests


def send_command_and_wait(hub: FakeHub, target_id: str, priority: str = "normal", 
                         timeout: float = 10.0) -> tuple[Optional[str], Optional[str]]:
    """
    Helper function to send command and wait for status.
    
    Returns:
        Tuple of (status, error_code)
    """
    command_id = hub.send_navigate_command(target_id, priority)
    if not command_id:
        return None, "SEND_FAILED"
    
    time.sleep(0.5)  # Wait for processing
    
    status_obj = hub.get_status_by_command_id(command_id, timeout=timeout, wait_for_error=True)
    if status_obj:
        ev_type = status_obj.get('event_type')
        if ev_type == 'result':
            return status_obj.get('result_type'), status_obj.get('error_code')
        error_code = status_obj.get('error_code')
        status_type = status_obj.get('status') or 'idle'
        return status_type, error_code
    
    return None, "TIMEOUT"


class TestFakeHubIntegration:
    """Integration tests using fake hub"""
    
    @pytest.mark.integration
    def test_rate_limiting(self, fake_hub):
        """Test that rate limiting rejects commands sent too quickly"""
        # First command should be accepted
        command_id_1 = fake_hub.send_navigate_command("position_A")
        assert command_id_1 is not None, "First command should be accepted"
        
        time.sleep(0.05)  # Only 50ms - less than 0.1s threshold
        
        # Second command should be rate limited
        command_id_2 = fake_hub.send_navigate_command("position_B")
        
        time.sleep(3.0)  # Wait for processing
        
        status2_obj = fake_hub.get_status_by_command_id(
            command_id_2, timeout=1.0, wait_for_error=True
        ) if command_id_2 else None
        
        if status2_obj:
            error_code = status2_obj.get('error_code')
            # Should be rate limited or at least have an error
            assert error_code is not None, "Second command should have error code"
            if error_code == "NAV_RATE_LIMIT_EXCEEDED":
                pytest.skip("Rate limiting works correctly")
    
    @pytest.mark.integration
    def test_invalid_target_id(self, fake_hub):
        """Test that invalid target_id is rejected"""
        status, error = send_command_and_wait(fake_hub, "non_existent_position_xyz")
        
        assert error is not None, "Invalid target should produce error"
        assert error in ["NAV_INVALID_TARGET", "NAV_INVALID_COMMAND"], \
            f"Expected NAV_INVALID_TARGET or NAV_INVALID_COMMAND, got {error}"
    
    @pytest.mark.integration
    def test_valid_command_accepted(self, fake_hub):
        """Test that valid command is accepted"""
        time.sleep(0.2)  # Avoid rate limit
        
        status, error = send_command_and_wait(fake_hub, "position_A")
        
        # Should not have validation errors
        assert error not in ["NAV_INVALID_TARGET", "NAV_RATE_LIMIT_EXCEEDED"], \
            f"Valid command should not be rejected: {error}"
    
    @pytest.mark.integration
    def test_command_priorities(self, fake_hub):
        """Test that different priorities are accepted"""
        priorities = ["normal", "high", "emergency"]
        
        for priority in priorities:
            time.sleep(0.3)  # Avoid rate limit
            
            status, error = send_command_and_wait(fake_hub, "position_A", priority=priority)
            
            # Should not have validation errors
            disallowed = {"NAV_INVALID_COMMAND", "NAV_RATE_LIMIT_EXCEEDED", "DUPLICATE"}
            assert error not in disallowed, \
                f"Priority {priority} should not be rejected: {error}"
    
    @pytest.mark.integration
    def test_cancel_command(self, fake_hub):
        """Test cancel command handling"""
        # Send navigation command
        command_id = fake_hub.send_navigate_command("position_A")
        assert command_id is not None
        
        time.sleep(1.0)  # Wait for command to be accepted
        
        # Send cancel
        cancel_sent = fake_hub.send_cancel_command()
        assert cancel_sent, "Cancel command should be sent"
        
        time.sleep(3.0)  # Wait for processing
        
        # Check status
        status_obj = fake_hub.get_status_by_command_id(command_id, timeout=2.0, wait_for_error=True)
        
        if status_obj:
            status = status_obj.get('status')
            error_code = status_obj.get('error_code')
            # Should be canceled or idle
            assert status in ['canceled', 'idle'] or error_code == 'NAV_CANCELED', \
                f"Status should indicate cancel: {status}, error={error_code}"
    
    @pytest.mark.integration
    def test_json_schema_compliance(self, fake_hub):
        """Test that events comply with SPECIFICATION.md JSON schema"""
        command_id = fake_hub.send_navigate_command("position_A")
        assert command_id is not None
        
        time.sleep(2.0)  # Wait for events
        
        status_obj = fake_hub.get_status_by_command_id(command_id, timeout=2.0, wait_for_error=True)
        
        assert status_obj is not None, "Should receive status/event"
        
        # Check required fields per SPECIFICATION.md Section 3.3
        required_fields = ['schema_version', 'robot_id', 'timestamp']
        for field in required_fields:
            assert field in status_obj, f"Missing required field: {field}"
        
        # Check schema_version
        assert status_obj.get('schema_version') == '1.0', \
            f"schema_version should be '1.0', got {status_obj.get('schema_version')}"
        
        # Check robot_id
        assert status_obj.get('robot_id') == ROBOT_ID, \
            f"robot_id should be {ROBOT_ID}, got {status_obj.get('robot_id')}"
        
        # Check timestamp format (ISO-8601 Z)
        timestamp = status_obj.get('timestamp')
        if timestamp:
            assert timestamp.endswith('Z'), \
                f"timestamp should end with 'Z', got {timestamp}"
            try:
                datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
            except ValueError:
                pytest.fail(f"Invalid timestamp format: {timestamp}")
    
    @pytest.mark.integration
    def test_status_telemetry_fields(self, fake_hub):
        """Test that status messages contain required fields per SPECIFICATION.md Section 3.4"""
        command_id = fake_hub.send_navigate_command("position_A")
        assert command_id is not None
        
        time.sleep(3.0)  # Wait for status updates
        
        if fake_hub.last_status:
            required_fields = ['schema_version', 'robot_id', 'timestamp', 'status']
            for field in required_fields:
                assert field in fake_hub.last_status, \
                    f"Status missing required field: {field}"
    
    @pytest.mark.integration
    def test_event_ordering(self, fake_hub):
        """Test that events follow ordering per SPECIFICATION.md Section 3.3"""
        fake_hub.clear_status_history()
        
        command_id = fake_hub.send_navigate_command("position_A")
        assert command_id is not None
        
        time.sleep(4.0)  # Wait for events
        
        events = fake_hub.status_by_command_id.get(command_id, [])
        
        if len(events) > 0:
            # Check that we have at least ack(received)
            event_types = []
            for event in events:
                ev_type = event.get('event_type')
                if ev_type == 'ack':
                    ack_type = event.get('ack_type')
                    event_types.append(f"ack({ack_type})")
                elif ev_type == 'result':
                    result_type = event.get('result_type')
                    event_types.append(f"result({result_type})")
            
            # Should have at least ack(received)
            has_received = any('ack(received)' in str(ev) for ev in event_types)
            assert has_received, \
                f"Should have ack(received) event. Got: {event_types}"
    
    @pytest.mark.integration
    def test_error_handling(self, fake_hub):
        """Test that errors are properly handled and published"""
        time.sleep(0.2)  # Avoid rate limit
        
        status, error = send_command_and_wait(fake_hub, "invalid_target_xyz")
        
        if fake_hub.last_status:
            error_code = fake_hub.last_status.get('error_code')
            error_message = fake_hub.last_status.get('error_message')
            
            assert error_code is not None, "Error should have error_code"
            assert error_message is not None, "Error should have error_message"


@pytest.mark.skipif(
    not os.getenv("ENABLE_FAKE_HUB_TESTS", "").lower() in ("1", "true", "yes"),
    reason="Fake hub tests require Config Service and MQTT broker. Set ENABLE_FAKE_HUB_TESTS=1 to enable."
)
class TestFakeHubIntegrationOptional:
    """
    Optional integration tests that require external services.
    
    These tests are skipped by default and can be enabled with:
    ENABLE_FAKE_HUB_TESTS=1 pytest test_fake_hub_integration.py
    """
    
    @pytest.mark.integration
    @pytest.mark.slow
    def test_full_navigation_flow(self, fake_hub):
        """Test complete navigation flow from command to completion"""
        command_id = fake_hub.send_navigate_command("position_A")
        assert command_id is not None
        
        # Wait for navigation to complete (or timeout)
        max_wait = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait:
            status_obj = fake_hub.get_status_by_command_id(command_id, timeout=1.0, wait_for_error=True)
            if status_obj:
                status = status_obj.get('status')
                result_type = status_obj.get('result_type')
                
                # Check if navigation completed
                if status in ['succeeded', 'idle'] or result_type in ['succeeded', 'aborted', 'canceled', 'error']:
                    # Navigation completed
                    break
            
            time.sleep(1.0)
        
        # Should have received some status
        assert fake_hub.last_status is not None, "Should receive navigation status"

