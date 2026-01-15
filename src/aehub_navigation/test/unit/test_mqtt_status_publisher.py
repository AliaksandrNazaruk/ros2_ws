#!/usr/bin/env python3

# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for MQTTStatusPublisher

Tests strict payload format according to AE.HUB MVP SRS Draft 0.3.
"""

import pytest
from unittest.mock import Mock, MagicMock, call
from datetime import datetime
from aehub_navigation.mqtt_status_publisher import MQTTStatusPublisher
from aehub_navigation.navigation_state_manager import NavigationState
import json


class TestMQTTStatusPublisher:
    """Test suite for MQTTStatusPublisher"""
    
    @pytest.fixture
    def status_publisher(self):
        """Create a MQTTStatusPublisher instance"""
        return MQTTStatusPublisher()
    
    @pytest.fixture
    def mock_mqtt_manager(self):
        """Create a mock MQTTConnectionManager"""
        mock = Mock()
        mock.publish = Mock(return_value=True)
        mock.is_connected = True
        return mock
    
    @pytest.fixture
    def configured_publisher(self, status_publisher, mock_mqtt_manager):
        """Create a configured status publisher"""
        status_publisher.set_mqtt_manager(mock_mqtt_manager)
        status_publisher.set_robot_id("test_robot_001")
        return status_publisher
    
    def test_initialization(self, status_publisher):
        """Test that status publisher initializes correctly"""
        assert status_publisher.mqtt_manager is None
        assert status_publisher.robot_id is None
        assert status_publisher.status_topic is None
        assert status_publisher.current_status is None
    
    def test_set_mqtt_manager(self, status_publisher, mock_mqtt_manager):
        """Test setting MQTT manager"""
        status_publisher.set_mqtt_manager(mock_mqtt_manager)
        assert status_publisher.mqtt_manager == mock_mqtt_manager
    
    def test_set_robot_id(self, status_publisher):
        """Test setting robot ID"""
        robot_id = "test_robot_001"
        status_publisher.set_robot_id(robot_id)
        assert status_publisher.robot_id == robot_id
        assert status_publisher.status_topic == f"aroc/robot/{robot_id}/status/navigation"
    
    def test_update_status(self, configured_publisher):
        """Test updating status"""
        configured_publisher.updateStatus(
            state=NavigationState.NAVIGATING,
            target_id="position_A",
            progress_percent=50,
            eta_seconds=30,
            current_position={'x': 1.0, 'y': 2.0, 'theta': 0.0}
        )
        
        assert configured_publisher.current_status == "navigating"
        assert configured_publisher.current_target_id == "position_A"
        assert configured_publisher.progress_percent == 50
        assert configured_publisher.eta_seconds == 30
        assert configured_publisher.current_position == {'x': 1.0, 'y': 2.0, 'theta': 0.0}
    
    def test_publish_status_idle(self, configured_publisher):
        """Test publishing idle status with strict format"""
        configured_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 0.0, 'y': 0.0, 'theta': 0.0}
        )
        
        configured_publisher.publishStatus()
        
        # Verify publish was called
        assert configured_publisher.mqtt_manager.publish.called
        
        # Get the published payload
        call_args = configured_publisher.mqtt_manager.publish.call_args
        topic = call_args[0][0]
        payload_str = call_args[0][1]
        qos = call_args.kwargs.get('qos', 1)  # qos is passed as keyword argument
        
        assert topic == "aroc/robot/test_robot_001/status/navigation"
        assert qos == 1
        
        # Parse and verify payload structure
        payload = json.loads(payload_str)
        
        # Check all required fields are present
        assert 'schema_version' in payload
        assert payload['schema_version'] == '1.0'
        assert 'robot_id' in payload
        assert payload['robot_id'] == 'test_robot_001'
        assert 'timestamp' in payload
        assert 'status' in payload
        assert payload['status'] == 'idle'
        assert 'target_id' in payload
        assert payload['target_id'] is None
        assert 'progress_percent' in payload
        assert payload['progress_percent'] == 0
        assert 'eta_seconds' in payload
        assert payload['eta_seconds'] == 0
        assert 'current_position' in payload
        assert 'error_code' in payload
        assert payload['error_code'] is None
        assert 'error_message' in payload
        assert payload['error_message'] is None
        
        # Verify current_position structure
        assert payload['current_position']['x'] == 0.0
        assert payload['current_position']['y'] == 0.0
        assert payload['current_position']['theta'] == 0.0
        
        # Verify timestamp is ISO-8601 format
        datetime.fromisoformat(payload['timestamp'].replace('Z', '+00:00'))
    
    def test_publish_status_navigating(self, configured_publisher):
        """Test publishing navigating status with strict format"""
        configured_publisher.updateStatus(
            state=NavigationState.NAVIGATING,
            target_id="position_A",
            progress_percent=42,
            eta_seconds=15,
            current_position={'x': 1.5, 'y': 2.5, 'theta': 0.785}
        )
        
        configured_publisher.publishStatus()
        
        payload_str = configured_publisher.mqtt_manager.publish.call_args[0][1]
        payload = json.loads(payload_str)
        
        assert payload['status'] == 'navigating'
        assert payload['target_id'] == 'position_A'
        assert payload['progress_percent'] == 42
        assert payload['eta_seconds'] == 15
        assert payload['current_position']['x'] == 1.5
        assert payload['current_position']['y'] == 2.5
        assert payload['current_position']['theta'] == 0.785
        assert payload['error_code'] is None
        assert payload['error_message'] is None
    
    def test_publish_status_error(self, configured_publisher):
        """Test publishing error status with strict format"""
        configured_publisher.updateStatus(
            state=NavigationState.ERROR,
            target_id="position_B",
            error_code="NAV_GOAL_ABORTED",
            error_message="Test error message",
            current_position={'x': 2.0, 'y': 3.0, 'theta': 1.57}
        )
        
        configured_publisher.publishStatus()
        
        payload_str = configured_publisher.mqtt_manager.publish.call_args[0][1]
        payload = json.loads(payload_str)
        
        assert payload['status'] == 'error'
        assert payload['target_id'] == 'position_B'
        assert payload['error_code'] == 'NAV_GOAL_ABORTED'
        assert payload['error_message'] == 'Test error message'
        assert payload['progress_percent'] == 0  # Still present
        assert payload['eta_seconds'] == 0  # Still present
    
    def test_publish_status_succeeded(self, configured_publisher):
        """Test publishing succeeded status with strict format"""
        configured_publisher.updateStatus(
            state=NavigationState.SUCCEEDED,
            target_id="position_C",
            current_position={'x': 3.0, 'y': 4.0, 'theta': 0.0}
        )
        
        configured_publisher.publishStatus()
        
        payload_str = configured_publisher.mqtt_manager.publish.call_args[0][1]
        payload = json.loads(payload_str)
        
        assert payload['status'] == 'succeeded'
        assert payload['target_id'] == 'position_C'
        assert payload['progress_percent'] == 0
        assert payload['eta_seconds'] == 0
        assert payload['error_code'] is None
        assert payload['error_message'] is None
    
    def test_publish_without_mqtt_manager(self, status_publisher):
        """Test that publish does nothing without MQTT manager"""
        status_publisher.set_robot_id("test_robot")
        status_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 0.0, 'y': 0.0, 'theta': 0.0}
        )
        
        # Should not raise exception, just return
        status_publisher.publishStatus()
        # No assertions needed - just verify no exception
    
    def test_publish_without_robot_id(self, status_publisher, mock_mqtt_manager):
        """Test that publish does nothing without robot_id"""
        status_publisher.set_mqtt_manager(mock_mqtt_manager)
        status_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 0.0, 'y': 0.0, 'theta': 0.0}
        )
        
        status_publisher.publishStatus()
        
        # Should not call publish
        assert not mock_mqtt_manager.publish.called
    
    def test_publish_without_status(self, configured_publisher):
        """Test that publish uses default 'idle' status when no status set"""
        # Don't call updateStatus - current_status should be None
        
        # Clear any default status
        configured_publisher.current_status = None
        
        configured_publisher.publishStatus()
        
        # Should publish with default 'idle' status (per SPECIFICATION.md)
        assert configured_publisher.mqtt_manager.publish.called
        call_args = configured_publisher.mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['status'] == 'idle'
    
    def test_update_status_preserves_current_position(self, configured_publisher):
        """Test that updating status preserves current_position if not provided"""
        # Set initial position
        configured_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 1.0, 'y': 2.0, 'theta': 0.5}
        )
        
        # Update without position
        configured_publisher.updateStatus(
            state=NavigationState.NAVIGATING,
            target_id="position_A"
        )
        
        # Position should still be the old one
        assert configured_publisher.current_position == {'x': 1.0, 'y': 2.0, 'theta': 0.5}
    
    def test_update_status_updates_current_position(self, configured_publisher):
        """Test that updating status updates current_position if provided"""
        configured_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 1.0, 'y': 2.0, 'theta': 0.5}
        )
        
        new_position = {'x': 3.0, 'y': 4.0, 'theta': 1.0}
        configured_publisher.updateStatus(
            state=NavigationState.NAVIGATING,
            target_id="position_A",
            current_position=new_position
        )
        
        # Position should be updated
        assert configured_publisher.current_position == new_position
        # Should be a copy, not the same object
        assert configured_publisher.current_position is not new_position
    
    def test_publish_qos_always_one(self, configured_publisher):
        """Test that publish always uses QoS=1"""
        configured_publisher.updateStatus(
            state=NavigationState.IDLE,
            current_position={'x': 0.0, 'y': 0.0, 'theta': 0.0}
        )
        
        configured_publisher.publishStatus()
        
        call_args = configured_publisher.mqtt_manager.publish.call_args
        qos = call_args.kwargs.get('qos', 1)  # qos is passed as keyword argument
        assert qos == 1
    
    def test_schema_version_always_1_0(self, configured_publisher):
        """Test that schema_version is always '1.0'"""
        for state in NavigationState:
            configured_publisher.updateStatus(
                state=state,
                current_position={'x': 0.0, 'y': 0.0, 'theta': 0.0}
            )
            configured_publisher.publishStatus()
            
            payload_str = configured_publisher.mqtt_manager.publish.call_args[0][1]
            payload = json.loads(payload_str)
            assert payload['schema_version'] == '1.0'
            
            # Reset mock for next iteration
            configured_publisher.mqtt_manager.publish.reset_mock()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

