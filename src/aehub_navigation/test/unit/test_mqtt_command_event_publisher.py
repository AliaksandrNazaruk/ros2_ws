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
Unit tests for MQTTCommandEventPublisher

Tests command event publishing (ACK and RESULT events) with JSON schema validation.
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
import json
from datetime import datetime
from aehub_navigation.mqtt_command_event_publisher import MQTTCommandEventPublisher
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager


class TestMQTTCommandEventPublisher:
    """Test suite for MQTTCommandEventPublisher"""
    
    @pytest.fixture
    def event_publisher(self):
        """Create a MQTTCommandEventPublisher instance"""
        return MQTTCommandEventPublisher()
    
    @pytest.fixture
    def mock_mqtt_manager(self):
        """Create a mock MQTTConnectionManager"""
        manager = MagicMock(spec=MQTTConnectionManager)
        manager.publish = Mock(return_value=True)
        return manager
    
    @pytest.fixture
    def configured_publisher(self, event_publisher, mock_mqtt_manager):
        """Create a configured publisher with MQTT manager and robot_id"""
        event_publisher.set_mqtt_manager(mock_mqtt_manager)
        event_publisher.set_robot_id('robot_001')
        return event_publisher
    
    def test_initialization(self, event_publisher):
        """Test that publisher initializes correctly"""
        assert event_publisher.mqtt_manager is None
        assert event_publisher.robot_id is None
        assert event_publisher.events_topic is None
    
    def test_set_mqtt_manager(self, event_publisher, mock_mqtt_manager):
        """Test setting MQTT manager"""
        event_publisher.set_mqtt_manager(mock_mqtt_manager)
        
        assert event_publisher.mqtt_manager == mock_mqtt_manager
    
    def test_set_robot_id(self, event_publisher):
        """Test setting robot_id"""
        event_publisher.set_robot_id('robot_002')
        
        assert event_publisher.robot_id == 'robot_002'
        assert event_publisher.events_topic == 'aroc/robot/robot_002/commands/events'
    
    def test_publish_ack_received(self, configured_publisher, mock_mqtt_manager):
        """Test publishing ACK received event"""
        configured_publisher.publish_ack('cmd_123', 'target_A', 'received')
        
        assert mock_mqtt_manager.publish.called
        call_args = mock_mqtt_manager.publish.call_args
        assert call_args[0][0] == 'aroc/robot/robot_001/commands/events'
        assert call_args[0][1] is not None  # JSON payload
        assert call_args[1]['qos'] == 1
        
        # Verify JSON schema
        payload = json.loads(call_args[0][1])
        assert payload['schema_version'] == '1.0'
        assert payload['robot_id'] == 'robot_001'
        assert payload['event_type'] == 'ack'
        assert payload['ack_type'] == 'received'
        assert payload['command_id'] == 'cmd_123'
        assert payload['target_id'] == 'target_A'
        assert 'timestamp' in payload
    
    def test_publish_ack_accepted(self, configured_publisher, mock_mqtt_manager):
        """Test publishing ACK accepted event"""
        configured_publisher.publish_ack('cmd_456', 'target_B', 'accepted')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['ack_type'] == 'accepted'
        assert payload['command_id'] == 'cmd_456'
        assert payload['target_id'] == 'target_B'
    
    def test_publish_ack_rejected(self, configured_publisher, mock_mqtt_manager):
        """Test publishing ACK rejected event with reason"""
        configured_publisher.publish_ack('cmd_789', 'target_C', 'rejected', reason='validation_failed')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['ack_type'] == 'rejected'
        assert payload['command_id'] == 'cmd_789'
        assert payload['reason'] == 'validation_failed'
    
    def test_publish_ack_rejected_no_reason(self, configured_publisher, mock_mqtt_manager):
        """Test publishing ACK rejected event without reason"""
        configured_publisher.publish_ack('cmd_999', 'target_D', 'rejected')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['ack_type'] == 'rejected'
        assert 'reason' not in payload
    
    def test_publish_ack_target_id_none(self, configured_publisher, mock_mqtt_manager):
        """Test publishing ACK with None target_id"""
        configured_publisher.publish_ack('cmd_000', None, 'received')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['target_id'] is None
    
    def test_publish_result_succeeded(self, configured_publisher, mock_mqtt_manager):
        """Test publishing RESULT succeeded event"""
        configured_publisher.publish_result('cmd_111', 'target_E', 'succeeded')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['schema_version'] == '1.0'
        assert payload['robot_id'] == 'robot_001'
        assert payload['event_type'] == 'result'
        assert payload['result_type'] == 'succeeded'
        assert payload['command_id'] == 'cmd_111'
        assert payload['target_id'] == 'target_E'
        assert payload['error_code'] is None
        assert payload['error_message'] is None
        assert 'timestamp' in payload
    
    def test_publish_result_aborted(self, configured_publisher, mock_mqtt_manager):
        """Test publishing RESULT aborted event with error"""
        configured_publisher.publish_result(
            'cmd_222', 'target_F', 'aborted',
            error_code='NAV_GOAL_ABORTED',
            error_message='Navigation goal was aborted'
        )
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['result_type'] == 'aborted'
        assert payload['error_code'] == 'NAV_GOAL_ABORTED'
        assert payload['error_message'] == 'Navigation goal was aborted'
    
    def test_publish_result_canceled(self, configured_publisher, mock_mqtt_manager):
        """Test publishing RESULT canceled event"""
        configured_publisher.publish_result('cmd_333', 'target_G', 'canceled')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['result_type'] == 'canceled'
        assert payload['error_code'] is None
    
    def test_publish_result_error(self, configured_publisher, mock_mqtt_manager):
        """Test publishing RESULT error event"""
        configured_publisher.publish_result(
            'cmd_444', 'target_H', 'error',
            error_code='NAV_INVALID_TARGET',
            error_message='Target position not found'
        )
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        assert payload['result_type'] == 'error'
        assert payload['error_code'] == 'NAV_INVALID_TARGET'
        assert payload['error_message'] == 'Target position not found'
    
    def test_publish_ack_without_mqtt_manager(self, event_publisher):
        """Test publishing ACK without MQTT manager (should not crash)"""
        event_publisher.set_robot_id('robot_003')
        # Should not raise exception
        event_publisher.publish_ack('cmd_555', 'target_I', 'received')
    
    def test_publish_ack_without_robot_id(self, event_publisher, mock_mqtt_manager):
        """Test publishing ACK without robot_id (should not crash)"""
        event_publisher.set_mqtt_manager(mock_mqtt_manager)
        # Should not raise exception
        event_publisher.publish_ack('cmd_666', 'target_J', 'received')
        # Should not publish
        assert not mock_mqtt_manager.publish.called
    
    def test_publish_result_without_mqtt_manager(self, event_publisher):
        """Test publishing RESULT without MQTT manager (should not crash)"""
        event_publisher.set_robot_id('robot_004')
        # Should not raise exception
        event_publisher.publish_result('cmd_777', 'target_K', 'succeeded')
    
    def test_publish_event_exception_handling(self, configured_publisher, mock_mqtt_manager):
        """Test that exceptions in _publish_event are handled gracefully"""
        mock_mqtt_manager.publish.side_effect = Exception("MQTT error")
        
        # Should not raise exception
        result = configured_publisher.publish_ack('cmd_888', 'target_L', 'received')
        # _publish_event returns False on exception
        # But publish_ack doesn't return value, so we just check it doesn't crash
    
    def test_timestamp_format(self, configured_publisher, mock_mqtt_manager):
        """Test that timestamp is in ISO-8601 Z format"""
        configured_publisher.publish_ack('cmd_999', 'target_M', 'received')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        timestamp = payload['timestamp']
        
        # Should end with 'Z' and be valid ISO-8601
        assert timestamp.endswith('Z')
        # Try to parse it
        datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    
    def test_json_schema_compliance_ack(self, configured_publisher, mock_mqtt_manager):
        """Test that ACK events comply with SPECIFICATION.md Section 3.3"""
        configured_publisher.publish_ack('550e8400-e29b-41d4-a716-446655440000', 'target_N', 'accepted', reason='test')
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        
        # Required fields per spec
        assert 'schema_version' in payload
        assert 'robot_id' in payload
        assert 'timestamp' in payload
        assert 'event_type' in payload
        assert 'command_id' in payload
        assert 'target_id' in payload
        assert 'ack_type' in payload
        # Optional field
        assert 'reason' in payload
    
    def test_json_schema_compliance_result(self, configured_publisher, mock_mqtt_manager):
        """Test that RESULT events comply with SPECIFICATION.md Section 3.3"""
        configured_publisher.publish_result(
            '550e8400-e29b-41d4-a716-446655440001', 'target_O', 'error',
            error_code='NAV_INVALID_TARGET',
            error_message='Target not found'
        )
        
        call_args = mock_mqtt_manager.publish.call_args
        payload = json.loads(call_args[0][1])
        
        # Required fields per spec
        assert 'schema_version' in payload
        assert 'robot_id' in payload
        assert 'timestamp' in payload
        assert 'event_type' in payload
        assert 'command_id' in payload
        assert 'target_id' in payload
        assert 'result_type' in payload
        # Optional fields
        assert 'error_code' in payload
        assert 'error_message' in payload
    
    def test_qos_always_one(self, configured_publisher, mock_mqtt_manager):
        """Test that all events are published with QoS 1"""
        configured_publisher.publish_ack('cmd_aaa', 'target_P', 'received')
        call_args1 = mock_mqtt_manager.publish.call_args
        assert call_args1[1]['qos'] == 1
        
        configured_publisher.publish_result('cmd_bbb', 'target_Q', 'succeeded')
        call_args2 = mock_mqtt_manager.publish.call_args
        assert call_args2[1]['qos'] == 1
    
    def test_multiple_events_sequential(self, configured_publisher, mock_mqtt_manager):
        """Test publishing multiple events sequentially"""
        configured_publisher.publish_ack('cmd_1', 'target_1', 'received')
        configured_publisher.publish_ack('cmd_1', 'target_1', 'accepted')
        configured_publisher.publish_result('cmd_1', 'target_1', 'succeeded')
        
        assert mock_mqtt_manager.publish.call_count == 3
        
        # Verify ordering: received -> accepted -> succeeded
        calls = mock_mqtt_manager.publish.call_args_list
        payload1 = json.loads(calls[0][0][1])
        payload2 = json.loads(calls[1][0][1])
        payload3 = json.loads(calls[2][0][1])
        
        assert payload1['ack_type'] == 'received'
        assert payload2['ack_type'] == 'accepted'
        assert payload3['result_type'] == 'succeeded'

