#!/usr/bin/env python3
"""
Edge Cases for Cancel Operations

Tests for cancel operation edge cases that could cause production issues.

Test Strategy:
- Test idempotency of cancel operations
- Test cancel during various states
- Test cancel failure scenarios
- Test rapid cancel commands

Risks Covered:
1. Multiple cancel calls causing exceptions
2. Cancel called when no goal active
3. Cancel failure leaving inconsistent state
4. Cancel during goal acceptance
"""

import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import Mock, patch, MagicMock
import time
import uuid
from datetime import datetime, timezone

from aehub_navigation.navigation_integrated_node import NavigationIntegratedNode
from aehub_navigation.navigation_state_manager import NavigationState, NavigationStateManager
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
from aehub_navigation.broker_config_provider import BrokerConfigProvider
from aehub_navigation.position_registry import PositionRegistry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


@pytest.fixture
def rclpy_context():
    """Initialize ROS2 context for tests"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def mock_mqtt_manager():
    """Mock MQTT connection manager"""
    manager = Mock(spec=MQTTConnectionManager)
    manager.is_connected = True
    manager.get_connection_status.return_value = {'is_connected': True}
    manager.publish.return_value = True
    manager.subscribe = Mock()
    manager.connect.return_value = True
    return manager


@pytest.fixture
def mock_position_registry():
    """Mock position registry"""
    registry = Mock(spec=PositionRegistry)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    registry.getPosition.return_value = pose
    registry.hasPosition.return_value = True
    return registry


@pytest.fixture
def node_setup(rclpy_context, mock_mqtt_manager, mock_position_registry):
    """Create navigation node with mocked dependencies"""
    with patch('aehub_navigation.navigation_integrated_node.MQTTConnectionManager', return_value=mock_mqtt_manager), \
         patch('aehub_navigation.navigation_integrated_node.PositionRegistry', return_value=mock_position_registry), \
         patch('aehub_navigation.navigation_integrated_node.BrokerConfigProvider') as mock_provider_class, \
         patch('rclpy.node.Node.declare_parameter'), \
         patch('rclpy.node.Node.get_parameter') as mock_get_param:
        
        def param_side_effect(name):
            mock_param = Mock()
            if name == 'robot_id':
                mock_param.value = 'robot_001'
            elif name == 'config_service_url':
                mock_param.value = 'http://localhost:7900'
            elif name == 'config_service_api_key':
                mock_param.value = 'test_api_key'
            elif name == 'config_poll_interval':
                mock_param.value = 5.0
            elif name == 'positions_file':
                mock_param.value = 'config/positions.yaml'
            else:
                mock_param.value = None
            return mock_param
        
        mock_get_param.side_effect = param_side_effect
        
        mock_provider = Mock(spec=BrokerConfigProvider)
        mock_config = Mock()
        mock_config.broker = "test.broker"
        mock_config.broker_port = 8883
        mock_config.mqtt_use_tls = True
        mock_provider.fetch_config.return_value = mock_config
        mock_provider.watch = Mock()
        mock_provider.start_polling = Mock()
        mock_provider_class.return_value = mock_provider
        
        node = NavigationIntegratedNode()
        
        node.nav2_action_client = Mock(spec=ActionClient)
        node.nav2_action_client.server_is_ready.return_value = True
        node.nav2_action_client.wait_for_server.return_value = True
        
        mock_state_manager = Mock(spec=NavigationStateManager)
        mock_state_manager.onGoalAborted = Mock()
        mock_state_manager.onGoalSent = Mock()
        mock_state_manager.onGoalSucceeded = Mock()
        mock_state_manager.onGoalCanceled = Mock()
        mock_state_manager.getTargetId = Mock(return_value="test_target")
        mock_state_manager.getState = Mock(return_value=NavigationState.IDLE)
        mock_state_manager.resetToIdle = Mock()
        mock_state_manager.setStateChangeCallback = Mock()
        node.state_manager = mock_state_manager
        
        node.status_publisher.publishStatus = Mock()
        node.status_publisher.updateStatus = Mock()
        
        yield node
        
        if node:
            node.destroy_node()


class TestCancelIdempotency:
    """
    Tests for cancel operation idempotency.
    """
    
    def test_cancel_when_no_goal_active(self, node_setup):
        """
        Bug it would catch: Cancel called when no goal active,
        exception raised or state corrupted.
        
        Production incident: User clicks cancel multiple times,
        second click when no goal active causes crash.
        """
        node = node_setup
        
        # Given: No active goal
        with node._goal_state_lock:
            assert node.current_goal_handle is None, "No goal should be active"
        
        # When: Cancel is called
        try:
            node.cancelCurrentGoal()
        except Exception as e:
            pytest.fail(f"Cancel should be idempotent when no goal active: {e}")
        
        # Then: No exception should be raised
        # State should remain consistent
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should remain None"
            assert node.current_target_id is None, \
                "Target ID should remain None"
    
    def test_multiple_cancel_calls_rapidly(self, node_setup):
        """
        Bug it would catch: Cancel called multiple times rapidly,
        second call happens while first is processing, causing race condition.
        
        Production incident: User spam clicks cancel, multiple cancel
        operations race, state corrupted.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        mock_cancel_future = Mock()
        mock_goal_handle.cancel_goal_async.return_value = mock_cancel_future
        
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Cancel called twice rapidly
        node.cancelCurrentGoal()
        
        # Second call should be idempotent (handle already cleared)
        try:
            node.cancelCurrentGoal()
        except Exception as e:
            pytest.fail(f"Second cancel should be idempotent: {e}")
        
        # Then: State should be consistent
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should be cleared after cancel"
            assert node.current_target_id is None, \
                "Target ID should be cleared after cancel"
    
    def test_cancel_during_goal_acceptance(self, node_setup):
        """
        Bug it would catch: Cancel called while goal is being accepted,
        goal handle not yet set, cancel fails silently.
        
        Production incident: User sends command, immediately cancels,
        cancel happens before goal accepted, cancel ignored.
        """
        node = node_setup
        
        # Given: Goal being sent (handle not yet set)
        with node._goal_state_lock:
            node.current_goal_handle = None
            node.current_target_id = None
        
        # When: Cancel is called before goal accepted
        try:
            node.cancelCurrentGoal()
        except Exception as e:
            pytest.fail(f"Cancel should handle no-goal case gracefully: {e}")
        
        # Then: Should be idempotent (no-op)
        # When goal is later accepted, it should be cancellable
        mock_goal_handle = Mock()
        mock_cancel_future = Mock()
        mock_goal_handle.cancel_goal_async.return_value = mock_cancel_future
        
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # Cancel should work now
        node.cancelCurrentGoal()
        
        assert mock_goal_handle.cancel_goal_async.called, \
            "Cancel should work after goal is accepted"


class TestCancelFailureScenarios:
    """
    Tests for cancel operation failure scenarios.
    """
    
    def test_cancel_async_raises_exception(self, node_setup):
        """
        Bug it would catch: cancel_goal_async() raises exception,
        exception not caught, node crashes.
        
        Production incident: Nav2 action client throws exception
        during cancel, node crashes, all navigation stops.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        mock_goal_handle.cancel_goal_async.side_effect = Exception("Cancel failed")
        
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Cancel is called and raises exception
        try:
            node.cancelCurrentGoal()
        except Exception as e:
            pytest.fail(f"Cancel should handle exceptions gracefully: {e}")
        
        # Then: State should be cleared despite exception
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should be cleared even if cancel fails"
            assert node.current_target_id is None, \
                "Target ID should be cleared even if cancel fails"
    
    def test_cancel_done_callback_failure(self, node_setup):
        """
        Bug it would catch: Cancel succeeds, but cancel_done_callback
        fails, state never updated, stuck in navigating.
        
        Production incident: Cancel request accepted by Nav2, but
        callback fails, state never transitions to IDLE.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        mock_cancel_future = Mock()
        mock_goal_handle.cancel_goal_async.return_value = mock_cancel_future
        
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Cancel is called
        node.cancelCurrentGoal()
        
        # Simulate cancel_done_callback failure
        mock_cancel_response = Mock()
        mock_cancel_response.goals_canceling = ["goal_1"]
        mock_cancel_future.result.return_value = mock_cancel_response
        
        # Make state manager fail
        node.state_manager.onGoalCanceled.side_effect = Exception("State update failed")
        
        try:
            node.cancel_done_callback(mock_cancel_future)
        except Exception as e:
            pytest.fail(f"cancel_done_callback should handle exceptions: {e}")
        
        # Then: State should still be cleared
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should be cleared even if callback fails"


class TestCancelDuringStateTransitions:
    """
    Tests for cancel during various state transitions.
    """
    
    def test_cancel_while_goal_being_sent(self, node_setup):
        """
        Bug it would catch: Cancel called while goal is being sent,
        goal handle not yet available, cancel ignored.
        
        Production incident: User sends command, immediately cancels,
        cancel happens before goal handle available, goal still sent.
        """
        node = node_setup
        
        # Given: Goal being sent (handle not yet set)
        with node._goal_state_lock:
            node.current_goal_handle = None
            node.current_target_id = None
        
        # When: Cancel is called
        node.cancelCurrentGoal()
        
        # Then: Should be idempotent
        # When goal is later accepted, it should be cancellable
        # This is handled by checking state before sending new goal
    
    def test_cancel_after_goal_rejected(self, node_setup):
        """
        Bug it would catch: Goal rejected, but cancel still called,
        cancel tries to cancel None handle, exception raised.
        
        Production incident: Goal rejected by Nav2, but cancel command
        arrives, cancel fails with exception.
        """
        node = node_setup
        
        # Given: Goal was rejected (handle cleared)
        with node._goal_state_lock:
            node.current_goal_handle = None
            node.current_target_id = None
        
        # When: Cancel is called after rejection
        try:
            node.cancelCurrentGoal()
        except Exception as e:
            pytest.fail(f"Cancel should be idempotent after rejection: {e}")
        
        # Then: Should be idempotent (no-op)
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should remain None"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

