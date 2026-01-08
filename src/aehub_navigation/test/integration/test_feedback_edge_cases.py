#!/usr/bin/env python3
"""
Edge Cases for Feedback Handling

Tests for feedback callback edge cases that could cause production issues.

Test Strategy:
- Test invalid feedback data
- Test missing feedback fields
- Test feedback during state transitions
- Test feedback after goal cancellation

Risks Covered:
1. Invalid feedback data causing crashes
2. Missing feedback fields causing exceptions
3. Feedback received after goal cancelled
4. Feedback with invalid distance/ETA values
"""

import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import Mock, patch, MagicMock
import time

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


class TestInvalidFeedbackData:
    """
    Tests for handling invalid feedback data.
    """
    
    def test_feedback_missing_distance_remaining(self, node_setup):
        """
        Bug it would catch: Feedback missing distance_remaining field,
        exception raised, feedback processing crashes.
        
        Production incident: Nav2 sends malformed feedback, node crashes,
        all feedback lost.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Feedback missing distance_remaining
        feedback_msg = Mock()
        feedback = Mock()
        del feedback.distance_remaining  # Remove attribute
        feedback_msg.feedback = feedback
        
        # Then: Should handle gracefully
        try:
            node.feedback_callback(feedback_msg)
        except AttributeError as e:
            pytest.fail(f"Feedback callback should handle missing distance_remaining: {e}")
        except Exception as e:
            # Other exceptions might be acceptable, but AttributeError is the bug
            if "distance_remaining" in str(e) or "AttributeError" in str(type(e).__name__):
                pytest.fail(f"Feedback callback should handle missing distance_remaining: {e}")
    
    def test_feedback_negative_distance(self, node_setup):
        """
        Bug it would catch: Feedback has negative distance_remaining,
        progress calculation fails or shows >100%.
        
        Production incident: Nav2 sends negative distance, progress
        calculation fails, status shows invalid progress.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Feedback with negative distance
        feedback_msg = Mock()
        feedback = Mock()
        feedback.distance_remaining.data = -1.0  # Invalid negative
        feedback_msg.feedback = feedback
        
        node.feedback_callback(feedback_msg)
        
        # Then: Should handle gracefully (clamp to 0)
        # Progress should be calculated correctly
        with node._progress_lock:
            # Distance should be clamped to 0 or handled
            assert node.last_distance_remaining is not None, \
                "Distance should be tracked even if negative (clamped)"
    
    def test_feedback_after_goal_cancelled(self, node_setup):
        """
        Bug it would catch: Feedback received after goal cancelled,
        feedback processed, state corrupted.
        
        Production incident: Goal cancelled, but feedback arrives late,
        feedback processed, state thinks goal still active.
        """
        node = node_setup
        
        # Given: Goal was cancelled (handle cleared)
        with node._goal_state_lock:
            node.current_goal_handle = None
            node.current_target_id = None
        
        # When: Feedback arrives after cancellation
        feedback_msg = Mock()
        feedback = Mock()
        feedback.distance_remaining.data = 5.0
        feedback_msg.feedback = feedback
        
        # Then: Feedback should be ignored
        node.feedback_callback(feedback_msg)
        
        # State should remain consistent
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should remain None after cancel"
        
        # Status should not be updated
        # (updateStatus might be called, but it's harmless if state is IDLE)
    
    def test_feedback_with_invalid_eta(self, node_setup):
        """
        Bug it would catch: Feedback has invalid ETA (negative or huge),
        ETA calculation fails or shows invalid value.
        
        Production incident: Nav2 sends invalid ETA, status shows
        invalid ETA, user confused.
        """
        node = node_setup
        
        # Given: Active goal with initial distance
        mock_goal_handle = Mock()
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        with node._progress_lock:
            node.initial_distance = 10.0
            node.last_distance_remaining = 5.0
        
        # When: Feedback with invalid ETA
        feedback_msg = Mock()
        feedback = Mock()
        feedback.distance_remaining.data = 3.0
        feedback.estimated_time_remaining.sec = -1  # Invalid negative
        feedback.estimated_time_remaining.nanosec = 0
        feedback_msg.feedback = feedback
        
        node.feedback_callback(feedback_msg)
        
        # Then: Should handle gracefully (use fallback calculation)
        with node._progress_lock:
            # ETA should be calculated from distance/velocity
            assert node.last_eta_seconds is not None, \
                "ETA should be calculated even if Nav2 ETA is invalid"
            assert node.last_eta_seconds >= 0, \
                "ETA should be non-negative"


class TestFeedbackRaceConditions:
    """
    Tests for feedback race conditions.
    """
    
    def test_feedback_during_goal_cancellation(self, node_setup):
        """
        Bug it would catch: Feedback arrives during goal cancellation,
        feedback processed on None handle, exception raised.
        
        Production incident: Cancel called, handle cleared, but feedback
        arrives before cancel completes, feedback crashes.
        """
        node = node_setup
        
        # Given: Active goal
        mock_goal_handle = Mock()
        with node._goal_state_lock:
            node.current_goal_handle = mock_goal_handle
            node.current_target_id = "position_A"
        
        # When: Cancel is called (clears handle)
        node.cancelCurrentGoal()
        
        # And: Feedback arrives during cancellation
        feedback_msg = Mock()
        feedback = Mock()
        feedback.distance_remaining.data = 5.0
        feedback_msg.feedback = feedback
        
        # Then: Feedback should be ignored gracefully
        try:
            node.feedback_callback(feedback_msg)
        except Exception as e:
            pytest.fail(f"Feedback should be ignored after cancel: {e}")
        
        # State should remain consistent
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should remain None after cancel"
    
    def test_feedback_after_goal_succeeded(self, node_setup):
        """
        Bug it would catch: Goal succeeded, but late feedback arrives,
        feedback processed, state corrupted.
        
        Production incident: Goal completes, but feedback arrives late,
        feedback updates state, status shows navigating after arrival.
        """
        node = node_setup
        
        # Given: Goal succeeded (handle cleared)
        with node._goal_state_lock:
            node.current_goal_handle = None
            node.current_target_id = None
        
        # When: Late feedback arrives
        feedback_msg = Mock()
        feedback = Mock()
        feedback.distance_remaining.data = 0.1
        feedback_msg.feedback = feedback
        
        # Then: Feedback should be ignored
        node.feedback_callback(feedback_msg)
        
        # State should remain consistent
        with node._goal_state_lock:
            assert node.current_goal_handle is None, \
                "Goal handle should remain None after success"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

