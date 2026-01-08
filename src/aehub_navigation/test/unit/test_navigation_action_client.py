#!/usr/bin/env python3

"""
Unit tests for NavigationActionClient

Tests navigation goal lifecycle management, callbacks, and error handling.
"""

import pytest
from unittest.mock import Mock, MagicMock, patch, call
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import ClientGoalHandle

from aehub_navigation.navigation_action_client import NavigationActionClient


@pytest.fixture
def rclpy_context():
    """Initialize ROS2 context for tests"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def mock_node(rclpy_context):
    """Create a mock ROS2 node"""
    node = Node('test_node')
    yield node
    node.destroy_node()


@pytest.fixture
def nav_action_client(mock_node):
    """Create a NavigationActionClient instance"""
    with patch('aehub_navigation.navigation_action_client.ActionClient'):
        client = NavigationActionClient(mock_node, action_name='navigate_to_pose')
        # Mock the internal action client
        client._action_client = MagicMock()
        yield client


class TestNavigationActionClient:
    """Test suite for NavigationActionClient"""
    
    def test_initialization(self, mock_node):
        """Test that NavigationActionClient initializes correctly"""
        with patch('aehub_navigation.navigation_action_client.ActionClient') as mock_action_client_class:
            client = NavigationActionClient(mock_node, action_name='test_action')
            
            assert client._node == mock_node
            assert client._action_name == 'test_action'
            assert client._current_goal_handle is None
            assert client._current_target_id is None
            assert client._current_command_id is None
            assert client._goal_response_callback is None
            assert client._feedback_callback is None
            assert client._result_callback is None
            mock_action_client_class.assert_called_once_with(mock_node, NavigateToPose, 'test_action')
    
    def test_set_callbacks(self, nav_action_client):
        """Test setting callbacks"""
        def goal_response_cb(future, target_id, command_id):
            pass
        
        def feedback_cb(feedback):
            pass
        
        def result_cb(future):
            pass
        
        nav_action_client.set_goal_response_callback(goal_response_cb)
        nav_action_client.set_feedback_callback(feedback_cb)
        nav_action_client.set_result_callback(result_cb)
        
        assert nav_action_client._goal_response_callback == goal_response_cb
        assert nav_action_client._feedback_callback == feedback_cb
        assert nav_action_client._result_callback == result_cb
    
    def test_send_goal_server_not_ready(self, nav_action_client, mock_node):
        """Test send_goal when server is not ready"""
        nav_action_client._action_client.server_is_ready.return_value = False
        nav_action_client._action_client.wait_for_server.return_value = False
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        
        result = nav_action_client.send_goal(pose, 'target_A', 'cmd_123')
        
        assert result is False
        nav_action_client._action_client.server_is_ready.assert_called()
    
    def test_send_goal_success(self, nav_action_client, mock_node):
        """Test successful goal sending"""
        nav_action_client._action_client.server_is_ready.return_value = True
        
        # Mock clock
        mock_clock = MagicMock()
        mock_time = MagicMock()
        mock_time.to_msg.return_value = MagicMock()
        mock_clock.now.return_value = mock_time
        mock_node.get_clock = Mock(return_value=mock_clock)
        
        # Mock send_goal_async
        mock_future = MagicMock()
        mock_future.add_done_callback = Mock()
        nav_action_client._action_client.send_goal_async.return_value = mock_future
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        
        result = nav_action_client.send_goal(pose, 'target_A', 'cmd_123')
        
        assert result is True
        nav_action_client._action_client.send_goal_async.assert_called_once()
        mock_future.add_done_callback.assert_called_once()
    
    def test_send_goal_exception(self, nav_action_client, mock_node):
        """Test send_goal when exception occurs"""
        nav_action_client._action_client.server_is_ready.return_value = True
        
        # Mock clock
        mock_clock = MagicMock()
        mock_time = MagicMock()
        mock_time.to_msg.return_value = MagicMock()
        mock_clock.now.return_value = mock_time
        mock_node.get_clock = Mock(return_value=mock_clock)
        
        # Mock send_goal_async to raise exception
        nav_action_client._action_client.send_goal_async.side_effect = Exception("Connection error")
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        
        result = nav_action_client.send_goal(pose, 'target_A', 'cmd_123')
        
        assert result is False
    
    def test_cancel_goal_no_active_goal(self, nav_action_client):
        """Test cancel_goal when no active goal"""
        nav_action_client._current_goal_handle = None
        
        result = nav_action_client.cancel_goal()
        
        assert result is False
    
    def test_cancel_goal_success(self, nav_action_client):
        """Test successful goal cancellation"""
        mock_goal_handle = MagicMock()
        mock_cancel_future = MagicMock()
        mock_goal_handle.cancel_goal_async.return_value = mock_cancel_future
        nav_action_client._current_goal_handle = mock_goal_handle
        
        result = nav_action_client.cancel_goal()
        
        assert result is True
        mock_goal_handle.cancel_goal_async.assert_called_once()
        assert nav_action_client._current_goal_handle is None
        assert nav_action_client._current_target_id is None
        assert nav_action_client._current_command_id is None
    
    def test_cancel_goal_exception(self, nav_action_client):
        """Test cancel_goal when exception occurs"""
        mock_goal_handle = MagicMock()
        mock_goal_handle.cancel_goal_async.side_effect = Exception("Cancel error")
        nav_action_client._current_goal_handle = mock_goal_handle
        
        result = nav_action_client.cancel_goal()
        
        assert result is False
        # State should still be cleared
        assert nav_action_client._current_goal_handle is None
        assert nav_action_client._current_target_id is None
        assert nav_action_client._current_command_id is None
    
    def test_is_goal_active_no_goal(self, nav_action_client):
        """Test is_goal_active when no goal is active"""
        nav_action_client._current_goal_handle = None
        
        assert nav_action_client.is_goal_active() is False
    
    def test_is_goal_active_with_goal(self, nav_action_client):
        """Test is_goal_active when goal is active"""
        mock_goal_handle = MagicMock()
        nav_action_client._current_goal_handle = mock_goal_handle
        
        assert nav_action_client.is_goal_active() is True
    
    def test_set_current_goal(self, nav_action_client):
        """Test set_current_goal"""
        mock_goal_handle = MagicMock()
        mock_result_future = MagicMock()
        mock_result_future.add_done_callback = Mock()
        mock_goal_handle.get_result_async.return_value = mock_result_future
        
        nav_action_client._result_callback = Mock()
        
        nav_action_client.set_current_goal(mock_goal_handle, 'target_B', 'cmd_456')
        
        assert nav_action_client._current_goal_handle == mock_goal_handle
        assert nav_action_client._current_target_id == 'target_B'
        assert nav_action_client._current_command_id == 'cmd_456'
        mock_goal_handle.get_result_async.assert_called_once()
        mock_result_future.add_done_callback.assert_called_once_with(nav_action_client._result_callback)
    
    def test_set_current_goal_no_result_callback(self, nav_action_client):
        """Test set_current_goal when result callback is not set"""
        mock_goal_handle = MagicMock()
        nav_action_client._result_callback = None
        
        nav_action_client.set_current_goal(mock_goal_handle, 'target_B', 'cmd_456')
        
        assert nav_action_client._current_goal_handle == mock_goal_handle
        assert nav_action_client._current_target_id == 'target_B'
        assert nav_action_client._current_command_id == 'cmd_456'
        # get_result_async should not be called if no callback
        mock_goal_handle.get_result_async.assert_not_called()
    
    def test_clear_current_goal(self, nav_action_client):
        """Test clear_current_goal"""
        mock_goal_handle = MagicMock()
        nav_action_client._current_goal_handle = mock_goal_handle
        nav_action_client._current_target_id = 'target_C'
        nav_action_client._current_command_id = 'cmd_789'
        
        nav_action_client.clear_current_goal()
        
        assert nav_action_client._current_goal_handle is None
        assert nav_action_client._current_target_id is None
        assert nav_action_client._current_command_id is None
    
    def test_get_current_target_id(self, nav_action_client):
        """Test get_current_target_id"""
        nav_action_client._current_target_id = 'target_D'
        
        assert nav_action_client.get_current_target_id() == 'target_D'
    
    def test_get_current_target_id_none(self, nav_action_client):
        """Test get_current_target_id when no goal"""
        nav_action_client._current_target_id = None
        
        assert nav_action_client.get_current_target_id() is None
    
    def test_get_current_command_id(self, nav_action_client):
        """Test get_current_command_id"""
        nav_action_client._current_command_id = 'cmd_999'
        
        assert nav_action_client.get_current_command_id() == 'cmd_999'
    
    def test_get_current_command_id_none(self, nav_action_client):
        """Test get_current_command_id when no goal"""
        nav_action_client._current_command_id = None
        
        assert nav_action_client.get_current_command_id() is None
    
    def test_wait_for_server(self, nav_action_client):
        """Test wait_for_server"""
        nav_action_client._action_client.wait_for_server.return_value = True
        
        result = nav_action_client.wait_for_server(timeout_sec=5.0)
        
        assert result is True
        # wait_for_server uses keyword argument timeout_sec
        nav_action_client._action_client.wait_for_server.assert_called_once_with(timeout_sec=5.0)
    
    def test_wait_for_server_timeout(self, nav_action_client):
        """Test wait_for_server timeout"""
        nav_action_client._action_client.wait_for_server.return_value = False
        
        result = nav_action_client.wait_for_server(timeout_sec=1.0)
        
        assert result is False
    
    def test_server_is_ready(self, nav_action_client):
        """Test server_is_ready"""
        nav_action_client._action_client.server_is_ready.return_value = True
        
        assert nav_action_client.server_is_ready() is True
        nav_action_client._action_client.server_is_ready.assert_called_once()
    
    def test_thread_safety_set_and_get(self, nav_action_client):
        """Test thread safety of set/get operations"""
        import threading
        
        results = []
        
        def set_goal(goal_id):
            mock_handle = MagicMock()
            nav_action_client.set_current_goal(mock_handle, f'target_{goal_id}', f'cmd_{goal_id}')
            target_id = nav_action_client.get_current_target_id()
            command_id = nav_action_client.get_current_command_id()
            results.append((goal_id, target_id, command_id))
        
        # Run multiple threads setting goals
        threads = []
        for i in range(10):
            t = threading.Thread(target=set_goal, args=(i,))
            threads.append(t)
            t.start()
        
        for t in threads:
            t.join()
        
        # All operations should complete without errors
        assert len(results) == 10
        # Verify thread safety - no corruption
        for goal_id, target_id, command_id in results:
            assert target_id == f'target_{goal_id}'
            assert command_id == f'cmd_{goal_id}'

