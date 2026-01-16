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
Navigation Action Client

Wrapper around Nav2 ActionClient for managing navigation goals.
Handles goal lifecycle: sending, cancellation, and callback management.

AE.HUB MVP: Abstracts Nav2 ActionClient complexity from NavigationIntegratedNode.

NOTE: This is a ROS2 component that requires a Node instance.
Thread-safe for concurrent access.
"""

from typing import Optional, Callable, Any
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import ClientGoalHandle


class NavigationActionClient:
    """
    Wrapper around Nav2 ActionClient for managing navigation goals.
    
    Provides simplified interface for:
    - Sending goals
    - Cancelling goals
    - Managing callbacks (response, feedback, result)
    """
    
    def __init__(self, node: Node, action_name: str = 'navigate_to_pose'):
        """
        Initialize navigation action client.
        
        Args:
            node: ROS2 Node instance (required for ActionClient)
            action_name: Name of the Nav2 action server (default: 'navigate_to_pose')
        """
        self._node = node
        self._action_name = action_name
        
        # Create Nav2 ActionClient
        self._action_client = ActionClient(node, NavigateToPose, action_name)
        
        # Thread-safe storage for current goal
        # Single lock protects goal state (handle, target_id, command_id)
        # All methods that access or modify goal state must acquire this lock
        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._current_target_id = None
        self._current_command_id = None
        self._cancel_requested = False
        
        # Callbacks (set by caller)
        self._goal_response_callback: Optional[Callable] = None
        self._feedback_callback: Optional[Callable] = None
        self._result_callback: Optional[Callable] = None
    
    def set_goal_response_callback(self, callback: Callable[[Any, Optional[str], Optional[str]], None]) -> None:
        """
        Set callback for goal response (accepted/rejected).
        
        Args:
            callback: Callable that receives (future, target_id, command_id)
        """
        self._goal_response_callback = callback
    
    def set_feedback_callback(self, callback: Callable[[Any], None]) -> None:
        """
        Set callback for goal feedback (progress updates).
        
        Args:
            callback: Callable that receives feedback message
        """
        self._feedback_callback = callback
    
    def set_result_callback(self, callback: Callable[[Any], None]) -> None:
        """
        Set callback for goal result (completed/cancelled/aborted).
        
        Args:
            callback: Callable that receives result future
        """
        self._result_callback = callback
    
    def send_goal(self, pose: PoseStamped, target_id: str, command_id: Optional[str] = None) -> bool:
        """
        Send navigation goal to Nav2.
        
        Args:
            pose: Target pose (PoseStamped)
            target_id: Target position ID
            command_id: Command ID for status correlation (optional)
        
        Returns:
            True if goal was sent successfully, False otherwise
        """
        # Ensure server is available (wait briefly if needed)
        try:
            # Try to wait up to 3 seconds for the server to be available
            if not self._action_client.server_is_ready():
                self._action_client.wait_for_server(timeout_sec=3.0)
        except Exception:
            pass
        if not self._action_client.server_is_ready():
            self._node.get_logger().error(
                f' Nav2 action server not available: {self._action_name}'
            )
            return False
        
        # Set timestamp
        pose.header.stamp = self._node.get_clock().now().to_msg()
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self._node.get_logger().info(
            f' Sending Nav2 goal: target_id={target_id}, '
            f'pose=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), '
            f'frame_id={pose.header.frame_id}, command_id={command_id}'
        )
        
        try:
            # Send goal with feedback callback
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )
            
            # Wrap goal response callback with target_id and command_id
            def goal_response_wrapper(future):
                if self._goal_response_callback:
                    self._goal_response_callback(future, target_id, command_id)
            
            send_goal_future.add_done_callback(goal_response_wrapper)
            
            self._node.get_logger().debug(' Goal sent to Nav2, callbacks registered')
            return True
            
        except Exception as e:
            self._node.get_logger().error(
                f' Failed to send Nav2 goal: {e}'
            )
            return False
    
    def cancel_goal(self) -> bool:
        """
        Cancel current navigation goal.
        
        Returns:
            True if cancel request was sent, False if no goal to cancel
        """
        with self._lock:
            goal_handle = self._current_goal_handle
            if goal_handle is None:
                self._node.get_logger().debug('No active goal to cancel')
                return False
            if self._cancel_requested:
                # Idempotent: a cancel is already in-flight.
                return True
            
            try:
                goal_handle.cancel_goal_async()
                self._node.get_logger().info(' Cancel request sent to Nav2')
                self._cancel_requested = True
                return True
                
            except Exception as e:
                self._node.get_logger().error(
                    f' Error cancelling goal: {e}'
                )
                # Keep state; caller may decide what to do.
                self._cancel_requested = True
                return False
    
    def is_goal_active(self) -> bool:
        """
        Check if there is an active goal.
        
        Returns:
            True if goal is active, False otherwise
        """
        with self._lock:
            return self._current_goal_handle is not None
    
    def set_current_goal(self, goal_handle: ClientGoalHandle, target_id: str, command_id: Optional[str] = None) -> None:
        """
        Set current goal handle (called from goal_response_callback when accepted).
        
        Args:
            goal_handle: Goal handle from Nav2
            target_id: Target position ID
            command_id: Command ID for status correlation (optional)
        """
        with self._lock:
            self._current_goal_handle = goal_handle
            self._current_target_id = target_id
            self._current_command_id = command_id
            self._cancel_requested = False
            
            # Register result callback
            if goal_handle and self._result_callback:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._result_callback)
    
    def clear_current_goal(self):
        """
        Clear current goal handle (called when goal completes/cancels/aborts).
        """
        with self._lock:
            self._current_goal_handle = None
            self._current_target_id = None
            self._current_command_id = None
            self._cancel_requested = False

    def is_cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested
    
    def get_current_target_id(self) -> Optional[str]:
        """
        Get current target ID.
        
        Returns:
            Current target ID, or None if no active goal
        """
        with self._lock:
            return self._current_target_id
    
    def get_current_command_id(self) -> Optional[str]:
        """
        Get current command ID.
        
        Returns:
            Current command ID, or None if no active goal
        """
        with self._lock:
            return self._current_command_id
    
    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to become available.
        
        Args:
            timeout_sec: Timeout in seconds
        
        Returns:
            True if server is available, False if timeout
        """
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)
    
    def server_is_ready(self) -> bool:
        """
        Check if Nav2 action server is ready.
        
        Returns:
            True if server is ready, False otherwise
        """
        return self._action_client.server_is_ready()

