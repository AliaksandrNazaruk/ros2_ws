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

"""Helper classes for integration testing."""

import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from aehub_msgs.msg import NavigationCommand, CommandAck, CommandResult, NavigationState


@dataclass
class EventRecord:
    """Recorded event from topics."""
    topic: str
    payload: dict
    timestamp: float


class ROSEventCollector:
    """Collects events from ROS topics for testing."""
    
    def __init__(self, node: Node):
        self._node = node
        self._events_lock = threading.Lock()
        self._events: List[EventRecord] = []
        
        # Subscriptions - use ROS message types (not JSON String)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        
        self._ack_sub = node.create_subscription(
            CommandAck,
            '/aehub/events/command_ack',
            lambda msg: self._on_ack(msg),
            qos
        )
        
        self._result_sub = node.create_subscription(
            CommandResult,
            '/aehub/events/command_result',
            lambda msg: self._on_result(msg),
            qos
        )
        
        self._state_sub = node.create_subscription(
            NavigationState,
            '/aehub/events/navigation_state',
            lambda msg: self._on_state(msg),
            qos
        )
    
    def _on_ack(self, msg: CommandAck):
        """Handle CommandAck message."""
        try:
            payload = {
                'command_id': msg.command_id,
                'status': msg.status,
                'ack_status': msg.status,  # Alias for compatibility
                'target_id': msg.target_id if msg.target_id else None,
                'reason': msg.reason if msg.reason else None,
                'stamp_sec': msg.stamp.sec,
                'stamp_nanosec': msg.stamp.nanosec,
            }
            with self._events_lock:
                self._events.append(EventRecord(
                    topic='ack',
                    payload=payload,
                    timestamp=time.time()
                ))
        except Exception as e:
            pass
    
    def _on_result(self, msg: CommandResult):
        """Handle CommandResult message."""
        try:
            payload = {
                'command_id': msg.command_id,
                'status': msg.status,
                'result_status': msg.status,  # Alias for compatibility
                'target_id': msg.target_id if msg.target_id else None,
                'reason': msg.reason if msg.reason else None,
                'stamp_sec': msg.stamp.sec,
                'stamp_nanosec': msg.stamp.nanosec,
            }
            with self._events_lock:
                self._events.append(EventRecord(
                    topic='result',
                    payload=payload,
                    timestamp=time.time()
                ))
        except Exception as e:
            pass
    
    def _on_state(self, msg: NavigationState):
        """Handle NavigationState message."""
        try:
            payload = {
                'public_state': msg.public_state,
                'state': msg.public_state,  # Alias for compatibility
                'active_command_id': msg.active_command_id if msg.active_command_id else None,
                'active_target_id': msg.active_target_id if msg.active_target_id else None,
                'internal_state': msg.internal_state if msg.internal_state else None,
                'stamp_sec': msg.stamp.sec,
                'stamp_nanosec': msg.stamp.nanosec,
            }
            with self._events_lock:
                self._events.append(EventRecord(
                    topic='state',
                    payload=payload,
                    timestamp=time.time()
                ))
        except Exception as e:
            pass
    
    def get_events(self) -> List[EventRecord]:
        """Get all collected events."""
        with self._events_lock:
            return list(self._events)
    
    def get_acks(self) -> List[dict]:
        """Get all ack events."""
        return [e.payload for e in self.get_events() if e.topic == 'ack']
    
    def get_results(self) -> List[dict]:
        """Get all result events."""
        return [e.payload for e in self.get_events() if e.topic == 'result']
    
    def get_states(self) -> List[dict]:
        """Get all state events."""
        return [e.payload for e in self.get_events() if e.topic == 'state']
    
    def clear(self):
        """Clear collected events."""
        with self._events_lock:
            self._events.clear()
    
    def wait_for_event(self, topic: str, timeout_sec: float = 2.0) -> Optional[dict]:
        """Wait for event on specific topic."""
        start = time.time()
        while time.time() - start < timeout_sec:
            events = [e.payload for e in self.get_events() if e.topic == topic]
            if events:
                return events[-1]  # Return last
            time.sleep(0.05)
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return None


class FakeNav2ActionServer(Node):
    """Fake Nav2 action server for testing."""
    
    def __init__(self, node_name: str = 'fake_nav2_server'):
        super().__init__(node_name)
        self._goal_handles: Dict[str, ServerGoalHandle] = {}
        self._lock = threading.Lock()
        
        # Behavior control
        self._should_accept_goals = True
        self._should_succeed_goals = True
        self._goal_delay_sec = 0.1
        
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback
        )
        self.get_logger().info('FakeNav2ActionServer started')
    
    def _goal_callback(self, goal_request):
        """Handle goal acceptance/rejection."""
        # Accept or reject based on flag
        if not self._should_accept_goals:
            self.get_logger().info('Rejecting goal')
            return GoalResponse.REJECT
        self.get_logger().info('Accepting goal')
        return GoalResponse.ACCEPT
    
    def set_should_accept_goals(self, accept: bool):
        """Control whether goals should be accepted."""
        self._should_accept_goals = accept
    
    def set_should_succeed_goals(self, succeed: bool):
        """Control whether goals should succeed."""
        self._should_succeed_goals = succeed
    
    def set_goal_delay_sec(self, delay: float):
        """Set delay before goal completion."""
        self._goal_delay_sec = delay
    
    def reject_next_goal(self):
        """Reject the next goal."""
        self._should_accept_goals = False
    
    def accept_next_goal(self):
        """Accept the next goal."""
        self._should_accept_goals = True
    
    def get_active_goals(self) -> List[str]:
        """Get list of active goal IDs."""
        with self._lock:
            return list(self._goal_handles.keys())
    
    def cancel_goal(self, goal_id: str = None):
        """Cancel a specific goal or all goals."""
        with self._lock:
            if goal_id:
                handle = self._goal_handles.get(goal_id)
                if handle:
                    handle.canceled()
                    del self._goal_handles[goal_id]
            else:
                for handle in self._goal_handles.values():
                    handle.canceled()
                self._goal_handles.clear()
    
    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Handle goal execution.
        
        In ROS2 Python, execute_callback should return result directly.
        The callback is called when goal is accepted.
        """
        goal = goal_handle.request
        goal_id = str(goal_handle.goal_id)
        
        self.get_logger().info(f'Goal executing: {goal_id}')
        
        # Accept/reject happens before execute callback
        # In execute callback, goal is already accepted
        with self._lock:
            self._goal_handles[goal_id] = goal_handle
        
        # Simulate navigation delay (blocking in execute_callback)
        # For async execution, we use threading
        import threading
        import queue
        result_queue = queue.Queue()
        
        def complete_goal():
            time.sleep(self._goal_delay_sec)
            with self._lock:
                if goal_id not in self._goal_handles:
                    # Goal was already cancelled/removed
                    result_queue.put(None)
                    return
                
                # Check if goal is still valid before completing
                try:
                    # Check status
                    status = goal_handle.status
                    if status != GoalStatus.STATUS_EXECUTING:
                        # Goal was cancelled or aborted externally
                        self.get_logger().info(f'Goal {goal_id} already completed with status {status}')
                        if goal_id in self._goal_handles:
                            del self._goal_handles[goal_id]
                        result_queue.put(None)
                        return
                except Exception as e:
                    # Goal handle is invalid
                    self.get_logger().warn(f'Goal handle invalid for {goal_id}: {e}')
                    if goal_id in self._goal_handles:
                        del self._goal_handles[goal_id]
                    result_queue.put(None)
                    return
                
                # Complete goal and store result
                try:
                    result = NavigateToPose.Result()
                    # Set error_code to 0 for success
                    result.error_code = 0
                    if self._should_succeed_goals:
                        goal_handle.succeed()  # succeed() doesn't take result argument
                        self.get_logger().info(f'Goal succeeded: {goal_id}')
                        result_queue.put(result)
                    else:
                        result.error_code = 1
                        goal_handle.abort()  # abort() doesn't take result argument
                        self.get_logger().info(f'Goal aborted: {goal_id}')
                        result_queue.put(result)
                except Exception as e:
                    self.get_logger().warn(f'Failed to complete goal {goal_id}: {e}')
                    result_queue.put(None)
                finally:
                    if goal_id in self._goal_handles:
                        del self._goal_handles[goal_id]
        
        thread = threading.Thread(target=complete_goal, daemon=True)
        thread.start()
        
        # Wait for result (with timeout)
        try:
            result = result_queue.get(timeout=self._goal_delay_sec + 1.0)
            if result is None:
                # Goal was cancelled or failed
                result = NavigateToPose.Result()
                result.error_code = 1
            return result
        except queue.Empty:
            # Timeout - goal should be handled by thread
            result = NavigateToPose.Result()
            result.error_code = 1
            return result


class CommandPublisher:
    """Helper to publish commands to executor node using NavigationCommand ROS messages."""
    
    def __init__(self, node: Node):
        self._node = node
        # Use unified command topic with NavigationCommand ROS message
        self._cmd_pub = node.create_publisher(
            NavigationCommand,
            '/aehub/commands/navigation',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
    
    def publish_navigate(self, command_id: str, x: float = 1.0, y: float = 2.0, theta: float = 0.0, target_id: str = None):
        """Publish navigate command as NavigationCommand ROS message."""
        msg = NavigationCommand()
        msg.command_id = command_id
        msg.type = "navigateTo"
        msg.stamp = self._node.get_clock().now().to_msg()
        
        if target_id:
            msg.target_id = target_id
        else:
            # Use x, y, theta fields directly (NavigationCommand has these fields)
            msg.x = float(x)
            msg.y = float(y)
            msg.theta = float(theta)
        
        self._cmd_pub.publish(msg)
        time.sleep(0.01)  # Small delay for processing
    
    def publish_cancel(self, command_id: str):
        """Publish cancel command as NavigationCommand ROS message."""
        msg = NavigationCommand()
        msg.command_id = command_id
        msg.type = "cancel"
        msg.stamp = self._node.get_clock().now().to_msg()
        
        self._cmd_pub.publish(msg)
        time.sleep(0.01)  # Small delay for processing
