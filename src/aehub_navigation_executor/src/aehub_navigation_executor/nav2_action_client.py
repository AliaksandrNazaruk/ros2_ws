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

"""Thin Nav2 NavigateToPose wrapper.

Design goals:
- Keep ROS action mechanics out of business node orchestration.
- Provide a small, testable interface (send_goal/cancel_goal callbacks).
- Maintain single-active-goal invariant.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2GoalOutcome(Enum):
    """Nav2 goal outcome enum."""
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"
    ERROR = "error"


@dataclass(frozen=True)
class Nav2GoalRef:
    target_id: str
    command_id: str


def goal_status_to_str(status: int) -> str:
    if status == GoalStatus.STATUS_SUCCEEDED:
        return "succeeded"
    if status == GoalStatus.STATUS_ABORTED:
        return "aborted"
    if status == GoalStatus.STATUS_CANCELED:
        return "canceled"
    return "unknown"


def goal_status_to_outcome(status: int) -> Nav2GoalOutcome:
    """Convert GoalStatus to Nav2GoalOutcome."""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return Nav2GoalOutcome.SUCCEEDED
    if status == GoalStatus.STATUS_CANCELED:
        return Nav2GoalOutcome.CANCELED
    if status == GoalStatus.STATUS_ABORTED:
        return Nav2GoalOutcome.ABORTED
    return Nav2GoalOutcome.ERROR


class Nav2ActionClient:
    """NavigateToPose action client with single-goal tracking."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/navigate_to_pose",
        server_wait_timeout_sec: float = 2.0,
        on_goal_response: Optional[Callable[[str, str, bool, Optional[str]], None]] = None,
        on_result: Optional[Callable[[str, str, Nav2GoalOutcome, Optional[str]], None]] = None,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._server_wait_timeout_sec = float(server_wait_timeout_sec)

        self._client = ActionClient(node, NavigateToPose, action_name)
        self._lock = threading.Lock()
        self._goal_handle = None
        self._current: Optional[Nav2GoalRef] = None
        self._cancel_requested = False

        # New-style callbacks (used by navigation_executor_node)
        self._on_goal_response: Optional[Callable[[str, str, bool, Optional[str]], None]] = on_goal_response
        self._on_result: Optional[Callable[[str, str, Nav2GoalOutcome, Optional[str]], None]] = on_result

        # Old-style callbacks (for backward compatibility)
        self.on_goal_accepted: Optional[Callable[[Nav2GoalRef], None]] = None
        self.on_goal_rejected: Optional[Callable[[Nav2GoalRef, str], None]] = None
        self.on_goal_result: Optional[Callable[[Nav2GoalRef, str], None]] = None

    def wait_for_server(self) -> bool:
        return bool(self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec))

    def has_active_goal(self) -> bool:
        with self._lock:
            return self._goal_handle is not None

    def get_current_command_id(self) -> Optional[str]:
        with self._lock:
            return self._current.command_id if self._current else None

    def get_current_target_id(self) -> Optional[str]:
        with self._lock:
            return self._current.target_id if self._current else None

    def send_goal(self, pose: PoseStamped, target_id: str, command_id: str) -> bool:
        """Send a new goal.

        Returns True if request was dispatched (accept/reject is async).
        Returns False if action server is not available.

        NOTE: Caller is responsible for preemption policy (cancel previous first).
        """
        if not self.wait_for_server():
            return False

        ref = Nav2GoalRef(target_id=str(target_id or "pose"), command_id=str(command_id))
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut: self._handle_goal_response(fut, ref))
        return True

    def cancel_goal(self) -> bool:
        """Request cancellation of the active goal.

        Returns:
            True if a cancel was requested or already in-flight.
            False if there is no active goal.
        """
        with self._lock:
            if self._goal_handle is None:
                return False
            if self._cancel_requested:
                return True
            self._cancel_requested = True
            goal_handle = self._goal_handle

        try:
            goal_handle.cancel_goal_async()
        except Exception as e:
            self._node.get_logger().warning(f"Cancel request failed: {e}")
        return True

    # ----------------- internals -----------------

    def _handle_goal_response(self, future, ref: Nav2GoalRef) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            # New-style callback
            if self._on_goal_response:
                self._on_goal_response(ref.command_id, ref.target_id, False, f"goal_response_exception: {e}")
            # Old-style callback
            if self.on_goal_rejected:
                self.on_goal_rejected(ref, f"goal_response_exception: {e}")
            return

        if not goal_handle or not getattr(goal_handle, "accepted", False):
            # New-style callback
            if self._on_goal_response:
                self._on_goal_response(ref.command_id, ref.target_id, False, "nav2_rejected")
            # Old-style callback
            if self.on_goal_rejected:
                self.on_goal_rejected(ref, "nav2_rejected")
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._current = ref
            self._cancel_requested = False

        # New-style callback
        if self._on_goal_response:
            self._on_goal_response(ref.command_id, ref.target_id, True, None)
        # Old-style callback
        if self.on_goal_accepted:
            self.on_goal_accepted(ref)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._handle_goal_result(fut, ref))

    def _handle_goal_result(self, future, ref: Nav2GoalRef) -> None:
        outcome: Nav2GoalOutcome = Nav2GoalOutcome.ERROR
        status_str = "unknown"
        message: Optional[str] = None
        
        try:
            res = future.result()
            status = int(getattr(res, "status", GoalStatus.STATUS_UNKNOWN))
            status_str = goal_status_to_str(status)
            outcome = goal_status_to_outcome(status)
        except Exception as e:
            status_str = f"error: {e}"
            message = str(e)
            outcome = Nav2GoalOutcome.ERROR

        with self._lock:
            # Only clear if this result corresponds to current goal.
            if self._current and self._current.command_id == ref.command_id:
                self._goal_handle = None
                self._current = None
                self._cancel_requested = False

        # New-style callback
        if self._on_result:
            self._on_result(ref.command_id, ref.target_id, outcome, message)
        # Old-style callback
        if self.on_goal_result:
            self.on_goal_result(ref, status_str)
