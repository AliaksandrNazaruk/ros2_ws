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

"""Business-layer navigation command executor (Application Layer, Step 1).

Step 1: Transport → Executor integration (without Nav2).

Responsibilities:
- Subscribe to inbound commands (transport-agnostic ROS topics):
  - /aehub/commands/navigation (NavigationCommand ROS message)
- Enforce validation, idempotency/deduplication, and (navigateTo) rate limiting.
- Publish ROS events (ack/result/state) - transport layer subscribes and maps to MQTT.
- FSM transitions: IDLE → ACCEPTED → (CANCELED | ACCEPTED_DONE)

Architecture invariants (Step 1):
- Executor does NOT know about MQTT
- Executor does NOT know about JSON
- Executor does NOT know about Nav2 (Step 2)
- Executor only knows: Command → FSM → Event (ROS messages)
- Executor publishes ROS messages, transport decides what to do with them

This node is a LifecycleNode:
  configure -> allocate resources
  activate  -> start processing commands
  deactivate/cleanup -> stop and release resources
"""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

# Add package directory to path for direct script execution
if __name__ == "__main__" or "__file__" in globals():
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _package_dir = os.path.dirname(_script_dir)
    if _package_dir not in sys.path:
        sys.path.insert(0, _package_dir)

# Step 2: geometry_msgs imports will be added here (PoseStamped, Twist)
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from aehub_msgs.msg import NavigationCommand
from aehub_msgs.action import NavigationExecute

try:
    from .command_state_machine import CommandStateMachine
    from .command_validator import CommandValidator, ValidationResult
    from .event_publisher import EventPublisher
    from .executor_fsm import NavigationExecutorFSM
    # Step 2: Nav2ActionClient will be imported here
except ImportError:
    # Fallback for direct script execution
    import aehub_navigation_executor
    from aehub_navigation_executor.command_state_machine import CommandStateMachine
    from aehub_navigation_executor.command_validator import CommandValidator, ValidationResult
    from aehub_navigation_executor.event_publisher import EventPublisher
    from aehub_navigation_executor.executor_fsm import NavigationExecutorFSM
    # Step 2: Nav2ActionClient will be imported here


@dataclass(frozen=True)
class _QueuedCommand:
    """Internal command queue item - transport-agnostic."""
    command: NavigationCommand  # Transport-provided command envelope
    received_ts_s: float


class NavigationExecutorNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("aehub_navigation_executor")

        # --- Parameters (topics) ---
        # Transport-facing: single command topic (transport-agnostic)
        self.declare_parameter("cmd_topic", "commands/navigation")
        # Single event stream (rule-of-1-output-topic)
        self.declare_parameter("event_topic", "events/navigation")

        # Legacy parameters (deprecated, ignored; kept so old launch configs don't break)
        self.declare_parameter("cmd_navigate_topic", "commands/navigate_to")
        self.declare_parameter("cmd_cancel_topic", "commands/cancel")
        self.declare_parameter("event_ack_topic", "events/command_ack")
        self.declare_parameter("event_result_topic", "events/command_result")
        self.declare_parameter("event_state_topic", "events/navigation_state")

        # Capability API (action client)
        self.declare_parameter("capability_action_name", "capabilities/navigation/execute")

        # --- Parameters (policy) ---
        self.declare_parameter("rate_limit_sec", 1.0)
        self.declare_parameter("command_cache_ttl_sec", 600.0)

        # --- Runtime state ---
        self._active = False
        self._configured = False
        self._queue_lock = threading.Lock()
        # Bounded queue to prevent memory exhaustion during input flood
        # maxlen=100: drop oldest if queue overflows (DoS protection)
        self._queue: deque[_QueuedCommand] = deque(maxlen=100)

        self._validator: Optional[CommandValidator] = None
        self._fsm: Optional[CommandStateMachine] = None
        self._exec_fsm = NavigationExecutorFSM()
        self._events: Optional[EventPublisher] = None
        self._capability_client: Optional[ActionClient] = None
        self._active_goal_handle = None

        self._active_command_id: Optional[str] = None
        self._active_target_id: Optional[str] = None
        self._terminal_published: set[str] = set()

        # Publishers/subscriptions/timers created on configure
        self._cmd_sub = None  # Single command subscription
        # Step 2: cmd_vel_pub and stop_burst logic will be added here

        self._process_timer = None
        self._cleanup_timer = None

    # -------------------- Lifecycle --------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Policy
        rate_limit = float(self.get_parameter("rate_limit_sec").value)
        ttl = float(self.get_parameter("command_cache_ttl_sec").value)

        # Transport-facing topics (single command envelope)
        cmd_topic = str(self.get_parameter("cmd_topic").value)

        # Unified event stream (single output)
        event_topic = str(self.get_parameter("event_topic").value)

        # Capability API
        capability_action_name = str(self.get_parameter("capability_action_name").value)

        # Core components
        self._validator = CommandValidator(cache_ttl_sec=ttl)
        self._rate_limit_sec = rate_limit  # Store for validate_command calls
        self._fsm = CommandStateMachine()
        self._events = EventPublisher(self, event_topic=event_topic)
        self._configured = True  # Mark as configured

        # Single subscription to unified command topic (transport-agnostic)
        # Executor does not know about MQTT - only receives NavigationCommand messages
        # Command QoS: RELIABLE (commands cannot be lost)
        from rclpy.qos import ReliabilityPolicy, HistoryPolicy
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cmd_sub = self.create_subscription(
            NavigationCommand,
            cmd_topic,
            self._on_command,
            cmd_qos
        )

        self._capability_client = ActionClient(self, NavigationExecute, capability_action_name)

        self.get_logger().info(
            "Configured: "
            f"cmd_topic={cmd_topic}, event_topic={event_topic}, capability_action_name={capability_action_name}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Check if configured (Step 1: Application Layer only)
        if not self._validator or not self._fsm or not self._events:
            self.get_logger().error("Cannot activate: not configured")
            return TransitionCallbackReturn.FAILURE
        
        # Note: In rclpy, there is NO LifecyclePublisher.
        # Publishers created via create_publisher() are always active once created.
        # They work as long as:
        #   - Node is spinned (executor running)
        #   - QoS is compatible
        # There is NO need for explicit activation/deactivation in EventPublisher.
        
        self._active = True
        self._exec_fsm.set_ready()

        # Processing timer
        self._process_timer = self.create_timer(0.05, self._process_queue)  # 20 Hz
        self._cleanup_timer = self.create_timer(5.0, self._cleanup_cache)

        # Publish initial state
        self._publish_state_snapshot()
        self.get_logger().info("Activated (Step 1: Application Layer)")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        if self._process_timer:
            self._process_timer.cancel()
            self._process_timer = None
        if self._cleanup_timer:
            self._cleanup_timer.cancel()
            self._cleanup_timer = None
        # Note: In rclpy, there is NO LifecyclePublisher.
        # Publishers do NOT need deactivation - they are always active once created.
        # Step 2: stop_burst_stop() and Nav2 cancel will be added here
        self.get_logger().info("Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        with self._queue_lock:
            self._queue.clear()
        self._validator = None
        self._fsm = None
        self._events = None
        # Step 2: _nav2 cleanup will be added here
        self._active_command_id = None
        self._active_target_id = None
        self._terminal_published.clear()
        self._configured = False  # Mark as not configured
        self._exec_fsm.set_ready()
        self.get_logger().info("Cleaned up")
        return TransitionCallbackReturn.SUCCESS

    # -------------------- ROS callbacks (entry point from transport) --------------------
    def _on_command(self, msg: NavigationCommand) -> None:
        """
        Entry point from transport layer.
        
        Step 1 invariant:
        - Executor does NOT know about MQTT
        - Executor does NOT call Nav2 during command handling
        - Only validation, deduplication, FSM, and event publishing
        
        Args:
            msg: NavigationCommand envelope from transport
        """
        if not self._active:
            self.get_logger().debug("Received command but node is not active")
            return
        
        self.get_logger().info(f"Received command: {msg.command_id}, type: {msg.type}")
        
        # 1. Validate command
        ok, reason = self._validator.validate_command(msg)
        if not ok:
            self._publish_rejected(msg, reason)
            return
        
        # 2. Deduplicate
        if self._validator.is_duplicate(msg.command_id):
            self._replay_outcome(msg.command_id)
            return

        # 2b. Idempotency for in-flight commands:
        # MQTT/transport may redeliver the *same* command_id while the action is still executing.
        # Treat it as a no-op and re-publish current state, instead of rejecting as "busy".
        if (
            self._active_command_id
            and msg.command_id == self._active_command_id
            and msg.command_id not in self._terminal_published
        ):
            if self._events:
                # Re-ack as received (idempotent) and publish a fresh state snapshot.
                self._events.publish_ack(
                    msg.command_id,
                    target_id=self._active_target_id,
                    ack_status="received",
                    reason="",
                )
            self._publish_state_snapshot()
            return
        
        # 3. Enqueue for processing (async, no Nav2 calls here)
        self._enqueue(msg)

    def _enqueue(self, command: NavigationCommand) -> None:
        """Enqueue command for async processing."""
        if not self._active:
            return
        with self._queue_lock:
            was_full = len(self._queue) >= self._queue.maxlen if hasattr(self._queue, 'maxlen') and self._queue.maxlen else False
            self._queue.append(_QueuedCommand(command=command, received_ts_s=time.time()))
            if was_full:
                self.get_logger().warn(f"Command queue overflow: dropped oldest command (DoS protection)")

    # -------------------- Processing loop --------------------
    def _process_queue(self) -> None:
        """
        Process queued commands.
        
        Queue = boundary between transport and application logic.
        Step 1: Commands are validated and events published, but Nav2 is NOT called.
        """
        if not self._active:
            return
        item: Optional[_QueuedCommand] = None
        with self._queue_lock:
            if self._queue:
                # deque.popleft() is O(1), list.pop(0) is O(n)
                item = self._queue.popleft()
        if not item:
            return

        cmd = item.command
        self.get_logger().info(f"Processing queued command: {cmd.command_id}, type: {cmd.type}")
        if cmd.type == "navigateTo":
            self._handle_navigate_request(cmd)
        elif cmd.type == "cancel":
            self._handle_cancel_request(cmd)
        else:
            self.get_logger().warn(f"Unknown command type: {cmd.type}")
            self._publish_rejected(cmd, f"invalid_command_type: {cmd.type}")

    def _publish_rejected(self, cmd: NavigationCommand, reason: str) -> None:
        """
        Publish rejection event for invalid command.
        
        Args:
            cmd: NavigationCommand that was rejected
            reason: Rejection reason
        """
        if not self._events:
            return

        # Rejection is still a (short) execution of this command.
        self._exec_fsm.start(cmd.command_id)
        
        target_id = cmd.target_id if cmd.target_id else None
        self._events.publish_ack(
            cmd.command_id,
            target_id=target_id,
            ack_status="rejected",
            reason=reason
        )
        self._events.publish_result(
            cmd.command_id,
            target_id=target_id,
            result_status="error",
            reason=reason
        )
        # Store outcome for deduplication
        if self._validator:
            self._validator.store_outcome(
                cmd.command_id,
                ack_data={"ack_status": "rejected", "reason": reason, "target_id": target_id},
                result_data={"result_status": "error", "reason": reason, "target_id": target_id},
            )
        self._terminal_published.add(cmd.command_id)
        # Rejections are terminal immediately.
        self._exec_fsm.mark_done()
        self._publish_state_snapshot()

    def _handle_navigate_request(self, cmd: NavigationCommand) -> None:
        """
        Handle navigateTo request by calling the Navigation Capability action.
        
        Args:
            cmd: NavigationCommand with type="navigateTo"
        """
        if not self._validator or not self._events or not self._fsm or not self._capability_client:
            self.get_logger().error(f"Cannot handle navigate request: validator={self._validator is not None}, events={self._events is not None}, fsm={self._fsm is not None}")
            return

        # One active navigation at a time (simplest, deterministic policy).
        if (
            self._active_command_id
            and self._active_command_id not in self._terminal_published
            and cmd.command_id != self._active_command_id
        ):
            self._publish_rejected(cmd, "busy: active_navigation_in_progress")
            return
        
        # Note: Validation already done in _on_command() before enqueueing.
        # No need to re-validate here, especially rate limiting which would fail
        # if called too quickly after previous command. Only basic checks remain.
        
        # Get target_id
        target_id = cmd.target_id if cmd.target_id else "pose"
        
        # Update state machine
        self._fsm.start_navigation(cmd.command_id, target_id=target_id)
        self._active_command_id = cmd.command_id
        self._active_target_id = target_id
        self._exec_fsm.start(cmd.command_id)

        # Edge confirmation: received
        self._events.publish_ack(cmd.command_id, target_id=target_id, ack_status="received")

        goal = NavigationExecute.Goal()
        goal.command_id = cmd.command_id
        goal.target_id = cmd.target_id  # metadata (may be empty)
        goal.x = float(cmd.x)
        goal.y = float(cmd.y)
        goal.theta = float(cmd.theta)

        send_future = self._capability_client.send_goal_async(
            goal,
            feedback_callback=self._on_capability_feedback,
        )
        send_future.add_done_callback(self._on_capability_goal_response)

        self._publish_state_snapshot()

    def _on_capability_feedback(self, feedback_msg) -> None:
        # Hook for future: progress/ETA bridging into NavigationEvent if needed.
        del feedback_msg

    def _on_capability_goal_response(self, future) -> None:
        if not self._events or not self._validator:
            return

        cmd_id = self._active_command_id or ""
        tgt_id = self._active_target_id

        try:
            goal_handle = future.result()
        except Exception as e:
            if cmd_id:
                self._events.publish_ack(cmd_id, target_id=tgt_id, ack_status="rejected", reason=f"capability_unavailable: {e}")
                self._events.publish_result(cmd_id, target_id=tgt_id, result_status="error", reason="capability_unavailable")
                self._validator.store_outcome(
                    cmd_id,
                    ack_data={"ack_status": "rejected", "reason": "capability_unavailable", "target_id": tgt_id},
                    result_data={"result_status": "error", "reason": "capability_unavailable", "target_id": tgt_id},
                )
                self._terminal_published.add(cmd_id)
            self._active_goal_handle = None
            self._active_command_id = None
            self._active_target_id = None
            self._exec_fsm.mark_done()
            self._publish_state_snapshot()
            return

        if not goal_handle.accepted:
            if cmd_id:
                self._events.publish_ack(cmd_id, target_id=tgt_id, ack_status="rejected", reason="capability_rejected_goal")
                self._events.publish_result(cmd_id, target_id=tgt_id, result_status="error", reason="capability_rejected_goal")
                self._validator.store_outcome(
                    cmd_id,
                    ack_data={"ack_status": "rejected", "reason": "capability_rejected_goal", "target_id": tgt_id},
                    result_data={"result_status": "error", "reason": "capability_rejected_goal", "target_id": tgt_id},
                )
                self._terminal_published.add(cmd_id)
            self._active_goal_handle = None
            self._active_command_id = None
            self._active_target_id = None
            self._exec_fsm.mark_done()
            self._publish_state_snapshot()
            return

        # Accepted
        self._active_goal_handle = goal_handle
        if cmd_id:
            self._events.publish_ack(cmd_id, target_id=tgt_id, ack_status="accepted")
            self._validator.store_outcome(cmd_id, ack_data={"ack_status": "accepted", "target_id": tgt_id}, result_data=None)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_capability_result)
        self._publish_state_snapshot()

    def _on_capability_result(self, future) -> None:
        if not self._events or not self._validator:
            return

        cmd_id = self._active_command_id or ""
        tgt_id = self._active_target_id

        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as e:
            if cmd_id:
                self._events.publish_result(cmd_id, target_id=tgt_id, result_status="error", reason=f"capability_result_error: {e}")
                self._validator.store_outcome(
                    cmd_id,
                    result_data={"result_status": "error", "reason": "capability_result_error", "target_id": tgt_id},
                )
                self._terminal_published.add(cmd_id)
            self._active_goal_handle = None
            self._active_command_id = None
            self._active_target_id = None
            self._exec_fsm.mark_done()
            self._publish_state_snapshot()
            return

        if cmd_id:
            status = str(getattr(result, "status", "") or "error")
            reason = str(getattr(result, "reason", "") or "")
            self._events.publish_result(cmd_id, target_id=tgt_id, result_status=status, reason=reason)
            self._validator.store_outcome(
                cmd_id,
                result_data={"result_status": status, "reason": reason, "target_id": tgt_id},
            )
            self._terminal_published.add(cmd_id)

        self._active_goal_handle = None
        self._active_command_id = None
        self._active_target_id = None
        self._exec_fsm.mark_done()
        self._publish_state_snapshot()

    def _handle_cancel_request(self, cmd: NavigationCommand) -> None:
        """
        Handle cancel request by canceling the active capability goal (if any).
        
        Args:
            cmd: NavigationCommand with type="cancel"
        """
        if not self._validator or not self._events or not self._fsm:
            return

        # Any cancel command is an execution episode, even if there is no active goal.
        self._exec_fsm.start(cmd.command_id)

        # Edge confirmation: received
        self._events.publish_ack(cmd.command_id, ack_status="received")

        if self._active_goal_handle is not None:
            try:
                cancel_future = self._active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self._on_capability_cancel_done(cmd.command_id, f))
            except Exception:
                # Best-effort: still report cancel command succeeded.
                self._events.publish_ack(cmd.command_id, ack_status="accepted")
                self._events.publish_result(cmd.command_id, result_status="succeeded")
                self._validator.store_outcome(cmd.command_id, ack_data={"ack_status": "accepted"}, result_data={"result_status": "succeeded"})
                self._terminal_published.add(cmd.command_id)
                self._exec_fsm.mark_done()
        else:
            # No active goal: cancel is a no-op, but succeeds.
            self._events.publish_ack(cmd.command_id, ack_status="accepted")
            self._events.publish_result(cmd.command_id, result_status="succeeded")
            self._validator.store_outcome(cmd.command_id, ack_data={"ack_status": "accepted"}, result_data={"result_status": "succeeded"})
            self._terminal_published.add(cmd.command_id)
            self._exec_fsm.mark_done()
        
        # Publish state snapshot
        self._publish_state_snapshot()

    def _on_capability_cancel_done(self, cancel_command_id: str, future) -> None:
        if not self._events or not self._validator:
            return
        try:
            _ = future.result()
        except Exception:
            # Still treat cancel as accepted+succeeded (best-effort request).
            pass
        self._events.publish_ack(cancel_command_id, ack_status="accepted")
        self._events.publish_result(cancel_command_id, result_status="succeeded")
        self._validator.store_outcome(
            cancel_command_id,
            ack_data={"ack_status": "accepted"},
            result_data={"result_status": "succeeded"},
        )
        self._terminal_published.add(cancel_command_id)
        self._exec_fsm.mark_done()
        self._publish_state_snapshot()

    def _replay_outcome(self, command_id: str) -> None:
        """Replay cached ACK and RESULT for duplicate command, then publish state snapshot."""
        if not command_id or not self._validator or not self._events:
            return
        oc = self._validator.get_outcome(command_id)
        if not oc:
            return
        if oc.ack_data:
            self._events.publish_ack(
                command_id,
                target_id=oc.ack_data.get("target_id"),
                ack_status=str(oc.ack_data.get("ack_status") or ""),
                reason=oc.ack_data.get("reason"),
            )
        if oc.result_data:
            self._events.publish_result(
                command_id,
                target_id=oc.result_data.get("target_id"),
                result_status=str(oc.result_data.get("result_status") or ""),
                reason=oc.result_data.get("reason"),
            )
        # Always publish state snapshot after replay for consistency
        # UI/orchestration layers rely on /events/state for current state
        self._publish_state_snapshot()

    def _cleanup_cache(self) -> None:
        if self._validator:
            self._validator.cleanup_expired()

    def _publish_state_snapshot(self) -> None:
        if not self._events or not self._fsm:
            return
        cmd_snap = self._fsm.snapshot()
        exec_snap = self._exec_fsm.snapshot()
        self._events.publish_state(
            # New public FSM: READY/EXECUTING/DONE (as strings)
            public_state=exec_snap.public_state,
            # Keep command-level state for debugging/compatibility
            active_command_id=cmd_snap.command_id,
            active_target_id=cmd_snap.target_id,
            internal_state=cmd_snap.internal_state,
        )

    # Step 2: stop_burst logic will be added here (_stop_burst_start, _stop_burst_stop, _stop_tick)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = NavigationExecutorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
