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
Event publisher abstraction (ROS-only, transport-agnostic).

Publishes ROS events to a SINGLE ROS topic (`NavigationEvent`).
Executor does NOT know about MQTT/JSON/mapping - only ROS messages.

Transport layer will subscribe to these ROS topics and map them to MQTT.
"""

from typing import Optional
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from aehub_msgs.msg import NavigationEvent

# Single event stream QoS:
# We intentionally use TRANSIENT_LOCAL so late-joining subscribers can see recent
# events (especially the latest state snapshot). This means ack/result may also
# be re-delivered, which is acceptable and aligns with idempotency/replay.
NAV_EVENT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class EventPublisher:
    """
    Publishes navigation events to a single ROS topic.
    
    Transport-agnostic: only ROS messages, no JSON, no MQTT knowledge.
    Transport layer will subscribe and map to MQTT.
    """
    
    def __init__(self, node, event_topic: str = "events/navigation"):
        """
        Initialize event publisher.
        
        Args:
            node: ROS2 node instance
            event_topic: Unified ROS topic for NavigationEvent stream
        """
        self._node = node
        self._event_pub = self._node.create_publisher(
            NavigationEvent,
            event_topic,
            NAV_EVENT_QOS,
        )
    
    def publish_ack(self, command_id: str, target_id: Optional[str] = None, 
                   ack_status: str = None, status: str = None, 
                   reason: Optional[str] = None, error_code: Optional[str] = None):
        """
        Publish ACK event as ROS message (transport-agnostic).
        
        Args:
            command_id: Command ID (UUID v4)
            target_id: Optional target ID
            ack_status: "received" | "accepted" | "rejected" (preferred)
            status: "received" | "accepted" | "rejected" (deprecated, for compatibility)
            reason: Optional rejection reason
            error_code: Optional error code (deprecated, for compatibility)
        """
        # Support both old and new parameter names (compatibility)
        status_value = ack_status or status or "received"
        reason_value = reason or error_code or ""
        
        ev = NavigationEvent()
        ev.stamp = self._node.get_clock().now().to_msg()
        ev.event_type = "ack"
        ev.command_id = command_id
        if target_id:
            ev.target_id = target_id
        ev.ack_status = status_value
        ev.ack_reason = reason_value
        self._event_pub.publish(ev)
    
    def publish_result(self, command_id: str, target_id: Optional[str] = None,
                      result_status: str = None, result: str = None,
                      reason: Optional[str] = None, error_code: Optional[str] = None):
        """
        Publish RESULT event as ROS message (transport-agnostic).
        
        Args:
            command_id: Command ID (UUID v4)
            target_id: Optional target ID
            result_status: "succeeded" | "canceled" | "aborted" | "error" (preferred)
            result: "succeeded" | "canceled" | "aborted" | "error" (deprecated, for compatibility)
            reason: Optional error reason or message
            error_code: Optional error code (deprecated, for compatibility)
        """
        # Support both old and new parameter names (compatibility)
        result_value = result_status or result or "error"
        reason_value = reason or error_code or ""
        
        ev = NavigationEvent()
        ev.stamp = self._node.get_clock().now().to_msg()
        ev.event_type = "result"
        ev.command_id = command_id
        if target_id:
            ev.target_id = target_id
        ev.result_status = result_value
        ev.result_reason = reason_value
        self._event_pub.publish(ev)
    
    def publish_state(self, public_state: Optional[str] = None, state: Optional[str] = None,
                     active_command_id: Optional[str] = None, active_target_id: Optional[str] = None,
                     internal_state: Optional[str] = None):
        """
        Publish STATE snapshot as ROS message (transport-agnostic).
        
        Args:
            public_state: Public state "idle" | "navigating" | "canceled" (preferred)
            state: State (deprecated, for compatibility)
            active_command_id: Optional active command ID
            active_target_id: Optional active target ID
            internal_state: Optional internal FSM state
        """
        state_value = public_state or state or "idle"
        
        ev = NavigationEvent()
        ev.stamp = self._node.get_clock().now().to_msg()
        ev.event_type = "state"
        if active_command_id:
            ev.command_id = active_command_id
        if active_target_id:
            ev.target_id = active_target_id
        ev.public_state = state_value
        ev.internal_state = internal_state or ""
        self._event_pub.publish(ev)
