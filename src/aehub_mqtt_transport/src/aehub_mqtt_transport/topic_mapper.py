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
Topic mapper for MQTT ↔ ROS topic mapping.

Pure adapter that:
- Describes and applies ROS topic ⇄ MQTT topic correspondence
- Defines routing rules, QoS, and payload transformation
- Does NOT know about MQTT client internals, lifecycle, or business logic
"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Type

import threading
import json

from std_msgs.msg import String


# =========================
# Data model
# =========================

class Direction(Enum):
    """Direction of topic mapping."""
    ROS_TO_MQTT = "ros_to_mqtt"
    MQTT_TO_ROS = "mqtt_to_ros"


@dataclass(frozen=True)
class MappingRule:
    """
    Mapping rule for ROS ↔ MQTT topic bridge.
    
    Attributes:
        direction: Direction of mapping (ROS_TO_MQTT or MQTT_TO_ROS)
        ros_topic: ROS topic name
        ros_msg_type: ROS message type class
        mqtt_topic: MQTT topic template (may contain {robot_id})
        mqtt_qos: MQTT QoS level (0, 1, or 2)
        mqtt_retain: MQTT retain flag
        enabled: Whether rule is enabled (allows hot-disable)
    """
    direction: Direction
    
    ros_topic: str
    ros_msg_type: Type
    
    mqtt_topic: str  # may contain {robot_id}
    mqtt_qos: int = 1
    mqtt_retain: bool = False
    
    enabled: bool = True


# =========================
# TopicMapper
# =========================

class TopicMapper:
    """
    Pure mapping layer between ROS topics and MQTT topics.
    
    Responsibilities:
    - Create ROS subscriptions / publishers
    - Register MQTT subscriptions
    - Apply topic templates ({robot_id})
    - Forward payloads without semantic interpretation
    
    Does NOT:
    - Know about MQTT client internals
    - Know about lifecycle
    - Know about business logic
    - Parse JSON semantically
    - Handle reconnect logic
    """
    
    def __init__(
        self,
        ros_node,
        mqtt_client,
        robot_id: str,
        rules: List[MappingRule],
    ):
        """
        Initialize topic mapper.
        
        Args:
            ros_node: ROS2 node instance (for creating pubs/subs)
            mqtt_client: MQTT client instance (must support subscribe/publish with callbacks)
            robot_id: Robot ID for topic template substitution
            rules: List of mapping rules
        """
        self._node = ros_node
        self._mqtt = mqtt_client
        self._robot_id = robot_id
        self._rules = [r for r in rules if r.enabled]
        
        self._ros_subs = []
        self._ros_pubs: Dict[str, any] = {}
        self._mqtt_subs: Dict[str, Callable] = {}
        
        self._lock = threading.Lock()
        self._running = False
    
    # =========================
    # Lifecycle
    # =========================
    
    def set_mqtt_client(self, mqtt_client):
        """
        Set MQTT client (can be called before or after start).
        
        Args:
            mqtt_client: MQTT client instance
        """
        with self._lock:
            self._mqtt = mqtt_client
    
    def start(self) -> None:
        """
        Start topic mapper.
        
        Creates ROS subscriptions and MQTT subscriptions according to rules.
        Idempotent: safe to call multiple times.
        
        Requires MQTT client to be set (via constructor or set_mqtt_client).
        """
        if not self._mqtt:
            raise RuntimeError("MQTT client not set. Call set_mqtt_client() first.")
        
        with self._lock:
            if self._running:
                return
            self._running = True
        
        self._node.get_logger().info(
            f"TopicMapper starting with {len(self._rules)} rules"
        )
        
        for rule in self._rules:
            if rule.direction == Direction.ROS_TO_MQTT:
                self._setup_ros_to_mqtt(rule)
            elif rule.direction == Direction.MQTT_TO_ROS:
                self._setup_mqtt_to_ros(rule)
    
    def stop(self) -> None:
        """
        Stop topic mapper.
        
        Tears down all subscriptions and publishers.
        Idempotent: safe to call multiple times.
        """
        with self._lock:
            if not self._running:
                return
            self._running = False
        
        # ROS subscriptions
        for sub in self._ros_subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._ros_subs.clear()
        
        # ROS publishers
        for pub in self._ros_pubs.values():
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass
        self._ros_pubs.clear()
        
        # MQTT subscriptions
        for topic in list(self._mqtt_subs.keys()):
            try:
                self._mqtt.unsubscribe(topic)
            except Exception:
                pass
        self._mqtt_subs.clear()
        
        self._node.get_logger().info("TopicMapper stopped")
    
    # =========================
    # Internal helpers
    # =========================
    
    def _expand_mqtt_topic(self, template: str) -> str:
        """Expand MQTT topic template with robot_id."""
        return template.format(robot_id=self._robot_id)
    
    # -------------------------
    # ROS → MQTT
    # -------------------------
    
    def _setup_ros_to_mqtt(self, rule: MappingRule) -> None:
        """Setup ROS → MQTT mapping."""
        mqtt_topic = self._expand_mqtt_topic(rule.mqtt_topic)
        
        def ros_callback(msg):
            try:
                payload = self._ros_msg_to_payload(msg)
                self._mqtt.publish(
                    topic=mqtt_topic,
                    payload=payload,
                    qos=rule.mqtt_qos,
                    retain=rule.mqtt_retain,
                )
            except Exception as e:
                self._node.get_logger().error(
                    f"ROS→MQTT publish failed ({mqtt_topic}): {e}"
                )
        
        sub = self._node.create_subscription(
            rule.ros_msg_type,
            rule.ros_topic,
            ros_callback,
            10,  # QoS depth
        )
        self._ros_subs.append(sub)
        
        self._node.get_logger().info(
            f"Mapped ROS→MQTT: {rule.ros_topic} → {mqtt_topic}"
        )
    
    # -------------------------
    # MQTT → ROS
    # -------------------------
    
    def _setup_mqtt_to_ros(self, rule: MappingRule) -> None:
        """Setup MQTT → ROS mapping."""
        mqtt_topic = self._expand_mqtt_topic(rule.mqtt_topic)
        
        # Create ROS publisher (lazy creation)
        pub = self._node.create_publisher(
            rule.ros_msg_type,
            rule.ros_topic,
            10,  # QoS depth
        )
        self._ros_pubs[rule.ros_topic] = pub
        
        def mqtt_callback(topic: str, payload: bytes):
            try:
                msg = self._payload_to_ros_msg(rule.ros_msg_type, topic, payload)
                pub.publish(msg)
            except Exception as e:
                self._node.get_logger().error(
                    f"MQTT→ROS publish failed ({topic}): {e}"
                )
        
        self._mqtt.subscribe(
            topic=mqtt_topic,
            qos=rule.mqtt_qos,
            callback=mqtt_callback,
        )
        self._mqtt_subs[mqtt_topic] = mqtt_callback
        
        self._node.get_logger().info(
            f"Mapped MQTT→ROS: {mqtt_topic} → {rule.ros_topic}"
        )
    
    # =========================
    # Payload handling
    # =========================
    
    @staticmethod
    def _ros_msg_to_payload(msg) -> str | bytes:
        """
        Convert ROS message to MQTT payload.
        
        Policy:
        - std_msgs/String → payload = msg.data
        - bytes (future) → payload = raw
        
        No semantic serialization inside mapper.
        """
        if isinstance(msg, String):
            return msg.data
        raise TypeError(f"Unsupported ROS message type: {type(msg)}")
    
    @staticmethod
    def _payload_to_ros_msg(msg_type: Type, mqtt_topic: str, payload: bytes):
        """
        Convert MQTT payload to ROS message.
        
        Policy:
        - publish as std_msgs/String
        - include MQTT topic in the message (transport metadata)
        - best-effort decode JSON payload and merge top-level fields for convenience
        
        Semantics are responsibility of upper layer, not transport.
        """
        if msg_type is String:
            payload_text = payload.decode("utf-8", errors="replace")
            envelope = {
                "mqtt_topic": str(mqtt_topic),
                "payload_raw": payload_text,
            }
            try:
                parsed = json.loads(payload_text)
                if isinstance(parsed, dict):
                    # Merge parsed fields to make command_id and other fields directly accessible.
                    envelope.update(parsed)
            except Exception:
                # Ignore parse errors; keep raw payload only.
                pass
            return String(data=json.dumps(envelope, ensure_ascii=False))
        raise TypeError(f"Unsupported ROS message type: {msg_type}")
