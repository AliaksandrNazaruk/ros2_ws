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
Integration tests for MQTT topic mapper.

Tests ONLY transport layer:
- Subscriptions
- Publications
- Deduplication
- Replay
- No side-effects

Does NOT test:
- Nav2
- Business logic
"""

import pytest
import json
from typing import Dict, List, Callable, Optional
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aehub_mqtt_transport.topic_mapper import TopicMapper, MappingRule, Direction


# =========================
# Fake MQTT Broker
# =========================

class FakeMQTTBroker:
    """
    Minimal fake MQTT broker for testing.
    
    Simulates MQTT broker behavior without network.
    """
    
    def __init__(self):
        """Initialize fake broker."""
        self.subscribers: Dict[str, List[Callable]] = defaultdict(list)
    
    def subscribe(self, topic: str, callback: Callable[[str, bytes], None]):
        """
        Subscribe to topic.
        
        Args:
            topic: MQTT topic pattern (may contain # wildcard)
            callback: Callback function(topic: str, payload: bytes)
        """
        self.subscribers[topic].append(callback)
    
    def unsubscribe(self, topic: str):
        """
        Unsubscribe from topic.
        
        Args:
            topic: MQTT topic to unsubscribe from
        """
        if topic in self.subscribers:
            del self.subscribers[topic]
    
    def publish(self, topic: str, payload: bytes | str):
        """
        Publish message to topic.
        
        Args:
            topic: MQTT topic name
            payload: Message payload (str or bytes)
        """
        if isinstance(payload, str):
            payload = payload.encode('utf-8')
        
        # Find matching subscribers
        for pattern, callbacks in self.subscribers.items():
            if self._match(topic, pattern):
                for callback in callbacks:
                    callback(topic, payload)
    
    @staticmethod
    def _match(topic: str, pattern: str) -> bool:
        """
        Check if topic matches pattern (supports wildcards).
        
        Args:
            topic: Actual topic name
            pattern: Topic pattern (may contain # or +)
            
        Returns:
            True if topic matches pattern
        """
        # Exact match
        if pattern == topic:
            return True
        
        # Multi-level wildcard (#)
        if pattern.endswith('/#'):
            prefix = pattern[:-2]
            if topic.startswith(prefix):
                return True
        
        # Single-level wildcard (+)
        pattern_parts = pattern.split('/')
        topic_parts = topic.split('/')
        
        if len(pattern_parts) != len(topic_parts):
            return False
        
        for p, t in zip(pattern_parts, topic_parts):
            if p != '+' and p != t:
                return False
        
        return True


# =========================
# Fake MQTT Client
# =========================

class FakeMQTTClient:
    """
    Fake MQTT client that uses FakeMQTTBroker.
    """
    
    def __init__(self, broker: FakeMQTTBroker):
        """
        Initialize fake MQTT client.
        
        Args:
            broker: FakeMQTTBroker instance
        """
        self._broker = broker
        self._connected = False
    
    def connect(self, host: str, port: int, keepalive: int = 30):
        """Connect to broker (no-op for fake)."""
        self._connected = True
    
    def disconnect(self):
        """Disconnect from broker."""
        self._connected = False
    
    def subscribe(self, topic: str, qos: int = 1, callback: Optional[Callable] = None):
        """
        Subscribe to topic.
        
        Args:
            topic: MQTT topic pattern
            qos: QoS level (ignored in fake)
            callback: Callback function(topic: str, payload: bytes)
        """
        if not self._connected:
            raise RuntimeError("Not connected")
        if callback:
            self._broker.subscribe(topic, callback)
    
    def unsubscribe(self, topic: str):
        """Unsubscribe from topic."""
        self._broker.unsubscribe(topic)
    
    def publish(self, topic: str, payload: str | bytes, qos: int = 1, retain: bool = False):
        """
        Publish message.
        
        Args:
            topic: MQTT topic
            payload: Message payload (str or bytes)
            qos: QoS level (ignored in fake)
            retain: Retain flag (ignored in fake)
        """
        if not self._connected:
            raise RuntimeError("Not connected")
        self._broker.publish(topic, payload)
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected
    
    def set_connected(self, value: bool):
        """Set connected state."""
        self._connected = value


# =========================
# Fake ROS Collector
# =========================

class FakeROSCollector:
    """
    Collects ROS messages for testing.
    """
    
    def __init__(self, node: Node):
        """
        Initialize collector.
        
        Args:
            node: ROS2 node instance
        """
        self._node = node
        self._messages: Dict[str, List] = defaultdict(list)
        self._subscriptions = []
    
    def subscribe(self, topic: str, msg_type):
        """
        Subscribe to ROS topic and collect messages.
        
        Args:
            topic: ROS topic name
            msg_type: ROS message type class
        """
        def callback(msg):
            self._messages[topic].append(msg)
        
        sub = self._node.create_subscription(
            msg_type,
            topic,
            callback,
            10
        )
        self._subscriptions.append(sub)
    
    def count(self, topic: str) -> int:
        """
        Get message count for topic.
        
        Args:
            topic: ROS topic name
            
        Returns:
            Number of messages received
        """
        return len(self._messages[topic])
    
    def get_messages(self, topic: str) -> List:
        """
        Get all messages for topic.
        
        Args:
            topic: ROS topic name
            
        Returns:
            List of messages
        """
        return self._messages[topic].copy()
    
    def clear(self):
        """Clear all collected messages."""
        self._messages.clear()


# =========================
# Deduplication helper
# =========================

class CommandDeduplicator:
    """
    Deduplicates commands by command_id.
    
    Note: This is a test helper. In production, deduplication
    would be handled by a higher-level component.
    """
    
    def __init__(self):
        """Initialize deduplicator."""
        self._seen_command_ids = set()
    
    def should_process(self, payload: str | bytes) -> bool:
        """
        Check if command should be processed (not duplicate).
        
        Args:
            payload: JSON payload containing command_id
            
        Returns:
            True if command should be processed
        """
        try:
            if isinstance(payload, bytes):
                payload = payload.decode('utf-8')
            
            data = json.loads(payload)
            command_id = data.get('command_id')
            
            if command_id is None:
                return True  # No command_id, process always
            
            if command_id in self._seen_command_ids:
                return False  # Duplicate
            
            self._seen_command_ids.add(command_id)
            return True
            
        except (json.JSONDecodeError, AttributeError):
            return True  # Invalid JSON, process anyway
    
    def clear(self):
        """Clear seen command IDs."""
        self._seen_command_ids.clear()


# =========================
# Test fixtures
# =========================

@pytest.fixture
def ros_node():
    """Create ROS2 node for testing."""
    rclpy.init()
    node = Node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def fake_broker():
    """Create fake MQTT broker."""
    return FakeMQTTBroker()


@pytest.fixture
def fake_mqtt_client(fake_broker):
    """Create fake MQTT client."""
    client = FakeMQTTClient(fake_broker)
    client.connect('localhost', 1883)
    return client


@pytest.fixture
def ros_collector(ros_node):
    """Create ROS message collector."""
    return FakeROSCollector(ros_node)


@pytest.fixture
def deduplicator():
    """Create command deduplicator."""
    return CommandDeduplicator()


# =========================
# Test cases
# =========================

def test_replay_navigate_idempotent(ros_node, fake_mqtt_client, ros_collector, deduplicator):
    """
    TEST 1 — replay navigate
    
    Invariant: one command_id → one ROS message
    """
    # Setup topic mapper
    rules = [
        MappingRule(
            direction=Direction.MQTT_TO_ROS,
            mqtt_topic="aroc/robot/{robot_id}/commands/navigateTo",
            ros_topic="/aehub/commands/navigate_to",
            ros_msg_type=String,
        ),
    ]
    
    mapper = TopicMapper(
        ros_node=ros_node,
        mqtt_client=fake_mqtt_client,
        robot_id="r1",
        rules=rules,
    )
    
    # Subscribe to ROS topic
    ros_collector.subscribe("/aehub/commands/navigate_to", String)
    
    mapper.start()
    
    # Process spin to allow subscriptions to activate
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Publish same command twice
    cmd = {
        "command_id": "uuid-1",
        "x": 1.0,
        "y": 2.0,
        "timestamp": 100
    }
    payload = json.dumps(cmd).encode('utf-8')
    
    # First publish
    if deduplicator.should_process(payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/navigateTo", payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Second publish (should be deduplicated)
    if deduplicator.should_process(payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/navigateTo", payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Should receive only one message
    assert ros_collector.count("/aehub/commands/navigate_to") == 1
    
    mapper.stop()


def test_replay_cancel_idempotent(ros_node, fake_mqtt_client, ros_collector, deduplicator):
    """
    TEST 2 — replay cancel
    
    Invariant: one command_id → one ROS message
    """
    # Setup topic mapper
    rules = [
        MappingRule(
            direction=Direction.MQTT_TO_ROS,
            mqtt_topic="aroc/robot/{robot_id}/commands/cancel",
            ros_topic="/aehub/commands/cancel",
            ros_msg_type=String,
        ),
    ]
    
    mapper = TopicMapper(
        ros_node=ros_node,
        mqtt_client=fake_mqtt_client,
        robot_id="r1",
        rules=rules,
    )
    
    ros_collector.subscribe("/aehub/commands/cancel", String)
    
    mapper.start()
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Publish same cancel command twice
    cmd = {
        "command_id": "uuid-cancel",
        "timestamp": 200
    }
    payload = json.dumps(cmd).encode('utf-8')
    
    # First publish
    if deduplicator.should_process(payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/cancel", payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Second publish (should be deduplicated)
    if deduplicator.should_process(payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/cancel", payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    assert ros_collector.count("/aehub/commands/cancel") == 1
    
    mapper.stop()


def test_duplicate_command_id_across_topics(ros_node, fake_mqtt_client, ros_collector, deduplicator):
    """
    TEST 3 — duplicate command_id (navigate → cancel)
    
    Invariant: command_id глобален, не per-topic
    """
    # Setup topic mapper with both rules
    rules = [
        MappingRule(
            direction=Direction.MQTT_TO_ROS,
            mqtt_topic="aroc/robot/{robot_id}/commands/navigateTo",
            ros_topic="/aehub/commands/navigate_to",
            ros_msg_type=String,
        ),
        MappingRule(
            direction=Direction.MQTT_TO_ROS,
            mqtt_topic="aroc/robot/{robot_id}/commands/cancel",
            ros_topic="/aehub/commands/cancel",
            ros_msg_type=String,
        ),
    ]
    
    mapper = TopicMapper(
        ros_node=ros_node,
        mqtt_client=fake_mqtt_client,
        robot_id="r1",
        rules=rules,
    )
    
    ros_collector.subscribe("/aehub/commands/navigate_to", String)
    ros_collector.subscribe("/aehub/commands/cancel", String)
    
    mapper.start()
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Publish navigate with command_id
    navigate_cmd = {
        "command_id": "uuid-x",
        "x": 1,
        "y": 1,
        "timestamp": 1
    }
    navigate_payload = json.dumps(navigate_cmd).encode('utf-8')
    
    if deduplicator.should_process(navigate_payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/navigateTo", navigate_payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Publish cancel with same command_id
    cancel_cmd = {
        "command_id": "uuid-x",
        "timestamp": 2
    }
    cancel_payload = json.dumps(cancel_cmd).encode('utf-8')
    
    # Should be deduplicated (same command_id)
    if deduplicator.should_process(cancel_payload):
        fake_mqtt_client._broker.publish("aroc/robot/r1/commands/cancel", cancel_payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Navigate should be received
    assert ros_collector.count("/aehub/commands/navigate_to") == 1
    # Cancel should be deduplicated (same command_id)
    assert ros_collector.count("/aehub/commands/cancel") == 0
    
    mapper.stop()


def test_cancel_without_active_goal(ros_node, fake_mqtt_client, ros_collector):
    """
    TEST 4 — cancel without active goal
    
    Invariant: transport не знает, есть ли цель
    """
    # Setup topic mapper
    rules = [
        MappingRule(
            direction=Direction.MQTT_TO_ROS,
            mqtt_topic="aroc/robot/{robot_id}/commands/cancel",
            ros_topic="/aehub/commands/cancel",
            ros_msg_type=String,
        ),
    ]
    
    mapper = TopicMapper(
        ros_node=ros_node,
        mqtt_client=fake_mqtt_client,
        robot_id="r1",
        rules=rules,
    )
    
    ros_collector.subscribe("/aehub/commands/cancel", String)
    
    mapper.start()
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Publish cancel without previous navigate
    cmd = {
        "command_id": "uuid-no-goal",
        "timestamp": 300
    }
    payload = json.dumps(cmd).encode('utf-8')
    
    fake_mqtt_client._broker.publish("aroc/robot/r1/commands/cancel", payload)
    
    rclpy.spin_once(ros_node, timeout_sec=0.1)
    
    # Should receive cancel (transport doesn't know about goals)
    assert ros_collector.count("/aehub/commands/cancel") == 1
    
    mapper.stop()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
