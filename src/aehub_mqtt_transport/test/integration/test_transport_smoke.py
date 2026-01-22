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
Smoke test for transport layer.

Process-level integration test that verifies:
- Node starts successfully
- YAML config can be validated and loaded
- MQTT → ROS mapping works
- ROS → MQTT mapping works
- Node shuts down cleanly

This is a real integration test using actual ROS2 and fake MQTT broker.
"""

import json
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aehub_mqtt_transport.topic_mapper import TopicMapper, MappingRule, Direction
from aehub_mqtt_transport.mapping_loader import load_mapping_config, MappingSchemaError

# Import test utilities
from test_mqtt_topic_mapper import FakeMQTTBroker, FakeMQTTClient, FakeROSCollector


# #region agent log
DEBUG_LOG_PATH = "/home/boris/ros2_ws/.cursor/debug.log"

def _log_debug(location: str, message: str, data: dict = None, hypothesis_id: str = None):
    """Write debug log entry."""
    try:
        log_entry = {
            "location": location,
            "message": message,
            "timestamp": int(time.time() * 1000),
            "sessionId": "debug-session",
            "runId": "smoke-test",
        }
        if data:
            log_entry["data"] = data
        if hypothesis_id:
            log_entry["hypothesisId"] = hypothesis_id
        
        with open(DEBUG_LOG_PATH, "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except Exception:
        pass  # Don't fail test on logging errors
# #endregion


class TransportSmokeTest:
    """
    Smoke test for transport layer.
    
    Tests actual transport functionality with real ROS2 and fake MQTT.
    """
    
    def __init__(self):
        """Initialize smoke test."""
        self._mqtt_broker = FakeMQTTBroker()
        self._mqtt_client = FakeMQTTClient(self._mqtt_broker)
        self._mqtt_client.connect('localhost', 1883)
        
        self._node: Optional[Node] = None
        self._ros_collector: Optional[FakeROSCollector] = None
        self._topic_mapper: Optional[TopicMapper] = None
        
        _log_debug("test_transport_smoke.py:__init__", "Smoke test initialized")
    
    def setup_ros(self):
        """Setup ROS2 node and collector."""
        if not rclpy.ok():
            rclpy.init()
        
        self._node = Node('smoke_test_node')
        self._ros_collector = FakeROSCollector(self._node)
        
        _log_debug("test_transport_smoke.py:setup_ros", "ROS node and collector setup")
    
    def test_yaml_config_validation(self):
        """Test 1: YAML config can be loaded and validated."""
        workspace_path = Path(__file__).parent.parent.parent.parent.parent
        config_path = workspace_path / "src" / "aehub_mqtt_transport" / "config" / "mqtt_mapping.v1.yaml"
        
        _log_debug("test_transport_smoke.py:test_yaml_config_validation", "Loading YAML config", {
            "config_path": str(config_path)
        }, hypothesis_id="A")
        
        try:
            config = load_mapping_config(str(config_path))
            
            _log_debug("test_transport_smoke.py:test_yaml_config_validation", "YAML config loaded successfully", {
                "schema_version": config.get("schema", {}).get("version"),
                "topics_count": len(config.get("topics", {}))
            }, hypothesis_id="A")
            
            assert config is not None, "Config should not be None"
            assert 'schema' in config, "Config should have 'schema' section"
            assert 'topics' in config, "Config should have 'topics' section"
            
            return config
        except Exception as e:
            _log_debug("test_transport_smoke.py:test_yaml_config_validation", "YAML config validation failed", {
                "error": str(e),
                "error_type": type(e).__name__
            }, hypothesis_id="A")
            raise
    
    def test_mqtt_to_ros_mapping(self):
        """Test 2: MQTT → ROS mapping works."""
        robot_id = "smoke-test-robot"
        
        # Create mapping rule
        rules = [
            MappingRule(
                direction=Direction.MQTT_TO_ROS,
                mqtt_topic=f"aroc/robot/{robot_id}/commands/navigateTo",
                ros_topic="/aehub/commands/navigate_to",
                ros_msg_type=String,
            ),
        ]
        
        _log_debug("test_transport_smoke.py:test_mqtt_to_ros_mapping", "Setting up MQTT→ROS mapping", {
            "robot_id": robot_id,
            "rules_count": len(rules)
        }, hypothesis_id="B")
        
        # Setup topic mapper
        self._topic_mapper = TopicMapper(
            ros_node=self._node,
            mqtt_client=self._mqtt_client,
            robot_id=robot_id,
            rules=rules,
        )
        
        # Subscribe to ROS topic
        self._ros_collector.subscribe("/aehub/commands/navigate_to", String)
        
        # Start mapper
        self._topic_mapper.start()
        
        # Process ROS to activate subscriptions
        rclpy.spin_once(self._node, timeout_sec=0.1)
        time.sleep(0.5)
        
        _log_debug("test_transport_smoke.py:test_mqtt_to_ros_mapping", "Topic mapper started", 
            hypothesis_id="B")
        
        # Publish MQTT message
        mqtt_topic = f"aroc/robot/{robot_id}/commands/navigateTo"
        payload = {
            "command_id": "test-123",
            "x": 1.0,
            "y": 2.0,
            "timestamp": int(time.time())
        }
        payload_bytes = json.dumps(payload).encode('utf-8')
        
        _log_debug("test_transport_smoke.py:test_mqtt_to_ros_mapping", "Publishing MQTT message", {
            "topic": mqtt_topic,
            "payload_length": len(payload_bytes)
        }, hypothesis_id="B")
        
        self._mqtt_broker.publish(mqtt_topic, payload_bytes)
        
        # Wait for message propagation
        time.sleep(0.5)
        
        # Process ROS messages
        for _ in range(10):
            rclpy.spin_once(self._node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check if message was received
        count = self._ros_collector.count("/aehub/commands/navigate_to")
        
        _log_debug("test_transport_smoke.py:test_mqtt_to_ros_mapping", "Checking ROS message", {
            "topic": "/aehub/commands/navigate_to",
            "message_count": count
        }, hypothesis_id="B")
        
        assert count > 0, f"Should receive message on /aehub/commands/navigate_to, got {count}"
        
        # Verify message content
        messages = self._ros_collector.get_messages("/aehub/commands/navigate_to")
        if messages:
            msg_text = messages[0].data
            _log_debug("test_transport_smoke.py:test_mqtt_to_ros_mapping", "ROS message received", {
                "message_content": msg_text[:100]  # First 100 chars
            }, hypothesis_id="B")
            
            # Verify it's valid JSON
            parsed = json.loads(msg_text)
            assert "command_id" in parsed, "Message should contain command_id"
            assert parsed["command_id"] == "test-123", "Command ID should match"
    
    def test_ros_to_mqtt_mapping(self):
        """Test 3: ROS → MQTT mapping works."""
        robot_id = "smoke-test-robot"
        
        # Track published MQTT messages
        published_messages = []
        
        def mqtt_callback(topic: str, payload: bytes):
            published_messages.append((topic, payload))
        
        # Create mapping rule
        rules = [
            MappingRule(
                direction=Direction.ROS_TO_MQTT,
                ros_topic="/aehub/mqtt/out",
                mqtt_topic=f"aroc/robot/{robot_id}/events",
                ros_msg_type=String,
            ),
        ]
        
        _log_debug("test_transport_smoke.py:test_ros_to_mqtt_mapping", "Setting up ROS→MQTT mapping", {
            "robot_id": robot_id,
            "rules_count": len(rules)
        }, hypothesis_id="C")
        
        # Setup topic mapper
        self._topic_mapper = TopicMapper(
            ros_node=self._node,
            mqtt_client=self._mqtt_client,
            robot_id=robot_id,
            rules=rules,
        )
        
        # Subscribe to MQTT topic
        mqtt_topic = f"aroc/robot/{robot_id}/events"
        self._mqtt_broker.subscribe(mqtt_topic, mqtt_callback)
        
        # Start mapper
        self._topic_mapper.start()
        
        # Process ROS to activate subscriptions
        rclpy.spin_once(self._node, timeout_sec=0.1)
        time.sleep(0.5)
        
        # Publish ROS message
        ros_msg = String()
        ros_msg.data = json.dumps({"event": "test-event", "timestamp": int(time.time())})
        
        _log_debug("test_transport_smoke.py:test_ros_to_mqtt_mapping", "Publishing ROS message", {
            "topic": "/aehub/mqtt/out",
            "message_length": len(ros_msg.data)
        }, hypothesis_id="C")
        
        pub = self._node.create_publisher(String, "/aehub/mqtt/out", 10)
        pub.publish(ros_msg)
        
        # Wait for message propagation
        time.sleep(0.5)
        
        # Process ROS messages
        for _ in range(10):
            rclpy.spin_once(self._node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check if message was published to MQTT
        _log_debug("test_transport_smoke.py:test_ros_to_mqtt_mapping", "Checking MQTT message", {
            "topic": mqtt_topic,
            "published_count": len(published_messages)
        }, hypothesis_id="C")
        
        assert len(published_messages) > 0, f"Should publish message to MQTT {mqtt_topic}"
        assert published_messages[0][0] == mqtt_topic, "MQTT topic should match"
        
        # Verify payload
        payload = published_messages[0][1].decode('utf-8')
        parsed = json.loads(payload)
        assert "event" in parsed, "Payload should contain event"
    
    def shutdown(self):
        """Shutdown test resources cleanly."""
        _log_debug("test_transport_smoke.py:shutdown", "Shutting down test resources")
        
        if self._topic_mapper:
            self._topic_mapper.stop()
            self._topic_mapper = None
        
        if self._node:
            self._node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        _log_debug("test_transport_smoke.py:shutdown", "Test resources shut down")
    
    def run(self):
        """Run all smoke tests."""
        try:
            _log_debug("test_transport_smoke.py:run", "Starting smoke test suite")
            
            # Setup
            self.setup_ros()
            
            # Test 1: YAML config validation
            print("Test 1: YAML config validation...")
            self.test_yaml_config_validation()
            print("  ✅ YAML config loads and validates successfully")
            
            # Test 2: MQTT → ROS mapping
            print("Test 2: MQTT → ROS mapping...")
            self.test_mqtt_to_ros_mapping()
            print("  ✅ MQTT → ROS mapping works")
            
            # Test 3: ROS → MQTT mapping
            print("Test 3: ROS → MQTT mapping...")
            self.test_ros_to_mqtt_mapping()
            print("  ✅ ROS → MQTT mapping works")
            
            _log_debug("test_transport_smoke.py:run", "All smoke tests passed")
            
            print("\n✅ All smoke tests passed!")
            return True
            
        except Exception as e:
            _log_debug("test_transport_smoke.py:run", "Smoke test failed", {
                "error": str(e),
                "error_type": type(e).__name__
            })
            raise
        finally:
            self.shutdown()


def test_transport_smoke():
    """Main smoke test function."""
    # Clear previous logs
    if os.path.exists(DEBUG_LOG_PATH):
        os.remove(DEBUG_LOG_PATH)
    
    test = TransportSmokeTest()
    test.run()


if __name__ == '__main__':
    test_transport_smoke()
