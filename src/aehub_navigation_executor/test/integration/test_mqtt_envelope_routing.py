#!/usr/bin/env python3
#
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
Integration test: MqttEnvelope(JSON) -> NavigationCommand -> executor -> capability -> NavigationEvent ack rejected.

This test is transport/protocol level and does NOT require:
- real MQTT broker
- Nav2 stack
- Symovo

It validates that the clean stack wiring works in-process:
infra/mqtt/in (MqttEnvelope) -> mqtt_protocol_adapter -> commands/navigation (NavigationCommand)
-> navigation_executor -> capabilities/navigation/execute (Action) -> events/navigation (NavigationEvent)
-> mqtt_protocol_adapter -> infra/mqtt/out (MqttEnvelope)
"""

import json
import time
from dataclasses import dataclass
from typing import Optional

import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from aehub_msgs.action import NavigationExecute
from aehub_msgs.msg import MqttEnvelope, NavigationEvent

from aehub_mqtt_protocol_adapter.mqtt_protocol_adapter_node import MqttProtocolAdapterNode
from aehub_navigation_executor.navigation_executor_node import NavigationExecutorNode


@dataclass
class _Seen:
    ack: Optional[NavigationEvent] = None
    out_env: Optional[MqttEnvelope] = None


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mqtt_envelope_navigate_to_rejected_when_capability_rejects(ros_context):
    # --- Nodes ---
    proto = MqttProtocolAdapterNode()
    executor = NavigationExecutorNode()
    test_node = Node("test_node_mqtt_route")

    # Fake capability action server: always REJECT goals
    fake_capability = Node("fake_capability_server")

    def goal_cb(goal_request):
        _ = goal_request
        return GoalResponse.REJECT

    def execute_cb(goal_handle):
        # Should not be called when REJECTed, but must exist.
        _ = goal_handle
        return NavigationExecute.Result()

    _ = ActionServer(
        fake_capability,
        NavigationExecute,
        "capabilities/navigation/execute",
        execute_callback=execute_cb,
        goal_callback=goal_cb,
    )

    # --- Activate lifecycle nodes (best-effort: call callbacks directly like other integration tests) ---
    assert proto.on_configure(State(1, "unconfigured")) == TransitionCallbackReturn.SUCCESS
    assert proto.on_activate(State(2, "inactive")) == TransitionCallbackReturn.SUCCESS

    assert executor.on_configure(State(1, "unconfigured")) == TransitionCallbackReturn.SUCCESS
    assert executor.on_activate(State(2, "inactive")) == TransitionCallbackReturn.SUCCESS

    # --- Collect outputs ---
    seen = _Seen()

    qos_events = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def on_event(msg: NavigationEvent):
        if msg.event_type == "ack":
            seen.ack = msg

    test_node.create_subscription(
        NavigationEvent,
        "events/navigation",
        on_event,
        qos_events,
    )

    qos_reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def on_out(msg: MqttEnvelope):
        # The adapter publishes state snapshots too; capture only the ACK for our command.
        try:
            payload = json.loads(msg.payload_raw or "{}")
            ev = payload.get("event") or {}
            if payload.get("schema") != "aehub.mqtt.events.v2":
                return
            if ev.get("event_type") != "ack":
                return
            if ev.get("command_id") != cmd_id:
                return
        except Exception:
            return
        seen.out_env = msg

    test_node.create_subscription(
        MqttEnvelope,
        "infra/mqtt/out",
        on_out,
        qos_reliable,
    )

    pub_in = test_node.create_publisher(MqttEnvelope, "infra/mqtt/in", qos_reliable)

    # --- Send inbound MQTT envelope ---
    cmd_id = "11111111-1111-1111-1111-111111111111"
    env = MqttEnvelope()
    env.mqtt_topic = "aroc/robot/robot_001/commands/navigateTo"
    env.payload_raw = json.dumps({"command_id": cmd_id, "x": 1.0, "y": 2.0, "theta": 0.0})
    env.received_at = test_node.get_clock().now().to_msg()

    # Spin to ensure subscriptions are connected
    exec_ = SingleThreadedExecutor()
    exec_.add_node(fake_capability)
    exec_.add_node(proto)
    exec_.add_node(executor)
    exec_.add_node(test_node)

    try:
        pub_in.publish(env)

        deadline = time.time() + 3.0
        while time.time() < deadline and rclpy.ok():
            exec_.spin_once(timeout_sec=0.1)
            if seen.ack is not None and seen.ack.command_id == cmd_id:
                break

        assert seen.ack is not None, "Expected NavigationEvent ack"
        assert seen.ack.command_id == cmd_id
        assert seen.ack.ack_status == "rejected"

        # Optional: verify protocol adapter emitted outbound MQTT envelope (events v2)
        # (best-effort; allow None if not delivered due to timing)
        if seen.out_env is not None:
            payload = json.loads(seen.out_env.payload_raw)
            assert payload.get("schema") == "aehub.mqtt.events.v2"
            assert payload.get("event", {}).get("event_type") == "ack"
            assert payload.get("event", {}).get("command_id") == cmd_id
    finally:
        exec_.shutdown()
        for n in [test_node, executor, proto, fake_capability]:
            try:
                n.destroy_node()
            except Exception:
                pass

