#!/usr/bin/env python3
#
# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""aehub_mqtt_protocol_adapter.mqtt_protocol_adapter_node

Protocol edge adapter between:
- MQTT transport bridge (`infra/mqtt/in`, `infra/mqtt/out` as `aehub_msgs/MqttEnvelope`)
and
- internal transport-agnostic ROS API (`aehub_msgs/*`)

Responsibilities:
- JSON parse/serialize at the edge
- schema-level validation (basic fields)
- NO business logic: no dedup, no FSM, no rate limiting, no Nav2, no STOP
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg
from aehub_msgs.msg import MqttEnvelope, NavigationCommand, NavigationEvent, NavigationStatus


class MqttProtocolAdapterNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("mqtt_protocol_adapter_node")

        # Parameters (topics)
        # Defaults are relative to support namespace robot/<id>/...
        self.declare_parameter("mqtt_in_topic", "infra/mqtt/in")
        self.declare_parameter("mqtt_out_topic", "infra/mqtt/out")
        self.declare_parameter("cmd_topic", "commands/navigation")

        # Single event stream (rule-of-1-input-topic)
        self.declare_parameter("event_topic", "events/navigation")

        # UI telemetry stream (optional)
        self.declare_parameter("nav_status_topic", "status/navigation")

        # Legacy params (deprecated, ignored)
        self.declare_parameter("event_ack_topic", "events/command_ack")
        self.declare_parameter("event_result_topic", "events/command_result")
        self.declare_parameter("event_state_topic", "events/navigation_state")

        # Robot identity (for outbound event envelope; optional)
        self.declare_parameter("robot_id", "")

        self._active = False

        self._mqtt_in_sub = None
        self._mqtt_out_pub = None
        self._cmd_pub = None

        self._event_sub = None
        self._nav_status_sub = None

    # ---------------- lifecycle ----------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state

        mqtt_in = str(self.get_parameter("mqtt_in_topic").value)
        mqtt_out = str(self.get_parameter("mqtt_out_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        event_topic = str(self.get_parameter("event_topic").value)
        nav_status_topic = str(self.get_parameter("nav_status_topic").value)

        # QoS: inbound raw MQTT messages should not be lost in-process
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._mqtt_out_pub = self.create_publisher(MqttEnvelope, mqtt_out, qos_reliable)
        self._cmd_pub = self.create_publisher(NavigationCommand, cmd_topic, qos_reliable)

        self._mqtt_in_sub = self.create_subscription(MqttEnvelope, mqtt_in, self._on_mqtt_in, qos_reliable)

        # Subscribe to internal event stream and serialize out to MQTT
        self._event_sub = self.create_subscription(NavigationEvent, event_topic, self._on_event, qos_reliable)

        # Subscribe to internal UI telemetry and serialize out to MQTT
        self._nav_status_sub = self.create_subscription(
            NavigationStatus, nav_status_topic, self._on_nav_status, qos_reliable
        )

        self.get_logger().info(
            "Configured MQTT protocol adapter: "
            f"mqtt_in={mqtt_in} mqtt_out={mqtt_out} cmd_topic={cmd_topic} "
            f"event_topic={event_topic} nav_status_topic={nav_status_topic}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = True
        self.get_logger().info("Activated MQTT protocol adapter")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        self.get_logger().info("Deactivated MQTT protocol adapter")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False

        for sub in [self._mqtt_in_sub, self._event_sub, self._nav_status_sub]:
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:
                    pass
        self._mqtt_in_sub = None
        self._event_sub = None
        self._nav_status_sub = None

        for pub in [self._mqtt_out_pub, self._cmd_pub]:
            if pub is not None:
                try:
                    self.destroy_publisher(pub)
                except Exception:
                    pass
        self._mqtt_out_pub = None
        self._cmd_pub = None

        self.get_logger().info("Cleaned up MQTT protocol adapter")
        return TransitionCallbackReturn.SUCCESS

    # ---------------- inbound: MQTT → NavigationCommand ----------------
    def _on_mqtt_in(self, msg: MqttEnvelope) -> None:
        if not self._active:
            return

        mqtt_topic = str(msg.mqtt_topic or "")
        payload_raw = str(msg.payload_raw or "")

        if not mqtt_topic:
            self.get_logger().warn("MQTT envelope missing mqtt_topic")
            return

        payload = self._parse_json(payload_raw) if payload_raw else None
        if payload_raw and payload is None:
            # Non-JSON payload: ignore (protocol adapter is for JSON commands)
            self.get_logger().warn(f"Non-JSON payload on {mqtt_topic}: {payload_raw[:200]}")
            return
        payload = payload or {}

        # Route by topic suffix (no business logic, just protocol routing)
        if mqtt_topic.endswith("/commands/navigateTo"):
            cmd = self._build_navigation_command(payload, command_type="navigateTo")
        elif mqtt_topic.endswith("/commands/cancel"):
            cmd = self._build_navigation_command(payload, command_type="cancel")
        else:
            # Unknown inbound topic: ignore
            return

        if cmd is None:
            return

        self._cmd_pub.publish(cmd)

    def _build_navigation_command(self, payload: Dict[str, Any], *, command_type: str) -> Optional[NavigationCommand]:
        command_id = str(payload.get("command_id") or "")
        if not command_id:
            self.get_logger().warn(f"Rejecting inbound {command_type}: missing command_id")
            return None

        out = NavigationCommand()
        out.command_id = command_id
        out.type = command_type

        # Stamp: best effort. If payload includes unix seconds, convert; else now().
        out.stamp = self.get_clock().now().to_msg()

        target_id = payload.get("target_id")
        out.target_id = str(target_id) if isinstance(target_id, str) else ""

        if command_type == "navigateTo":
            # MVP (machine-owned): navigateTo is addressed by target_id (5 predefined positions).
            if not out.target_id:
                self.get_logger().warn("Rejecting inbound navigateTo: missing/invalid target_id")
                return None

            # Keep legacy fields in the ROS envelope, but do not require them.
            # Executor/capability action schema may still include x/y/theta; fill with defaults.
            try:
                out.x = float(payload.get("x", 0.0))
                out.y = float(payload.get("y", 0.0))
                out.theta = float(payload.get("theta", 0.0))
            except Exception:
                out.x = 0.0
                out.y = 0.0
                out.theta = 0.0

        return out

    # ---------------- outbound: events → MQTT JSON ----------------
    def _on_event(self, msg: NavigationEvent) -> None:
        if not self._active or self._mqtt_out_pub is None:
            return

        robot_id = str(self.get_parameter("robot_id").value or "")
        mqtt_topic = f"aroc/robot/{robot_id}/events" if robot_id else "aroc/robot/unknown/events"

        event: Dict[str, Any] = {
            "event_type": msg.event_type,
            "command_id": msg.command_id,
            "target_id": msg.target_id,
            "stamp": self._time_to_dict(msg.stamp),
        }
        if msg.event_type == "ack":
            event.update({"status": msg.ack_status, "reason": msg.ack_reason})
        elif msg.event_type == "result":
            event.update({"status": msg.result_status, "reason": msg.result_reason})
        elif msg.event_type == "state":
            event.update({"public_state": msg.public_state, "internal_state": msg.internal_state})

        payload = {
            "schema": "aehub.mqtt.events.v2",
            "robot_id": robot_id,
            "event": event,
        }

        out = MqttEnvelope()
        out.mqtt_topic = mqtt_topic
        out.payload_raw = json.dumps(payload, ensure_ascii=False)
        out.received_at = self.get_clock().now().to_msg()
        self._mqtt_out_pub.publish(out)

    def _on_nav_status(self, msg: NavigationStatus) -> None:
        if not self._active or self._mqtt_out_pub is None:
            return

        robot_id = str(self.get_parameter("robot_id").value or "")
        mqtt_topic = (
            f"aroc/robot/{robot_id}/status/navigation" if robot_id else "aroc/robot/unknown/status/navigation"
        )

        payload: Dict[str, Any] = {
            "schema": "aehub.mqtt.status.navigation.v1",
            "robot_id": robot_id,
            "status": {
                "stamp": self._time_to_dict(msg.stamp),
                "status": msg.status,
                "command_id": msg.command_id,
                "target_id": msg.target_id,
                "progress_percent": float(msg.progress_percent),
                "eta_seconds": int(msg.eta_seconds),
                "current_position": {"x": float(msg.current_x), "y": float(msg.current_y), "theta": float(msg.current_theta)},
            },
        }

        out = MqttEnvelope()
        out.mqtt_topic = mqtt_topic
        out.payload_raw = json.dumps(payload, ensure_ascii=False)
        out.received_at = self.get_clock().now().to_msg()
        self._mqtt_out_pub.publish(out)

    # ---------------- helpers ----------------
    @staticmethod
    def _parse_json(text: str) -> Optional[Dict[str, Any]]:
        try:
            obj = json.loads(text) if text else None
        except Exception:
            return None
        return obj if isinstance(obj, dict) else None

    @staticmethod
    def _time_to_dict(t: TimeMsg) -> Dict[str, int]:
        return {"sec": int(t.sec), "nanosec": int(t.nanosec)}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MqttProtocolAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

