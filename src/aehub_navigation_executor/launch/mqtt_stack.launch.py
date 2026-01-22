# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
Launch file for MQTT stack (broker credentials, transport, protocol adapter).

Starts (all lifecycle nodes):
- aehub_broker_credentials
- aehub_mqtt_transport
- aehub_mqtt_protocol_adapter

This launch file performs automatic configure+activate transitions to put the
system into ACTIVE state without requiring a separate lifecycle_manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
import yaml


def _load_defaults_from_packaged_yaml() -> tuple[str, str]:
    """Load default config_service_url and api_key from packaged YAML."""
    try:
        cfg_path = os.path.join(
            get_package_share_directory("aehub_broker_credentials"),
            "config",
            "broker_credentials.yaml",
        )
        if not os.path.exists(cfg_path):
            return ("", "")
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("broker_credentials_node", {}).get("ros__parameters", {}) or {}
        url = str(params.get("config_service_url", "") or "").strip()
        api_key = str(params.get("api_key", "") or "").strip()
        return (url, api_key)
    except Exception:
        return ("", "")


def _autostart_lifecycle(node: LifecycleNode, *, configure_delay_s: float, activate_delay_s: float):
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[TimerAction(period=configure_delay_s, actions=[configure_event])],
        )
    )
    # IMPORTANT: Chain activation only after successful CONFIGURE (node reaches INACTIVE).
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[TimerAction(period=activate_delay_s, actions=[activate_event])],
        )
    )

    return [configure_handler, activate_handler]


def generate_launch_description() -> LaunchDescription:
    default_url, default_api_key = _load_defaults_from_packaged_yaml()

    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=EnvironmentVariable("AEHUB_ROBOT_ID", default_value="robot_001"),
        description="Robot identifier used for ROS namespace: /robot/<id>/... (must be a valid ROS name segment).",
    )

    mqtt_robot_id_arg = DeclareLaunchArgument(
        "mqtt_robot_id",
        default_value=LaunchConfiguration("robot_id"),
        description="Robot identifier used in MQTT topics (aroc/robot/<id>/...).",
    )

    launch_broker_credentials_node_arg = DeclareLaunchArgument(
        "launch_broker_credentials_node",
        default_value="true",
        description="If true, start broker_credentials_node in this namespace (otherwise reuse an external publisher).",
    )

    broker_config_topic_arg = DeclareLaunchArgument(
        "broker_config_topic",
        default_value="infra/mqtt/broker_config",
        description="BrokerConfig topic name (set to /infra/mqtt/broker_config to reuse a global publisher).",
    )

    mqtt_ca_cert_path_arg = DeclareLaunchArgument(
        "mqtt_ca_cert_path",
        default_value="/home/boris/ros2_ws/ca.crt",
        description="Path to CA certificate used for TLS MQTT verification.",
    )

    config_service_url_arg = DeclareLaunchArgument(
        "config_service_url",
        default_value=EnvironmentVariable("CONFIG_SERVICE_URL", default_value=default_url or "http://localhost:7900"),
        description="Config service base URL for broker credentials node.",
    )
    api_key_arg = DeclareLaunchArgument(
        "api_key",
        default_value=EnvironmentVariable("CONFIG_SERVICE_API_KEY", default_value=default_api_key or ""),
        description="Config service API key for broker credentials node.",
    )

    # --- Nodes (all launched in root namespace) ---
    broker = LifecycleNode(
        package="aehub_broker_credentials",
        executable="broker_credentials_node",
        name="broker_credentials_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("launch_broker_credentials_node")),
        parameters=[
            {
                "config_service_url": LaunchConfiguration("config_service_url"),
                "api_key": LaunchConfiguration("api_key"),
                "broker_config_topic": LaunchConfiguration("broker_config_topic"),
            }
        ],
    )

    transport = LifecycleNode(
        package="aehub_mqtt_transport",
        executable="mqtt_transport_node",
        name="mqtt_transport_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_id": ParameterValue(LaunchConfiguration("mqtt_robot_id"), value_type=str)},
            {"broker_config_topic": LaunchConfiguration("broker_config_topic")},
            {"mqtt_ca_cert_path": LaunchConfiguration("mqtt_ca_cert_path")},
        ],
    )

    proto = LifecycleNode(
        package="aehub_mqtt_protocol_adapter",
        executable="mqtt_protocol_adapter_node",
        name="mqtt_protocol_adapter_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_id": ParameterValue(LaunchConfiguration("mqtt_robot_id"), value_type=str)}],
    )

    # --- Autostart lifecycle (simple staged delays) ---
    broker_handlers = _autostart_lifecycle(broker, configure_delay_s=0.5, activate_delay_s=1.0)
    transport_handlers = _autostart_lifecycle(transport, configure_delay_s=1.5, activate_delay_s=2.0)
    proto_handlers = _autostart_lifecycle(proto, configure_delay_s=2.5, activate_delay_s=3.0)

    return LaunchDescription(
        [
            robot_id_arg,
            mqtt_robot_id_arg,
            launch_broker_credentials_node_arg,
            broker_config_topic_arg,
            mqtt_ca_cert_path_arg,
            config_service_url_arg,
            api_key_arg,
            # All nodes in root namespace
            broker,
            transport,
            proto,
            *broker_handlers,
            *transport_handlers,
            *proto_handlers,
        ]
    )
