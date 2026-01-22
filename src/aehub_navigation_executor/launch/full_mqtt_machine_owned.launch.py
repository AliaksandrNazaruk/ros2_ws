# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Machine-owned production bringup (NO Nav2 ownership in this process).
#
# Starts:
# - aehub_broker_credentials (Lifecycle)
# - aehub_mqtt_transport (Lifecycle)
# - aehub_mqtt_protocol_adapter (Lifecycle)
# - aehub_navigation_executor (Lifecycle)
# - aehub_navigation_backend/navigation_backend_server (Lifecycle)
#
# Notes:
# - Navigation is executed via Symovo HTTP API (Transport engine).
# - navigateTo MUST use target_id (predefined mapping target_id -> station_id).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
import yaml


def _load_defaults_from_packaged_yaml() -> tuple[str, str]:
    """Load default config_service_url and api_key from packaged YAML (best-effort)."""
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

    # --- IDs ---
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=EnvironmentVariable("AEHUB_ROBOT_ID", default_value="robot_001"),
        description="Robot identifier (used in MQTT topics).",
    )
    mqtt_robot_id_arg = DeclareLaunchArgument(
        "mqtt_robot_id",
        default_value=LaunchConfiguration("robot_id"),
        description="Robot identifier used in MQTT topics (aroc/robot/<id>/...).",
    )

    # --- Broker credentials (config service) ---
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
    broker_config_topic_arg = DeclareLaunchArgument(
        "broker_config_topic",
        default_value="infra/mqtt/broker_config",
        description="BrokerConfig topic name.",
    )

    # --- MQTT TLS CA ---
    mqtt_ca_cert_path_arg = DeclareLaunchArgument(
        "mqtt_ca_cert_path",
        default_value="/home/boris/ros2_ws/ca.crt",
        description="Path to CA certificate used for TLS MQTT verification.",
    )

    # --- Machine HTTP backend ---
    symovo_endpoint_arg = DeclareLaunchArgument(
        "symovo_endpoint",
        default_value=EnvironmentVariable("SYMOVO_ENDPOINT", default_value="https://192.168.1.100"),
        description="Symovo API endpoint (base URL).",
    )
    amr_id_arg = DeclareLaunchArgument(
        "amr_id",
        default_value=EnvironmentVariable("AMR_ID", default_value="15"),
        description="Symovo AMR ID.",
    )
    tls_verify_arg = DeclareLaunchArgument(
        "tls_verify",
        default_value="false",
        description="Verify TLS certificates for Symovo HTTPS (true/false).",
    )

    workspace_root = os.path.expanduser("~/ros2_ws")
    default_targets_yaml = PythonExpression(["'", workspace_root, "/config/navigation_targets.yaml'"])
    targets_yaml_arg = DeclareLaunchArgument(
        "targets_yaml",
        default_value=EnvironmentVariable("AEHUB_NAV_TARGETS_YAML", default_value=default_targets_yaml),
        description="YAML file with targets: {target_id: station_id}.",
    )

    # --- Action wiring (executor <-> backend) ---
    capability_action_name_arg = DeclareLaunchArgument(
        "capability_action_name",
        default_value="capabilities/navigation/execute",
        description="Action name exposed by backend server and used by executor.",
    )

    # --- Nodes ---
    broker = LifecycleNode(
        package="aehub_broker_credentials",
        executable="broker_credentials_node",
        name="broker_credentials_node",
        namespace="",
        output="screen",
        emulate_tty=True,
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

    executor = LifecycleNode(
        package="aehub_navigation_executor",
        executable="navigation_executor_node",
        name="aehub_navigation_executor",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_id": ParameterValue(LaunchConfiguration("mqtt_robot_id"), value_type=str)},
            {"capability_action_name": LaunchConfiguration("capability_action_name")},
        ],
    )

    backend = LifecycleNode(
        package="aehub_navigation_backend",
        executable="navigation_backend_server",
        name="navigation_backend_server",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"action_name": LaunchConfiguration("capability_action_name")},
            {"symovo_endpoint": LaunchConfiguration("symovo_endpoint")},
            {"amr_id": LaunchConfiguration("amr_id")},
            {"tls_verify": LaunchConfiguration("tls_verify")},
            {"targets_yaml": LaunchConfiguration("targets_yaml")},
        ],
    )

    # --- Autostart lifecycle (simple staged delays) ---
    broker_handlers = _autostart_lifecycle(broker, configure_delay_s=0.5, activate_delay_s=1.0)
    transport_handlers = _autostart_lifecycle(transport, configure_delay_s=1.5, activate_delay_s=2.0)
    proto_handlers = _autostart_lifecycle(proto, configure_delay_s=2.5, activate_delay_s=3.0)
    backend_handlers = _autostart_lifecycle(backend, configure_delay_s=3.5, activate_delay_s=4.0)
    executor_handlers = _autostart_lifecycle(executor, configure_delay_s=4.5, activate_delay_s=5.0)

    return LaunchDescription(
        [
            robot_id_arg,
            mqtt_robot_id_arg,
            config_service_url_arg,
            api_key_arg,
            broker_config_topic_arg,
            mqtt_ca_cert_path_arg,
            symovo_endpoint_arg,
            amr_id_arg,
            tls_verify_arg,
            targets_yaml_arg,
            capability_action_name_arg,
            broker,
            transport,
            proto,
            backend,
            executor,
            *broker_handlers,
            *transport_handlers,
            *proto_handlers,
            *backend_handlers,
            *executor_handlers,
        ]
    )

