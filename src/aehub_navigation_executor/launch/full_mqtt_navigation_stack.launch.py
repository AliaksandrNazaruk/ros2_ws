# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Complete MQTT navigation stack bringup for Symovo robot.
#
# Starts:
# - aehub_broker_credentials (Lifecycle)
# - aehub_mqtt_transport (Lifecycle)
# - aehub_mqtt_protocol_adapter (Lifecycle)
# - aehub_navigation_executor (Lifecycle)
# - aehub_nav2_capability_server (Lifecycle)
# - Symovo + Nav2 stack via aehub_navigation/symovo_nav2.launch.py
#   (base_controller, map_server+amcl, symovo_map_mirror, localization orchestrator, tf hygiene, etc.)
#
# Notes:
# - Runs everything in the root namespace (no /robot/<id>/...).
# - Nav2 params should be provided via nav2_params_file for your robot.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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
    # Chain activation only after successful CONFIGURE (node reaches INACTIVE).
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

    # --- Common / IDs ---
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

    # --- Symovo (passed through to symovo_nav2.launch.py) ---
    symovo_endpoint_arg = DeclareLaunchArgument(
        "symovo_endpoint",
        default_value=EnvironmentVariable("SYMOVO_ENDPOINT", default_value="https://192.168.1.100"),
        description="Symovo API endpoint for base_controller + map mirror.",
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
    launch_base_controller_arg = DeclareLaunchArgument(
        "launch_base_controller",
        default_value="true",
        description="Launch base_controller node.",
    )
    use_legacy_map_fetcher_arg = DeclareLaunchArgument(
        "use_legacy_map_fetcher",
        default_value="false",
        description="If true, do not run symovo_map_mirror/localization orchestrator.",
    )

    workspace_root = os.path.expanduser("~/ros2_ws")
    default_map_output_dir = PythonExpression(["'", workspace_root, "/maps/symovo_map'"])
    map_output_dir_arg = DeclareLaunchArgument(
        "map_output_dir",
        default_value=default_map_output_dir,
        description="Where symovo_map_mirror writes map.pgm/map.yaml.",
    )

    # --- Nav2 params (passed through to symovo_nav2.launch.py as 'params_file') ---
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    nav2_autostart_arg = DeclareLaunchArgument("nav2_autostart", default_value="true")

    # Prefer installed aehub_navigation config, fallback to workspace_root/config for dev.
    try:
        _aehub_nav_share = get_package_share_directory("aehub_navigation")
        _default_params_path = os.path.join(_aehub_nav_share, "config", "nav2_symovo_params.yaml")
    except Exception:
        _default_params_path = ""

    if not _default_params_path or not os.path.exists(_default_params_path):
        default_nav2_params_file = PythonExpression(["'", workspace_root, "/config/nav2_symovo_params.yaml'"])
    else:
        default_nav2_params_file = _default_params_path

    nav2_params_file_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=default_nav2_params_file,
        description="Nav2 params YAML for Symovo (default: aehub_navigation/config/nav2_symovo_params.yaml).",
    )

    # --- Capability action wiring ---
    capability_action_name_arg = DeclareLaunchArgument(
        "capability_action_name",
        default_value="capabilities/navigation/execute",
        description="Action name exposed by capability server and used by executor.",
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

    # IMPORTANT:
    # aehub_nav2_capability_server executable hosts multiple ROS nodes in one process.
    # If we use launch_ros LifecycleNode with 'name', launch_ros remaps __node for the whole process
    # (renaming all nodes) which breaks the action server. So we launch it as a plain Node
    # (no __node remap), and drive lifecycle transitions via ros2cli.
    capability = Node(
        package="aehub_nav2_capability_server",
        executable="aehub_nav2_capability_server",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"capability_action_name": LaunchConfiguration("capability_action_name")},
            {"goal_frame": "map"},
            {"cmd_vel_topic": "cmd_vel"},
        ],
    )

    # Lifecycle management for capability server (do NOT use ros2cli here).
    # This executable hosts multiple nodes in one process, so we avoid __node remapping.
    # nav2_lifecycle_manager uses lifecycle services and retries until the node is up.
    capability_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_capability",
        namespace="",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"bond_timeout": 10.0},
            {"node_names": ["aehub_nav2_capability_server"]},
        ],
    )

    # Bring up Symovo + Nav2 stack (base_controller, map_server+amcl, map mirror, etc.)
    # Disable legacy MQTT gateway inside that launch (we use clean MQTT transport stack here).
    symovo_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("aehub_navigation"), "launch", "symovo_nav2.launch.py"])
        ),
        launch_arguments={
            "namespace": "",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("nav2_params_file"),
            "autostart": LaunchConfiguration("nav2_autostart"),
            "symovo_endpoint": LaunchConfiguration("symovo_endpoint"),
            "amr_id": LaunchConfiguration("amr_id"),
            "tls_verify": LaunchConfiguration("tls_verify"),
            "launch_base_controller": LaunchConfiguration("launch_base_controller"),
            "launch_navigation_integrated_node": "false",
            "use_legacy_map_fetcher": LaunchConfiguration("use_legacy_map_fetcher"),
            "map_output_dir": LaunchConfiguration("map_output_dir"),
            # Give localization time to publish map->odom before Nav2 navigation bringup.
            "nav2_bringup_delay_s": "20.0",
            "log_level": "info",
        }.items(),
    )

    # --- Lifecycle autostart (staggered) ---
    handlers = []
    handlers += _autostart_lifecycle(broker, configure_delay_s=0.5, activate_delay_s=1.0)
    handlers += _autostart_lifecycle(transport, configure_delay_s=1.5, activate_delay_s=2.0)
    handlers += _autostart_lifecycle(proto, configure_delay_s=2.5, activate_delay_s=3.0)
    handlers += _autostart_lifecycle(executor, configure_delay_s=3.5, activate_delay_s=4.0)
    # symovo_nav2_launch manages its own Nav2/map lifecycle.

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
            launch_base_controller_arg,
            use_legacy_map_fetcher_arg,
            map_output_dir_arg,
            use_sim_time_arg,
            nav2_autostart_arg,
            nav2_params_file_arg,
            capability_action_name_arg,
            # nodes
            broker,
            transport,
            proto,
            executor,
            capability,
            capability_lifecycle_manager,
            symovo_nav2_launch,
            *handlers,
        ]
    )

