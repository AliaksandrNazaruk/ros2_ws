# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Loopback (no Gazebo) simulation profile that uses the *clean* MQTT stack.
#
# Starts:
# - aehub_broker_credentials (Lifecycle)
# - aehub_mqtt_transport (Lifecycle)
# - aehub_mqtt_protocol_adapter (Lifecycle)
# - aehub_navigation_executor (Lifecycle)
# - aehub_nav2_capability_server (Lifecycle, managed by nav2_lifecycle_manager)
# - Nav2 loopback simulator via nav2_bringup/tb3_loopback_simulation.launch.py
#
# Notes:
# - Runs everything in the root namespace.
# - Intended for end-to-end MQTT behavior testing (ack/state/result) on a virtual robot_id.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
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

    # Force FastDDS to use UDP-only transport (disable SHM).
    # This avoids common "open_and_lock_file failed" failures on /dev/shm.
    udp_only_profile = os.path.join(
        get_package_share_directory("aehub_navigation_executor"),
        "config",
        "fastdds_udp_only.xml",
    )
    env_disable_shm = [
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", udp_only_profile),
        # Keep the legacy switch too (harmless if ignored by this distro).
        SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
    ]

    # --- IDs / MQTT wiring ---
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=EnvironmentVariable("AEHUB_ROBOT_ID", default_value="robot_sim_001"),
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

    # --- Nav2 loopback sim args ---
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    nav2_autostart_arg = DeclareLaunchArgument("nav2_autostart", default_value="true")

    nav2_share = get_package_share_directory("nav2_bringup")
    default_map = os.path.join(nav2_share, "maps", "tb3_sandbox.yaml")
    default_params = os.path.join(nav2_share, "params", "nav2_params.yaml")

    nav2_map_arg = DeclareLaunchArgument(
        "nav2_map",
        default_value=EnvironmentVariable("NAV2_MAP", default_value=default_map),
        description="Map yaml used by Nav2 loopback sim.",
    )
    nav2_params_file_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=EnvironmentVariable("NAV2_PARAMS_FILE", default_value=default_params),
        description="Nav2 params yaml used by Nav2 loopback sim.",
    )

    # --- Sim robot readiness ---
    sim_blocked_arg = DeclareLaunchArgument(
        "sim_blocked",
        default_value="false",
        description="If true, sim publishes blocked RobotStatus (motors disabled / estop active).",
    )

    # --- Capability action wiring ---
    capability_action_name_arg = DeclareLaunchArgument(
        "capability_action_name",
        default_value="capabilities/navigation/execute",
        description="Action name exposed by capability server and used by executor.",
    )

    # --- Nodes (clean MQTT stack) ---
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

    # Capability server (multi-node process): launch as plain Node (no __node remap).
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

    sim_robot_status = Node(
        package="aehub_navigation_executor",
        executable="sim_robot_status_publisher",
        name="sim_robot_status_publisher",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"blocked": LaunchConfiguration("sim_blocked")},
            {"publish_hz": 2.0},
            {"robot_status_topic": "robot/status"},
            # Loopback sim already publishes /odom; keep fake odom off to avoid conflicts.
            {"publish_fake_odom": False},
            # Loopback sim doesn't run AMCL; publish minimal amcl_pose evidence for readiness.
            {"publish_fake_amcl_pose": True},
        ],
    )

    # --- Nav2 loopback simulator ---
    loopback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "tb3_loopback_simulation.launch.py")),
        launch_arguments={
            "use_rviz": "False",
            "use_robot_state_pub": "True",
            "map": LaunchConfiguration("nav2_map"),
            "params_file": LaunchConfiguration("nav2_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("nav2_autostart"),
        }.items(),
    )

    # Conservative TF fix for loopback sim: identity map->odom.
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Ensure odom->base_footprint exists so the TF tree is connected:
    # map -> odom (static), odom -> base_footprint (static), base_footprint -> base_link (URDF).
    odom_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_base_footprint",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
    )

    # --- Lifecycle autostart (staggered) ---
    handlers = []
    handlers += _autostart_lifecycle(broker, configure_delay_s=0.5, activate_delay_s=1.0)
    handlers += _autostart_lifecycle(transport, configure_delay_s=1.5, activate_delay_s=2.0)
    handlers += _autostart_lifecycle(proto, configure_delay_s=2.5, activate_delay_s=3.0)
    handlers += _autostart_lifecycle(executor, configure_delay_s=3.5, activate_delay_s=4.0)

    return LaunchDescription(
        [
            *env_disable_shm,
            robot_id_arg,
            mqtt_robot_id_arg,
            config_service_url_arg,
            api_key_arg,
            broker_config_topic_arg,
            mqtt_ca_cert_path_arg,
            use_sim_time_arg,
            nav2_autostart_arg,
            nav2_map_arg,
            nav2_params_file_arg,
            sim_blocked_arg,
            capability_action_name_arg,
            broker,
            transport,
            proto,
            executor,
            capability,
            capability_lifecycle_manager,
            sim_robot_status,
            loopback,
            map_to_odom_tf,
            odom_to_base_tf,
            *handlers,
        ]
    )

