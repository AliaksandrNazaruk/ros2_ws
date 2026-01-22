# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Gazebo SITL profile (simulation replaces robot completely).
#
# Starts:
# - Gazebo + simulated robot (scan/odom/tf) via aehub_nav2_gazebo/gazebo_nav2.launch.py (without navigation_integrated_node)
# - Nav2 bringup (AMCL + map) from aehub_nav2_gazebo
# - Clean MQTT stack:
#   broker_credentials -> mqtt_transport -> mqtt_protocol_adapter -> navigation_executor -> nav2_capability_server
#
# Intended use:
# - Run on a capable PC (Gazebo host), headless or GUI.
# - Send MQTT navigateTo commands to robot_sim_001 and observe result success.

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

    # Force FastDDS to use UDP-only transport (disable SHM) for robustness.
    udp_only_profile = os.path.join(
        get_package_share_directory("aehub_navigation_executor"),
        "config",
        "fastdds_udp_only.xml",
    )
    env_disable_shm = [
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", udp_only_profile),
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
    )
    api_key_arg = DeclareLaunchArgument(
        "api_key",
        default_value=EnvironmentVariable("CONFIG_SERVICE_API_KEY", default_value=default_api_key or ""),
    )
    broker_config_topic_arg = DeclareLaunchArgument("broker_config_topic", default_value="infra/mqtt/broker_config")

    # --- MQTT TLS CA ---
    mqtt_ca_cert_path_arg = DeclareLaunchArgument(
        "mqtt_ca_cert_path",
        default_value=EnvironmentVariable("MQTT_CA_CERT", default_value="/home/boris/ros2_ws/ca.crt"),
    )

    # --- Nav2 / Gazebo args (passed through) ---
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    gui_arg = DeclareLaunchArgument("gui", default_value="false")
    world_arg = DeclareLaunchArgument("world", default_value="")
    map_arg = DeclareLaunchArgument("map", default_value="")
    params_file_arg = DeclareLaunchArgument("params_file", default_value="")
    nav2_autostart_arg = DeclareLaunchArgument("nav2_autostart", default_value="true")

    # --- Capability action wiring ---
    capability_action_name_arg = DeclareLaunchArgument(
        "capability_action_name",
        default_value="capabilities/navigation/execute",
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

    # Provide robot readiness in SITL (always "ready" by default).
    sim_robot_status = Node(
        package="aehub_navigation_executor",
        executable="sim_robot_status_publisher",
        name="sim_robot_status_publisher",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"blocked": False},
            {"publish_hz": 2.0},
            {"robot_status_topic": "robot/status"},
            # Gazebo provides odom; do not publish fake odom/amcl_pose here.
            {"publish_fake_odom": False},
            {"publish_fake_amcl_pose": False},
        ],
    )

    # Headless initial pose for AMCL (equivalent to RViz "set initial pose").
    sim_initial_pose = Node(
        package="aehub_navigation_executor",
        executable="sim_initial_pose_publisher",
        name="sim_initial_pose_publisher",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"x": 0.0},
            {"y": 0.0},
            {"theta": 0.0},
            {"frame_id": "map"},
            {"delay_s": 3.0},
            {"topic": "initialpose"},
        ],
    )

    # Gazebo + Nav2 bringup (disable legacy integrated gateway).
    aehub_nav2_gazebo_share = get_package_share_directory("aehub_nav2_gazebo")
    gazebo_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aehub_nav2_gazebo_share, "launch", "gazebo_nav2.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "gui": LaunchConfiguration("gui"),
            "world": LaunchConfiguration("world"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("nav2_autostart"),
            "launch_navigation_integrated_node": "false",
        }.items(),
    )

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
            gui_arg,
            world_arg,
            map_arg,
            params_file_arg,
            nav2_autostart_arg,
            capability_action_name_arg,
            gazebo_nav2,
            sim_robot_status,
            sim_initial_pose,
            broker,
            transport,
            proto,
            executor,
            capability,
            capability_lifecycle_manager,
            *handlers,
        ]
    )

