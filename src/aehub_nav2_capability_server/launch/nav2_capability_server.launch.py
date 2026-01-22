# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    capability_action_name = DeclareLaunchArgument(
        "capability_action_name",
        default_value="capabilities/navigation/execute",
        description="NavigationExecute action name exposed by capability server",
    )
    goal_frame = DeclareLaunchArgument(
        "goal_frame",
        default_value="map",
        description="Frame for goal PoseStamped (default: map)",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="cmd_vel topic for STOP burst (owned by capability)",
    )
    stop_burst_enabled = DeclareLaunchArgument(
        "stop_burst_enabled",
        default_value="true",
        description="Enable STOP burst on cancel (safety policy)",
    )
    stop_burst_duration_s = DeclareLaunchArgument(
        "stop_burst_duration_s",
        default_value="0.6",
        description="STOP burst duration in seconds",
    )
    stop_burst_rate_hz = DeclareLaunchArgument(
        "stop_burst_rate_hz",
        default_value="20.0",
        description="STOP burst publish rate (Hz)",
    )

    node = LifecycleNode(
        package="aehub_nav2_capability_server",
        executable="aehub_nav2_capability_server",
        name="aehub_nav2_capability_server",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"capability_action_name": LaunchConfiguration("capability_action_name")},
            {"goal_frame": LaunchConfiguration("goal_frame")},
            {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
            {"stop_burst_enabled": LaunchConfiguration("stop_burst_enabled")},
            {"stop_burst_duration_s": LaunchConfiguration("stop_burst_duration_s")},
            {"stop_burst_rate_hz": LaunchConfiguration("stop_burst_rate_hz")},
        ],
    )

    return LaunchDescription(
        [
            capability_action_name,
            goal_frame,
            cmd_vel_topic,
            stop_burst_enabled,
            stop_burst_duration_s,
            stop_burst_rate_hz,
            node,
        ]
    )

