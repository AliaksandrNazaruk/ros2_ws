# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=EnvironmentVariable("AEHUB_ROBOT_ID", default_value=""),
        description="Robot ID (metadata for outbound event envelope)",
    )

    return LaunchDescription(
        [
            robot_id_arg,
            LifecycleNode(
                package="aehub_mqtt_protocol_adapter",
                executable="mqtt_protocol_adapter_node",
                name="mqtt_protocol_adapter_node",
                namespace="",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"robot_id": LaunchConfiguration("robot_id")},
                ],
            ),
        ]
    )

