#!/usr/bin/env python3

"""
Simulation profile (headless) using Nav2 loopback simulator.

This launch is intended to run on the same host as the real stack, but in a
separate ROS_DOMAIN_ID (recommended) to avoid graph collisions.

It starts:
- nav2_bringup tb3_loopback_simulation (no RViz)
- navigation_integrated_node (MQTT gateway) configured for sim robot_id
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")

    robot_id = LaunchConfiguration("robot_id")
    config_service_url = LaunchConfiguration("config_service_url")
    config_service_api_key = LaunchConfiguration("config_service_api_key")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    loopback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "tb3_loopback_simulation.launch.py")),
        launch_arguments={
            "use_rviz": "False",
            "use_robot_state_pub": "True",
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    # Conservative TF fix for loopback sim:
    # Provide identity map->odom so Nav2 global frame is usable without AMCL.
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    gateway = Node(
        package="aehub_navigation",
        executable="navigation_integrated_node",
        name="navigation_integrated_node",
        output="screen",
        parameters=[
            {
                "robot_id": robot_id,
                "config_service_url": config_service_url,
                "config_service_api_key": config_service_api_key,
                # sim: use /odom as pose source
                "pose_source": "odom",
                "pose_topic": "odom",
                # sim: no symovo readiness checks
                "symovo_ready_check_enabled": False,
                "symovo_ready_check_fail_open": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value=EnvironmentVariable("ROBOT_ID_SIM", default_value="fahrdummy-01-local_sim"),
            ),
            DeclareLaunchArgument(
                "config_service_url",
                default_value=EnvironmentVariable("CONFIG_SERVICE_URL", default_value="http://localhost:7900"),
            ),
            DeclareLaunchArgument(
                "config_service_api_key",
                default_value=EnvironmentVariable("CONFIG_SERVICE_API_KEY", default_value=""),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=EnvironmentVariable(
                    "NAV2_MAP",
                    default_value=os.path.join(bringup_dir, "maps", "tb3_sandbox.yaml"),
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=EnvironmentVariable(
                    "NAV2_PARAMS_FILE",
                    default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
                ),
            ),
            loopback,
            map_to_odom_tf,
            gateway,
        ]
    )

