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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_001",
        description="Robot identifier used for ROS namespace: /robot/<id>/...",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("aehub_robot_readiness"), "config", "robot_interface_readiness_symovo.yaml"]
        ),
        description="Params file for base_controller + readiness monitor.",
    )

    ns = PathJoinSubstitution(["robot", LaunchConfiguration("robot_id")])

    base_controller = Node(
        package="base_controller",
        executable="base_controller_node",
        name="base_controller",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    readiness_monitor = Node(
        package="aehub_robot_readiness",
        executable="robot_readiness_monitor_node",
        name="robot_readiness_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            robot_id_arg,
            params_file_arg,
            GroupAction(
                [
                    PushRosNamespace(ns),
                    base_controller,
                    readiness_monitor,
                ]
            ),
        ]
    )

