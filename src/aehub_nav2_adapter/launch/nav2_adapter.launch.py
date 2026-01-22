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

"""
Launch file for Nav2AdapterNode.

NOTE: Nav2AdapterNode is typically instantiated programmatically by the executor
as a library. This launch file is provided for testing/demonstration purposes.

For production use, the executor creates an instance of Nav2AdapterNode and
manages its lifecycle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Nav2AdapterNode."""

    # Launch arguments
    action_name_arg = DeclareLaunchArgument(
        'action_name',
        default_value='navigate_to_pose',
        description='Nav2 action name for NavigateToPose',
    )

    server_wait_timeout_arg = DeclareLaunchArgument(
        'server_wait_timeout_sec',
        default_value='5.0',
        description='Timeout in seconds to wait for Nav2 action server',
    )

    # Note: Nav2AdapterNode is a library, not a standalone executable.
    # This launch file assumes there's an executable wrapper (not provided in initial implementation).
    # For actual use, the executor would create Nav2AdapterNode programmatically.
    #
    # If you want to test the adapter standalone, you would need to create a minimal
    # executable that instantiates Nav2AdapterNode, sets up event callbacks, and manages lifecycle.

    return LaunchDescription([
        action_name_arg,
        server_wait_timeout_arg,
        # TODO: Add executable node if standalone testing is needed
        # For now, this is a placeholder demonstrating the launch file structure
    ])