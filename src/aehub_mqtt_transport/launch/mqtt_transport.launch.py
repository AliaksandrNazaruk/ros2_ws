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
Launch file for mqtt_transport_node.

Clean ROS2 launch best practices:
- No os.environ direct access
- Only arguments + systemd EnvironmentFile
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    """Generate launch description for mqtt_transport_node."""
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value=EnvironmentVariable('AEHUB_ROBOT_ID', default_value=''),
        description='Robot ID for MQTT topics'
    )

    return LaunchDescription([
        robot_id_arg,
        
        LifecycleNode(
            package='aehub_mqtt_transport',
            executable='mqtt_transport_node',
            name='mqtt_transport_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
            }],
        )
    ])
