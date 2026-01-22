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
Launch file for broker_credentials_node.

Example usage:
    # Use packaged YAML defaults (recommended):
    ros2 launch aehub_broker_credentials broker_credentials.launch.py

    # Override defaults via launch args (optional):
    ros2 launch aehub_broker_credentials broker_credentials.launch.py \
        config_service_url:=https://mqtt.techvisioncloud.pl \
        api_key:=YOUR_API_KEY
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
import yaml


def _load_defaults_from_packaged_yaml() -> tuple[str, str]:
    """
    Best-effort load defaults from packaged params YAML.

    This keeps the node isolated from external environment (no ENV required).
    """
    try:
        cfg_path = os.path.join(
            get_package_share_directory('aehub_broker_credentials'),
            'config',
            'broker_credentials.yaml',
        )
    except Exception:
        return ('', '')

    try:
        if not os.path.exists(cfg_path):
            return ('', '')
        with open(cfg_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('broker_credentials_node', {}).get('ros__parameters', {}) or {}
        url = str(params.get('config_service_url', '') or '').strip()
        api_key = str(params.get('api_key', '') or '').strip()
        return (url, api_key)
    except Exception:
        return ('', '')


def generate_launch_description():
    """Generate launch description for broker_credentials_node."""
    default_url, default_api_key = _load_defaults_from_packaged_yaml()

    # Declare launch arguments (default to packaged YAML values)
    config_service_url_arg = DeclareLaunchArgument(
        'config_service_url',
        default_value=default_url,
        description=(
            'URL of Config Service (e.g., https://mqtt.techvisioncloud.pl). '
            'Defaults to packaged broker_credentials.yaml.'
        ),
    )

    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value=default_api_key,
        description=(
            'API key for Config Service authentication (X-API-Key header). '
            'Defaults to packaged broker_credentials.yaml.'
        ),
    )

    poll_interval_arg = DeclareLaunchArgument(
        'poll_interval_sec',
        default_value='5.0',
        description='Polling interval in seconds',
    )

    request_timeout_arg = DeclareLaunchArgument(
        'request_timeout_sec',
        default_value='2.0',
        description='HTTP request timeout in seconds',
    )

    fail_on_startup_arg = DeclareLaunchArgument(
        'fail_on_startup',
        default_value='false',
        description='Fail activation if initial fetch fails',
    )

    # Create lifecycle node
    broker_credentials_node = LifecycleNode(
        package='aehub_broker_credentials',
        executable='broker_credentials_node',
        name='broker_credentials_node',
        namespace='',
        parameters=[{
            'config_service_url': LaunchConfiguration('config_service_url'),
            'api_key': LaunchConfiguration('api_key'),
            'poll_interval_sec': LaunchConfiguration('poll_interval_sec'),
            'request_timeout_sec': LaunchConfiguration('request_timeout_sec'),
            'fail_on_startup': LaunchConfiguration('fail_on_startup'),
        }],
        output='screen',
        emulate_tty=True,  # Better log output
    )

    return LaunchDescription([
        config_service_url_arg,
        api_key_arg,
        poll_interval_arg,
        request_timeout_arg,
        fail_on_startup_arg,
        LogInfo(msg='Broker credentials node launched (use lifecycle to configure/activate).'),
        broker_credentials_node,
    ])
