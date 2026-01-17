#!/usr/bin/env python3

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
Launch file for AE.HUB Navigation

Launches integrated navigation node with all components.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('aehub_navigation')
    
    # Default positions file
    default_positions_file = os.path.join(pkg_dir, 'config', 'positions.yaml')
    
    # Launch arguments
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_001',
        description='Robot ID for MQTT topics'
    )
    
    # NOTE: mqtt_broker and mqtt_port are DEPRECATED
    # MQTT configuration is now fetched from centralized Broker Config Service
    declare_config_service_url = DeclareLaunchArgument(
        'config_service_url',
        default_value='http://localhost:7900',
        description='URL of Broker Config Service (centralized MQTT configuration)'
    )
    
    declare_config_service_api_key = DeclareLaunchArgument(
        'config_service_api_key',
        default_value='',
        description='API key for Broker Config Service authentication (REQUIRED)'
    )
    
    declare_config_poll_interval = DeclareLaunchArgument(
        'config_poll_interval',
        default_value='5.0',
        description='Interval for polling Config Service for changes (seconds)'
    )
    
    declare_positions_file = DeclareLaunchArgument(
        'positions_file',
        default_value=default_positions_file,
        description='Path to positions YAML file'
    )
    
    # Integrated navigation node
    navigation_node = Node(
        package='aehub_navigation',
        executable='navigation_integrated_node',
        name='navigation_integrated_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'config_service_url': LaunchConfiguration('config_service_url'),
            'config_service_api_key': LaunchConfiguration('config_service_api_key'),
            'config_poll_interval': LaunchConfiguration('config_poll_interval'),
            'positions_file': LaunchConfiguration('positions_file'),
        }]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_robot_id)
    ld.add_action(declare_config_service_url)
    ld.add_action(declare_config_service_api_key)
    ld.add_action(declare_config_poll_interval)
    ld.add_action(declare_positions_file)
    ld.add_action(navigation_node)
    
    return ld

