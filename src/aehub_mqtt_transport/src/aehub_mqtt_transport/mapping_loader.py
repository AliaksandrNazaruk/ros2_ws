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
Mapping configuration loader and validator.

Loads and validates YAML mapping configuration files.
"""

import yaml
from pathlib import Path
from typing import Dict, Any, List
import os

# Supported schema version
SUPPORTED_SCHEMA_VERSION = 1


class MappingSchemaError(Exception):
    """Raised when mapping schema validation fails."""
    pass


def load_mapping_config(config_path: str) -> Dict[str, Any]:
    """
    Load and validate mapping configuration from YAML file.
    
    Args:
        config_path: Path to YAML mapping configuration file
        
    Returns:
        Validated mapping configuration dictionary
        
    Raises:
        MappingSchemaError: If schema validation fails
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If YAML parsing fails
    """
    # Load YAML file
    config_file = Path(config_path)
    if not config_file.exists():
        raise FileNotFoundError(f"Mapping config file not found: {config_path}")
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    if not config:
        raise MappingSchemaError("Mapping config file is empty")
    
    # Validate schema
    validate_schema(config)
    
    return config


def validate_schema(config: Dict[str, Any]) -> None:
    """
    Validate mapping schema structure and version.
    
    Args:
        config: Mapping configuration dictionary
        
    Raises:
        MappingSchemaError: If validation fails
    """
    # Check schema section
    if 'schema' not in config:
        raise MappingSchemaError("Missing 'schema' section in mapping config")
    
    schema = config['schema']
    
    # Check schema name
    if 'name' not in schema:
        raise MappingSchemaError("Missing 'schema.name' in mapping config")
    
    if schema['name'] != 'aehub-mqtt-mapping':
        raise MappingSchemaError(
            f"Unexpected schema name: {schema['name']}. Expected: 'aehub-mqtt-mapping'"
        )
    
    # Check schema version
    if 'version' not in schema:
        raise MappingSchemaError("Missing 'schema.version' in mapping config")
    
    version = schema['version']
    if not isinstance(version, int):
        raise MappingSchemaError(f"Schema version must be integer, got: {type(version)}")
    
    if version != SUPPORTED_SCHEMA_VERSION:
        raise MappingSchemaError(
            f"Unsupported mapping schema version: {version}. "
            f"Supported version: {SUPPORTED_SCHEMA_VERSION}"
        )
    
    # Check topics section
    if 'topics' not in config:
        raise MappingSchemaError("Missing 'topics' section in mapping config")
    
    topics = config['topics']
    if not isinstance(topics, dict):
        raise MappingSchemaError("'topics' must be a dictionary")
    
    # Validate each topic mapping
    for category, category_topics in topics.items():
        if not isinstance(category_topics, dict):
            raise MappingSchemaError(
                f"Topic category '{category}' must be a dictionary"
            )
        
        for topic_name, topic_config in category_topics.items():
            validate_topic_mapping(topic_name, topic_config)


def validate_topic_mapping(topic_name: str, topic_config: Dict[str, Any]) -> None:
    """
    Validate individual topic mapping configuration.
    
    Args:
        topic_name: Name of the topic mapping
        topic_config: Topic mapping configuration dictionary
        
    Raises:
        MappingSchemaError: If validation fails
    """
    # Check direction
    if 'direction' not in topic_config:
        raise MappingSchemaError(
            f"Missing 'direction' in topic mapping '{topic_name}'"
        )
    
    direction = topic_config['direction']
    if direction not in ['inbound', 'outbound']:
        raise MappingSchemaError(
            f"Invalid direction '{direction}' in topic mapping '{topic_name}'. "
            f"Must be 'inbound' or 'outbound'"
        )
    
    # Check MQTT configuration
    if 'mqtt' not in topic_config:
        raise MappingSchemaError(
            f"Missing 'mqtt' section in topic mapping '{topic_name}'"
        )
    
    mqtt_config = topic_config['mqtt']
    if 'topic' not in mqtt_config:
        raise MappingSchemaError(
            f"Missing 'mqtt.topic' in topic mapping '{topic_name}'"
        )
    
    # Check ROS configuration
    if 'ros' not in topic_config:
        raise MappingSchemaError(
            f"Missing 'ros' section in topic mapping '{topic_name}'"
        )
    
    ros_config = topic_config['ros']
    if 'name' not in ros_config:
        raise MappingSchemaError(
            f"Missing 'ros.name' in topic mapping '{topic_name}'"
        )
    
    if 'msg_type' not in ros_config:
        raise MappingSchemaError(
            f"Missing 'ros.msg_type' in topic mapping '{topic_name}'"
        )


def get_config_path_from_share() -> str:
    """
    Get default mapping config path from ROS2 share directory.
    
    Returns:
        Path to default mapping config file
        
    Raises:
        FileNotFoundError: If default config file doesn't exist
    """
    # Try to find config in ROS2 share directory
    package_name = 'aehub_mqtt_transport'
    config_file = 'mqtt_mapping.v1.yaml'
    
    # Check share directory (typical ROS2 install location)
    share_paths = [
        os.path.join(os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0] if os.environ.get('AMENT_PREFIX_PATH') else '', 'share', package_name, 'config', config_file),
        os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'jazzy'), 'share', package_name, 'config', config_file),
    ]
    
    # Also check relative to workspace
    workspace_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', config_file)
    share_paths.insert(0, os.path.abspath(workspace_path))
    
    for path in share_paths:
        if os.path.exists(path):
            return path
    
    raise FileNotFoundError(
        f"Default mapping config file not found. "
        f"Expected at: {share_paths[0]} or in ROS2 share directory"
    )
