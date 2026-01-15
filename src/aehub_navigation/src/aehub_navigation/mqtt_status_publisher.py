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
MQTT Status Publisher

Publishes navigation status to: aroc/robot/{ROBOT_ID}/status/navigation

AE.HUB MVP requirement: UI must be able to show progress.

NOTE: This class no longer manages its own MQTT connection.
It uses MQTTConnectionManager provided by NavigationIntegratedNode.
"""

import json
from datetime import datetime
from typing import Optional

from aehub_navigation.navigation_state_manager import NavigationState
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager


class MQTTStatusPublisher:
    """
    Publishes navigation status via MQTT.
    
    Uses shared MQTTConnectionManager for connection management.
    """
    
    def __init__(self):
        """Initialize status publisher (without ROS2 node)."""
        self.mqtt_manager: Optional[MQTTConnectionManager] = None
        self.robot_id: Optional[str] = None
        self.status_topic: Optional[str] = None
        
        # Current status (will be updated by state manager)
        self.current_status = None
        self.current_target_id = None
        self.current_command_id = None  # Command ID for status correlation
        self.progress_percent = 0
        self.eta_seconds = 0
        self.error_code = None
        self.error_message = None
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    
    def set_mqtt_manager(self, mqtt_manager: MQTTConnectionManager):
        """Set MQTT connection manager (called by NavigationIntegratedNode)."""
        self.mqtt_manager = mqtt_manager
    
    def set_robot_id(self, robot_id: str):
        """Set robot ID and initialize status topic."""
        self.robot_id = robot_id
        self.status_topic = f'aroc/robot/{robot_id}/status/navigation'
    
    def updateStatus(self, state: NavigationState, target_id: str = None,
                     progress_percent: int = 0, eta_seconds: int = 0,
                     error_code: str = None, error_message: str = None,
                     current_position: dict = None, command_id: str = None):
        """Update current status
        
        Args:
            state: Navigation state
            target_id: Target position ID
            progress_percent: Progress percentage (0-100)
            eta_seconds: Estimated time to arrival in seconds
            error_code: Error code if any
            error_message: Error message if any
            current_position: Current robot position
            command_id: Command ID for status correlation
        """
        self.current_status = state.value
        self.current_target_id = target_id
        self.current_command_id = command_id
        self.progress_percent = progress_percent
        self.eta_seconds = eta_seconds
        self.error_code = error_code
        self.error_message = error_message
        if current_position is not None:
            self.current_position = current_position.copy()
    
    def publishStatus(self):
        """Publish status to MQTT in STRICT format (AE.HUB MVP SRS Draft 0.3)"""
        # Ensure status is always set (default to 'idle' if None)
        if not self.current_status:
            self.current_status = NavigationState.IDLE.value
        
        if not self.mqtt_manager or not self.status_topic or not self.robot_id:
            # Log missing requirements for debugging
            if self.mqtt_manager and hasattr(self.mqtt_manager, 'logger') and self.mqtt_manager.logger:
                missing = []
                if not self.current_status:
                    missing.append('current_status')
                if not self.mqtt_manager:
                    missing.append('mqtt_manager')
                if not self.status_topic:
                    missing.append('status_topic')
                if not self.robot_id:
                    missing.append('robot_id')
                self.mqtt_manager.logger.debug(f'  Cannot publish status: missing {", ".join(missing)}')
            return
        
        # NOTE: Keep runtime code free of IDE-specific debug logging (e.g. writing into .cursor/).
        
        # Build payload according to SPECIFICATION.md
        # Ensure status is never None - use 'idle' as default if not set
        status_value = self.current_status if self.current_status else 'idle'
        allowed_status = {"idle", "navigating", "paused", "error", "canceling", "succeeded", "aborted"}
        if status_value not in allowed_status:
            # Fail-safe: never publish unknown status values
            status_value = "error"
            self.error_code = self.error_code or "NAV_UNKNOWN_ERROR"
            self.error_message = self.error_message or f"Invalid status value requested: {self.current_status}"
        
        payload = {
            'schema_version': '1.0',
            'robot_id': self.robot_id,
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            'status': status_value,
            'target_id': self.current_target_id if self.current_target_id else None,
            'command_id': self.current_command_id if self.current_command_id else None,
            'progress_percent': self.progress_percent if self.progress_percent is not None else 0,
            'eta_seconds': self.eta_seconds if self.eta_seconds is not None else 0,
            'current_position': {
                'x': self.current_position.get('x', 0.0),
                'y': self.current_position.get('y', 0.0),
                'theta': self.current_position.get('theta', 0.0)
            },
            'error_code': self.error_code if self.error_code else None,
            'error_message': self.error_message if self.error_message else None
        }
        
        # Log status publication for debugging
        if self.mqtt_manager and hasattr(self.mqtt_manager, 'logger') and self.mqtt_manager.logger:
            self.mqtt_manager.logger.debug(
                f' Publishing status: status={self.current_status}, '
                f'target_id={self.current_target_id}, command_id={self.current_command_id}, '
                f'progress={self.progress_percent}%, eta={self.eta_seconds}s, error_code={self.error_code}'
            )
        
        # Publish via MQTT manager
        try:
            success = self.mqtt_manager.publish(self.status_topic, json.dumps(payload), qos=1)
            if self.mqtt_manager and hasattr(self.mqtt_manager, 'logger') and self.mqtt_manager.logger:
                if success:
                    self.mqtt_manager.logger.debug(f' Status published successfully to {self.status_topic}')
                else:
                    self.mqtt_manager.logger.warn(f'  Failed to publish status to {self.status_topic}')
        except Exception as e:
            # Log error but don't crash - status publishing failure should not break navigation
            if self.mqtt_manager and hasattr(self.mqtt_manager, 'logger') and self.mqtt_manager.logger:
                self.mqtt_manager.logger.error(f' Exception publishing status: {e}', exc_info=True)


# NOTE: MQTTStatusPublisher is no longer a standalone ROS2 node.
# It's used as a component within NavigationIntegratedNode.

