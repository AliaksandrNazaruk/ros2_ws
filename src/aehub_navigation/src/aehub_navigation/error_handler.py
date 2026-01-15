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
Navigation Error Handler

Unified error handling for navigation system.
Provides standardized error logging and optional MQTT error publishing.

AE.HUB MVP: Ensures consistent error handling across all components.

NOTE: This is a pure Python module, NOT a ROS2 Node.
"""

from typing import Optional, Dict, Any


class NavigationErrorHandler:
    """
    Unified error handler for navigation system.
    
    Provides:
    - Standardized error logging
    - Error code classification
    - Optional MQTT error publishing
    """
    
    # Standard error codes (AE.HUB MVP)
    # These codes match SPECIFICATION.md Section 5.9 and Section 3.3
    # All error codes published via MQTT must be from this set or documented extensions
    ERROR_CODES = {
        # Command validation errors (Section 5.5)
        'NAV_INVALID_COMMAND': 'Invalid navigation command format or content',
        'NAV_DUPLICATE_COMMAND': 'Command with same command_id already processed',
        'NAV_RATE_LIMIT_EXCEEDED': 'Command rate limit exceeded',
        'NAV_INVALID_TARGET': 'Target position ID not found or invalid',
        
        # Nav2 server errors (Section 5.8)
        'NAV_SERVER_UNAVAILABLE': 'Nav2 action server not available',
        'NAV_GOAL_REJECTED': 'Nav2 rejected the navigation goal',
        'NAV_GOAL_ABORTED': 'Navigation goal was aborted',
        'NAV_GOAL_CANCELLED': 'Navigation goal was cancelled',
        'NAV_TIMEOUT': 'Navigation goal timed out',
        
        # Nav2 detailed error codes (from nav2_msgs/action/NavigateToPose.action)
        'NAV_NO_VALID_CONTROL': 'No valid control available (base may be blocked or emergency stop active)',
        'NAV_FAILED_TO_MAKE_PROGRESS': 'Failed to make progress towards goal',
        'NAV_PATIENCE_EXCEEDED': 'Patience exceeded while waiting for progress',
        'NAV_INVALID_PATH': 'Invalid path to goal',
        'NAV_CONTROLLER_TIMED_OUT': 'Controller timed out',
        'NAV_TF_ERROR': 'Transform error',
        'NAV_INVALID_CONTROLLER': 'Invalid controller',
        
        # MQTT connection errors (Section 5.1)
        'NAV_MQTT_ERROR': 'MQTT connection error',

        # Hardware / robot readiness
        'NAV_ROBOT_NOT_READY': 'Robot is not ready to drive (manual mode / not drive-ready / charging)',
        
        # Generic errors
        'NAV_UNKNOWN_ERROR': 'Unknown error occurred',
    }
    
    def __init__(self, logger, status_publisher=None):
        """
        Initialize error handler.
        
        Args:
            logger: ROS2 logger instance (for logging)
            status_publisher: Optional MQTTStatusPublisher (for error publishing)
        """
        self._logger = logger
        self._status_publisher = status_publisher
    
    def handle_error(
        self,
        error_type: str,
        context: Dict[str, Any],
        command_id: Optional[str] = None
    ):
        """
        Handle error with standardized logging and optional MQTT publishing.
        
        Args:
            error_type: Error type (key from ERROR_CODES or custom)
            context: Context dictionary with error details
            command_id: Optional command ID for error correlation
        """
        # Get error message
        error_message = self.ERROR_CODES.get(
            error_type,
            f'Error: {error_type}'
        )
        
        # Add context details to error message
        if context:
            context_str = ', '.join(f'{k}={v}' for k, v in context.items())
            error_message = f'{error_message} ({context_str})'
        
        # Log error
        self.log_error(error_type, error_message, context, command_id)
        
        # Publish error to MQTT if publisher is available
        if self._status_publisher:
            self._logger.info(f' [ERROR HANDLER] Publishing error: error_type={error_type}, command_id={command_id}')
            target_id = None
            try:
                target_id = context.get('target_id') if isinstance(context, dict) else None
            except Exception:
                target_id = None
            self.publish_error(error_type, error_message, command_id, target_id=target_id)
        else:
            self._logger.warn(f'  [ERROR HANDLER] Status publisher not available, cannot publish error: error_type={error_type}')
    
    def log_error(
        self,
        error_code: str,
        error_message: str,
        context: Optional[Dict[str, Any]] = None,
        command_id: Optional[str] = None
    ):
        """
        Log error with standardized format.
        
        Args:
            error_code: Error code (standardized)
            error_message: Human-readable error message
            context: Optional context dictionary
            command_id: Optional command ID for correlation
        """
        log_msg = f' [{error_code}] {error_message}'
        if command_id:
            log_msg += f' (command_id: {command_id})'
        if context:
            log_msg += f' | Context: {context}'
        
        self._logger.error(log_msg)
    
    def log_exception(
        self,
        exception: Exception,
        context: Optional[Dict[str, Any]] = None,
        command_id: Optional[str] = None
    ):
        """
        Log exception with full traceback.
        
        Args:
            exception: Exception instance
            context: Optional context dictionary
            command_id: Optional command ID for correlation
        """
        error_code = 'NAV_UNKNOWN_ERROR'
        error_message = str(exception)
        
        log_msg = f' [{error_code}] Exception: {error_message}'
        if command_id:
            log_msg += f' (command_id: {command_id})'
        if context:
            log_msg += f' | Context: {context}'
        
        self._logger.error(log_msg, exc_info=True)
    
    def publish_error(
        self,
        error_code: str,
        error_message: str,
        command_id: Optional[str] = None,
        target_id: Optional[str] = None,
    ):
        """
        Publish error to MQTT via status publisher.
        
        Args:
            error_code: Error code (standardized)
            error_message: Human-readable error message
            command_id: Optional command ID for correlation
        """
        if not self._status_publisher:
            return
        
        try:
            from aehub_navigation.navigation_state_manager import NavigationState
            # Try also to publish command result event if available
            try:
                if hasattr(self._status_publisher, 'mqtt_manager') and hasattr(self._status_publisher, 'robot_id'):
                    from aehub_navigation.mqtt_command_event_publisher import MQTTCommandEventPublisher
                    event_pub = MQTTCommandEventPublisher()
                    event_pub.set_mqtt_manager(self._status_publisher.mqtt_manager)
                    event_pub.set_robot_id(self._status_publisher.robot_id)
                    event_pub.publish_result(
                        command_id or 'unknown',
                        target_id,
                        result_type='error',
                        error_code=error_code,
                        error_message=error_message,
                    )
            except Exception:
                pass
            
            # Update status publisher with error
            self._status_publisher.updateStatus(
                state=NavigationState.ERROR,
                error_code=error_code,
                error_message=error_message,
                command_id=command_id
            )
            
            # Publish error status
            self._status_publisher.publishStatus()
            
            self._logger.info(
                f' [ERROR HANDLER] Published error status: error_code={error_code}, '
                f'error_message={error_message}, command_id={command_id}'
            )
            
        except Exception as e:
            self._logger.error(
                f' Failed to publish error status: {e}',
                exc_info=True
            )
    
    @staticmethod
    def get_error_code_description(error_code: str) -> str:
        """
        Get human-readable description for error code.
        
        Args:
            error_code: Error code
        
        Returns:
            Human-readable description
        """
        return NavigationErrorHandler.ERROR_CODES.get(
            error_code,
            f'Unknown error code: {error_code}'
        )

