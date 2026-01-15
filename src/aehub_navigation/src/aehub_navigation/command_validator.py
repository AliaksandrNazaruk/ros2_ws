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
Command Validator

Validates navigation commands with comprehensive checks:
- Format validation (JSON schema)
- Required fields validation (command_id, timestamp, target_id, priority)
- UUID v4 format validation for command_id
- Duplicate command_id detection (in-memory set with TTL)

AE.HUB MVP: Ensures command integrity before processing.

NOTE: This is a pure Python module, NOT a ROS2 Node.
Thread-safe for concurrent access.
"""

import re
import sys
import threading
import time
import math
from typing import Optional, Tuple, Dict


class CommandValidator:
    """
    Validates navigation commands and manages duplicate detection.
    
    Thread-safe implementation using locks for concurrent access.
    Uses TTL-based cleanup for processed command IDs.
    """
    
    def __init__(self, max_processed_ids: int = 1000, ttl_seconds: int = 3600):
        """
        Initialize command validator.
        
        Args:
            max_processed_ids: Maximum number of processed IDs to keep in memory
            ttl_seconds: Time-to-live for processed IDs (seconds)
        """
        self._max_processed_ids = max_processed_ids
        self._ttl_seconds = ttl_seconds
        
        # Thread-safe storage for processed command IDs
        # Dict: command_id -> timestamp (when it was processed)
        self._processed_command_ids: Dict[str, float] = {}
        self._lock = threading.Lock()
        
        # UUID v4 pattern (8-4-4-4-12 hex digits)
        self._uuid_pattern = re.compile(
            r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$',
            re.IGNORECASE
        )
        
        # ISO-8601 timestamp pattern
        self._iso8601_pattern = re.compile(
            r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?(Z|[+-]\d{2}:\d{2})$'
        )
    
    def validate_command(
        self,
        command: dict,
        position_registry=None,
        expected_robot_id: Optional[str] = None,
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate navigation command with comprehensive checks.
        
        Args:
            command: Command dictionary to validate
            position_registry: Optional PositionRegistry to validate target_id existence
        
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if command is valid, False otherwise
            - error_message: Error message if validation failed, None if valid
        """
        # Type check
        if not isinstance(command, dict):
            return (False, f'Command must be a dictionary, got {type(command).__name__}')
        
        # Size limit check (prevent DoS)
        command_size = sys.getsizeof(command)
        max_command_size = 10 * 1024  # 10 KB limit
        if command_size > max_command_size:
            return (False, f'Command too large: {command_size} bytes (max: {max_command_size})')
        
        # Check for required fields (SPECIFICATION.md Section 3.1 / 3.3 common fields)
        #
        # We allow a reserved `target_id="__pose__"` for direct pose mode, but `target_id` is still required.
        required_fields = ['schema_version', 'robot_id', 'command_id', 'timestamp', 'priority', 'target_id']
        for field in required_fields:
            if field not in command:
                return (False, f'Missing required field: {field}')

        # schema_version
        schema_version = command.get('schema_version')
        if not isinstance(schema_version, str):
            return (False, f'schema_version must be a string, got {type(schema_version).__name__}')
        if schema_version != '1.0':
            return (False, f'Unsupported schema_version: {schema_version} (expected: 1.0)')

        # robot_id
        robot_id = command.get('robot_id')
        if not isinstance(robot_id, str):
            return (False, f'robot_id must be a string, got {type(robot_id).__name__}')
        if len(robot_id) == 0:
            return (False, 'robot_id cannot be empty')
        if len(robot_id) > 128:
            return (False, f'robot_id too long: {len(robot_id)} characters (max: 128)')
        if expected_robot_id and robot_id != expected_robot_id:
            return (False, f'robot_id mismatch: got {robot_id}, expected {expected_robot_id}')
        
        # Validate command_id (UUID v4 format)
        command_id = command.get('command_id', '')
        if not isinstance(command_id, str):
            return (False, f'command_id must be a string, got {type(command_id).__name__}')
        if len(command_id) == 0:
            return (False, 'command_id cannot be empty')
        if len(command_id) > 128:  # Reasonable UUID length limit
            return (False, f'command_id too long: {len(command_id)} characters (max: 128)')
        
        if not self._uuid_pattern.match(command_id):
            return (False, f'command_id must be a valid UUID v4 format, got: {command_id[:50]}')
        
        # Check for duplicate command_id (thread-safe)
        if self.is_duplicate(command_id):
            return (False, f'Duplicate command_id: {command_id}. This command has already been processed.')
        
        # Validate timestamp (ISO-8601 format)
        timestamp = command.get('timestamp', '')
        if not isinstance(timestamp, str):
            return (False, f'timestamp must be a string, got {type(timestamp).__name__}')
        if len(timestamp) == 0:
            return (False, 'timestamp cannot be empty')
        if len(timestamp) > 64:  # Reasonable ISO-8601 length limit
            return (False, f'timestamp too long: {len(timestamp)} characters (max: 64)')
        
        if not self._iso8601_pattern.match(timestamp):
            return (False, f'timestamp must be in ISO-8601 format, got: {timestamp[:50]}')
        
        # Validate target_id (required)
        target_id = command.get('target_id', '')
        if target_id is None:
            return (False, 'target_id cannot be null')
        if not isinstance(target_id, str):
            return (False, f'target_id must be a string, got {type(target_id).__name__}')
        if len(target_id) == 0:
            return (False, 'target_id cannot be empty')
        if len(target_id) > 64:  # Reasonable position ID length
            return (False, f'target_id too long: {len(target_id)} characters (max: 64)')
        
        # Check for injection patterns (basic)
        if not re.match(r'^[a-zA-Z0-9_]+$', target_id):
            return (False, f'target_id contains invalid characters (only alphanumeric and underscore allowed): {target_id}')
        
        # Reserved direct pose selector
        has_xy = ('x' in command) and ('y' in command)
        if target_id == '__pose__':
            if not has_xy:
                return (False, 'target_id="__pose__" requires x and y coordinates')
        else:
            if not isinstance(target_id, str):
                return (False, f'target_id must be a string, got {type(target_id).__name__}')
            if len(target_id) == 0:
                return (False, 'target_id cannot be empty')
            if len(target_id) > 64:  # Reasonable position ID length
                return (False, f'target_id too long: {len(target_id)} characters (max: 64)')
            
            # Check for injection patterns (basic)
            if not re.match(r'^[a-zA-Z0-9_]+$', target_id):
                return (False, f'target_id contains invalid characters (only alphanumeric and underscore allowed): {target_id}')
            
            # Check if target_id exists in registry (if provided)
            if position_registry is not None:
                if not position_registry.hasPosition(target_id):
                    return (False, f'Unknown target_id: {target_id}')
        
        # Validate direct pose (if present)
        if has_xy:
            x = command.get('x')
            y = command.get('y')
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                return (False, 'x and y must be numbers')
            if math.isnan(float(x)) or math.isnan(float(y)):
                return (False, 'x and y must not be NaN')
            if abs(float(x)) > 1e6 or abs(float(y)) > 1e6:
                return (False, 'x/y out of bounds (abs > 1e6) - refusing command')
            
            # Optional orientation (radians)
            theta = command.get('theta', command.get('yaw', None))
            if theta is not None:
                if not isinstance(theta, (int, float)):
                    return (False, 'theta/yaw must be a number (radians)')
                if math.isnan(float(theta)):
                    return (False, 'theta/yaw must not be NaN')
                if abs(float(theta)) > 1e9:
                    return (False, 'theta/yaw out of bounds (abs > 1e9) - refusing command')
            
            # Optional frame_id
            frame_id = command.get('frame_id', None)
            if frame_id is not None:
                if not isinstance(frame_id, str):
                    return (False, f'frame_id must be a string, got {type(frame_id).__name__}')
                if len(frame_id) == 0 or len(frame_id) > 64:
                    return (False, 'frame_id must be 1..64 characters')
                # Basic ROS frame id validation
                if not re.match(r'^[A-Za-z][A-Za-z0-9_/]*$', frame_id):
                    return (False, f'frame_id contains invalid characters: {frame_id}')
        
        # Validate priority
        priority = command.get('priority', 'normal')
        if not isinstance(priority, str):
            return (False, f'priority must be a string, got {type(priority).__name__}')
        # AE.HUB MVP: low priority is FORBIDDEN
        valid_priorities = ['normal', 'high', 'emergency']
        if priority not in valid_priorities:
            return (False, f'Invalid priority: {priority}. Must be one of: {", ".join(valid_priorities)}')
        
        # Check for unexpected fields (warn but don't fail)
        allowed_fields = required_fields + [
            'x', 'y', 'theta', 'yaw', 'frame_id',
            'reason',  # reason is optional for cancel
        ]
        unexpected_fields = [f for f in command.keys() if f not in allowed_fields]
        if unexpected_fields:
            # Note: We don't have logger here, so we just ignore unexpected fields
            pass
        
        return (True, None)
    
    def is_duplicate(self, command_id: str) -> bool:
        """
        Check if command_id has already been processed.
        
        Args:
            command_id: Command ID to check
        
        Returns:
            True if duplicate, False otherwise
        """
        with self._lock:
            return command_id in self._processed_command_ids
    
    def mark_as_processed(self, command_id: str):
        """
        Mark command_id as processed (after successful validation and acceptance).
        
        Args:
            command_id: Command ID to mark as processed
        """
        with self._lock:
            current_time = time.time()
            self._processed_command_ids[command_id] = current_time
            
            # Cleanup if set grows too large
            if len(self._processed_command_ids) > self._max_processed_ids:
                self._cleanup_expired_internal(current_time)
    
    def cleanup_expired(self):
        """
        Clean up expired command IDs based on TTL.
        Should be called periodically to prevent memory leaks.
        """
        with self._lock:
            current_time = time.time()
            self._cleanup_expired_internal(current_time)
    
    def _cleanup_expired_internal(self, current_time: float):
        """
        Internal cleanup method (assumes lock is already held).
        
        Args:
            current_time: Current timestamp for TTL calculation
        """
        # Remove expired entries (older than TTL)
        expired_ids = [
            cmd_id for cmd_id, timestamp in self._processed_command_ids.items()
            if (current_time - timestamp) > self._ttl_seconds
        ]
        
        for cmd_id in expired_ids:
            del self._processed_command_ids[cmd_id]
        
        # If still too large, remove oldest entries
        if len(self._processed_command_ids) > self._max_processed_ids:
            sorted_items = sorted(
                self._processed_command_ids.items(),
                key=lambda x: x[1]  # Sort by timestamp
            )
            # Remove oldest half
            ids_to_remove = [cmd_id for cmd_id, _ in sorted_items[:self._max_processed_ids // 2]]
            for cmd_id in ids_to_remove:
                del self._processed_command_ids[cmd_id]
    
    def get_processed_count(self) -> int:
        """
        Get current number of processed command IDs.
        
        Returns:
            Number of processed command IDs
        """
        with self._lock:
            return len(self._processed_command_ids)
    
    def clear(self):
        """
        Clear all processed command IDs.
        Useful for testing or reset scenarios.
        """
        with self._lock:
            self._processed_command_ids.clear()

