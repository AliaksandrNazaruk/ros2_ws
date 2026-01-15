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
Command Rate Limiter

Limits the rate of commands to prevent abuse and ensure system stability.
Checks minimum interval between commands with the same command_id.

AE.HUB MVP: Prevents command flooding and ensures reasonable command spacing.

NOTE: This is a pure Python module, NOT a ROS2 Node.
Thread-safe for concurrent access.
"""

import threading
import time
from typing import Optional, Tuple, Dict


class CommandRateLimiter:
    """
    Limits command rate by enforcing minimum interval between commands.
    
    Thread-safe implementation using locks for concurrent access.
    Automatically cleans up old entries to prevent memory leaks.
    """
    
    def __init__(self, min_interval_seconds: float = 0.1):
        """
        Initialize rate limiter.
        
        Args:
            min_interval_seconds: Minimum interval between commands with same ID (seconds)
        """
        self._min_interval_seconds = min_interval_seconds
        
        # Thread-safe storage for last command time
        # Dict: command_id -> timestamp (when last command was received)
        self._last_command_time: Dict[str, float] = {}
        self._lock = threading.Lock()
    
    def check_rate_limit(self, command_id: str, current_time: Optional[float] = None) -> Tuple[bool, Optional[str]]:
        """
        Check if command violates rate limit.
        
        Args:
            command_id: Command ID to check
            current_time: Current timestamp (if None, uses time.time())
        
        Returns:
            Tuple of (is_allowed, error_message)
            - is_allowed: True if command is allowed, False if rate limited
            - error_message: Error message if rate limited, None if allowed
        """
        if current_time is None:
            current_time = time.time()
        
        with self._lock:
            if command_id in self._last_command_time:
                time_since_last = current_time - self._last_command_time[command_id]
                if time_since_last < self._min_interval_seconds:
                    error_msg = (
                        f'Command rate limit exceeded: {command_id} sent too frequently '
                        f'({time_since_last:.3f}s < {self._min_interval_seconds}s)'
                    )
                    return (False, error_msg)
            
            # Update last command time
            self._last_command_time[command_id] = current_time
        
        return (True, None)
    
    def update_last_command_time(self, command_id: str, current_time: Optional[float] = None):
        """
        Update last command time for a command_id.
        
        Args:
            command_id: Command ID to update
            current_time: Current timestamp (if None, uses time.time())
        """
        if current_time is None:
            current_time = time.time()
        
        with self._lock:
            self._last_command_time[command_id] = current_time
    
    def cleanup_old_entries(self, max_age_seconds: float = 3600, current_time: Optional[float] = None):
        """
        Clean up old entries to prevent memory leaks.
        
        Args:
            max_age_seconds: Maximum age of entries to keep (seconds)
            current_time: Current timestamp (if None, uses time.time())
        """
        if current_time is None:
            current_time = time.time()
        
        with self._lock:
            # Remove entries older than max_age_seconds
            expired_ids = [
                cmd_id for cmd_id, timestamp in self._last_command_time.items()
                if (current_time - timestamp) > max_age_seconds
            ]
            
            for cmd_id in expired_ids:
                del self._last_command_time[cmd_id]
            
            # If still too large, remove oldest entries
            max_entries = 1000
            if len(self._last_command_time) > max_entries:
                sorted_items = sorted(
                    self._last_command_time.items(),
                    key=lambda x: x[1]  # Sort by timestamp
                )
                # Remove oldest half
                ids_to_remove = [cmd_id for cmd_id, _ in sorted_items[:max_entries // 2]]
                for cmd_id in ids_to_remove:
                    del self._last_command_time[cmd_id]
    
    def get_last_command_time(self, command_id: str) -> Optional[float]:
        """
        Get last command time for a command_id.
        
        Args:
            command_id: Command ID to query
        
        Returns:
            Timestamp of last command, or None if not found
        """
        with self._lock:
            return self._last_command_time.get(command_id)
    
    def clear(self):
        """
        Clear all command time records.
        Useful for testing or reset scenarios.
        """
        with self._lock:
            self._last_command_time.clear()

