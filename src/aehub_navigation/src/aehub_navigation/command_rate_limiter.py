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

"""aehub_navigation.command_rate_limiter

Key fix versus the previous implementation:
- Rate limiting must be *global* (or at least per-command type), not keyed by
  `command_id`. A caller can otherwise trivially bypass rate limiting by
  generating fresh UUIDs.

This module therefore enforces a minimum interval between *received commands*
overall (and optionally per "bucket").
"""

import threading
import time
from typing import Optional, Tuple


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
        
        # Thread-safe storage for last command time.
        # IMPORTANT: This is intentionally NOT keyed by command_id.
        self._last_command_time: float = 0.0
        self._lock = threading.Lock()
    
    def check_rate_limit(self, bucket: str = "default", current_time: Optional[float] = None) -> Tuple[bool, Optional[str]]:
        """
        Check if command violates rate limit.
        
        Args:
            bucket: Optional bucket name (kept for future extensibility).
                    Current implementation is global and ignores the value.
            current_time: Current timestamp (if None, uses time.time())
        
        Returns:
            Tuple of (is_allowed, error_message)
            - is_allowed: True if command is allowed, False if rate limited
            - error_message: Error message if rate limited, None if allowed
        """
        if current_time is None:
            current_time = time.time()
        
        with self._lock:
            time_since_last = current_time - self._last_command_time
            if time_since_last < self._min_interval_seconds:
                error_msg = (
                    f'Command rate limit exceeded: commands sent too frequently '
                    f'({time_since_last:.3f}s < {self._min_interval_seconds}s)'
                )
                return (False, error_msg)

            self._last_command_time = current_time
        
        return (True, None)
    
    def update_last_command_time(self, current_time: Optional[float] = None):
        """
        Update last command time for a command_id.
        
        Args:
            command_id: Command ID to update
            current_time: Current timestamp (if None, uses time.time())
        """
        if current_time is None:
            current_time = time.time()
        
        with self._lock:
            self._last_command_time = current_time
    
    def cleanup_old_entries(self, max_age_seconds: float = 3600, current_time: Optional[float] = None):
        """No-op for backward compatibility.

        The earlier implementation used a per-command-id dictionary and needed
        pruning. The current implementation is constant-space.
        """
        _ = max_age_seconds
        _ = current_time
        return
    
    def get_last_command_time(self) -> Optional[float]:
        """
        Get last command time for a command_id.
        
        Returns:
            Timestamp of last command, or None if not found
        """
        with self._lock:
            return self._last_command_time or None
    
    def clear(self):
        """
        Clear all command time records.
        Useful for testing or reset scenarios.
        """
        with self._lock:
            self._last_command_time = 0.0

