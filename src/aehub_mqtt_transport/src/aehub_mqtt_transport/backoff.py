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
Exponential backoff policy for retry logic.

Isolates retry delay calculation from connection logic.
"""


class ExponentialBackoff:
    """
    Exponential backoff for retry operations.
    
    Example:
        backoff = ExponentialBackoff(initial=1.0, maximum=30.0)
        while not success:
            delay = backoff.next_delay()
            time.sleep(delay)
            # ... retry operation ...
    """
    
    def __init__(self, initial: float, maximum: float):
        """
        Initialize exponential backoff.
        
        Args:
            initial: Initial delay in seconds
            maximum: Maximum delay in seconds
        """
        if initial <= 0:
            raise ValueError("initial delay must be > 0")
        if maximum < initial:
            raise ValueError("maximum delay must be >= initial")
        
        self._initial = initial
        self._maximum = maximum
        self._current = initial
    
    def reset(self):
        """Reset delay to initial value."""
        self._current = self._initial
    
    def next_delay(self) -> float:
        """
        Get next delay and increment for subsequent calls.
        
        Returns:
            Current delay value (before increment)
        """
        delay = self._current
        self._current = min(self._current * 2, self._maximum)
        return delay
    
    def current_delay(self) -> float:
        """
        Get current delay without incrementing.
        
        Returns:
            Current delay value
        """
        return self._current
