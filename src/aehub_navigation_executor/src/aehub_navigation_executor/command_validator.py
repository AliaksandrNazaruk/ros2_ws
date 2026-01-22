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
Command validator and deduplication logic.

Responsibility:
- Validate command structure
- Detect duplicates by command_id
- Store command outcomes for replay
"""

import json
import time
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    from aehub_msgs.msg import NavigationCommand
except ImportError:
    NavigationCommand = None


class ValidationResult(Enum):
    """Command validation result."""
    VALID = "valid"
    INVALID_COMMAND = "invalid_command"
    DUPLICATE = "duplicate"
    RATE_LIMITED = "rate_limited"


@dataclass
class CommandOutcome:
    """Stored outcome of a processed command."""
    command_id: str
    outcome: str  # "ack", "result"
    ack_data: Optional[dict] = None
    result_data: Optional[dict] = None
    timestamp: float = 0.0


class CommandValidator:
    """
    Validates commands and handles deduplication.
    
    Stores command outcomes for replay.
    """
    
    def __init__(self, cache_ttl_sec: float = 600.0):
        """
        Initialize command validator.
        
        Args:
            cache_ttl_sec: Time-to-live for command cache entries (default: 10 min)
        """
        self._cache_ttl_sec = cache_ttl_sec
        self._command_cache: Dict[str, CommandOutcome] = {}
        self._last_navigate_time: float = 0.0
    
    def validate_navigate(self, command_json: str, rate_limit_sec: float = 1.0) -> Tuple[bool, str, dict, bool, Optional[CommandOutcome]]:
        """
        Validate navigateTo command.
        
        Args:
            command_json: JSON string command
            rate_limit_sec: Rate limit in seconds
            
        Returns:
            Tuple of (ok, reason, data, is_dup, outcome)
            ok: True if valid, False otherwise
            reason: Error reason string
            data: Parsed command dict
            is_dup: True if duplicate
            outcome: Cached outcome if duplicate
        """
        try:
            cmd = json.loads(command_json)
        except json.JSONDecodeError:
            return False, "invalid_json", {}, False, None
        
        # Check required fields
        if 'command_id' not in cmd or 'timestamp' not in cmd:
            return False, "missing_fields", cmd, False, None
        
        command_id = cmd['command_id']
        
        # Check for duplicate
        if command_id in self._command_cache:
            outcome = self._command_cache[command_id]
            if outcome.ack_data or outcome.result_data:
                return False, "", cmd, True, outcome
        
        # Check rate limiting (only for navigateTo)
        current_time = time.time()
        if current_time - self._last_navigate_time < rate_limit_sec:
            return False, "rate_limited", cmd, False, None
        
        # Validate command structure
        # Must have either target_id OR (x, y, theta)
        has_target_id = 'target_id' in cmd
        has_coords = all(k in cmd for k in ['x', 'y'])
        
        if not (has_target_id or has_coords):
            return False, "missing_pose", cmd, False, None
        
        # Valid command
        self._last_navigate_time = current_time
        return True, "", cmd, False, None
    
    def validate_cancel(self, command_json: str) -> Tuple[bool, str, dict, bool, Optional[CommandOutcome]]:
        """
        Validate cancel command.
        
        Cancel is never rate-limited.
        
        Args:
            command_json: JSON string command
            
        Returns:
            Tuple of (ok, reason, data, is_dup, outcome)
            ok: True if valid, False otherwise
            reason: Error reason string
            data: Parsed command dict
            is_dup: True if duplicate
            outcome: Cached outcome if duplicate
        """
        try:
            cmd = json.loads(command_json)
        except json.JSONDecodeError:
            return False, "invalid_json", {}, False, None
        
        # Check required fields
        if 'command_id' not in cmd or 'timestamp' not in cmd:
            return False, "missing_fields", cmd, False, None
        
        command_id = cmd['command_id']
        
        # Check for duplicate
        if command_id in self._command_cache:
            outcome = self._command_cache[command_id]
            if outcome.ack_data or outcome.result_data:
                return False, "", cmd, True, outcome
        
        # Valid command (cancel is never rate-limited)
        return True, "", cmd, False, None
    
    def store_outcome(self, command_id: str, ack_data: Optional[dict] = None, result_data: Optional[dict] = None):
        """
        Store command outcome for replay.
        
        Args:
            command_id: Command ID
            ack_data: Ack data (optional)
            result_data: Result data (optional)
        """
        # IMPORTANT: a duplicate replay may require both ack and result. We therefore
        # merge updates instead of overwriting.
        now = time.time()
        existing = self._command_cache.get(command_id)
        if existing:
            merged_ack = existing.ack_data or ack_data
            merged_result = existing.result_data or result_data
        else:
            merged_ack = ack_data
            merged_result = result_data

        outcome = CommandOutcome(
            command_id=command_id,
            outcome="result" if merged_result else "ack",
            ack_data=merged_ack,
            result_data=merged_result,
            timestamp=now,
        )
        self._command_cache[command_id] = outcome
    
    def cleanup_expired(self):
        """Remove expired entries from cache."""
        current_time = time.time()
        expired_ids = [
            cmd_id for cmd_id, outcome in self._command_cache.items()
            if current_time - outcome.timestamp > self._cache_ttl_sec
        ]
        for cmd_id in expired_ids:
            del self._command_cache[cmd_id]
    
    def get_outcome(self, command_id: str) -> Optional[CommandOutcome]:
        """Get stored outcome for command_id."""
        return self._command_cache.get(command_id)
    
    def validate_command(self, cmd: 'NavigationCommand', rate_limit_sec: float = 1.0) -> Tuple[bool, str]:
        """
        Validate NavigationCommand message (Step 1: transport-agnostic validation).
        
        Args:
            cmd: NavigationCommand message
            rate_limit_sec: Rate limit in seconds (only for navigateTo)
            
        Returns:
            Tuple of (ok, reason)
            ok: True if valid, False otherwise
            reason: Error reason string (empty if valid)
        """
        if NavigationCommand is None:
            return False, "NavigationCommand not available"
        
        if not cmd or not cmd.command_id:
            return False, "missing_command_id"
        
        # Validate command_id format (UUID v4)
        import re
        uuid_pattern = r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$'
        if not re.match(uuid_pattern, cmd.command_id.lower()):
            return False, "invalid_command_id_format"
        
        # Validate command type
        if cmd.type not in ["navigateTo", "cancel"]:
            return False, f"invalid_command_type: {cmd.type}"
        
        # Type-specific validation
        if cmd.type == "navigateTo":
            # Must have either target_id OR (x, y)
            has_target_id = cmd.target_id and len(cmd.target_id) > 0
            has_coords = (cmd.x is not None and cmd.y is not None) or (cmd.x != 0.0 or cmd.y != 0.0)
            
            if not (has_target_id or has_coords):
                return False, "missing_pose"
            
            # Rate limiting (only for navigateTo)
            current_time = time.time()
            if current_time - self._last_navigate_time < rate_limit_sec:
                return False, "rate_limited"
            
            self._last_navigate_time = current_time
        
        # Cancel commands don't need additional validation
        return True, ""
    
    def is_duplicate(self, command_id: str) -> bool:
        """
        Check if command_id is a duplicate (has cached outcome).
        
        Args:
            command_id: Command ID to check
            
        Returns:
            True if duplicate (has cached outcome), False otherwise
        """
        return command_id in self._command_cache and (
            self._command_cache[command_id].ack_data is not None or
            self._command_cache[command_id].result_data is not None
        )
    
    def clear(self):
        """Clear all cached outcomes."""
        self._command_cache.clear()
        self._last_navigate_time = 0.0
