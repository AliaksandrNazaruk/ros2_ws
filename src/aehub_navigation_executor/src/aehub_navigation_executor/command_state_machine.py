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
Command state machine.

Manages internal states and transitions for navigation commands.
"""

from enum import Enum
from typing import Optional


class InternalState(Enum):
    """Internal state machine states."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    CANCELING = "canceling"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    ERROR = "error"


class PublicState(Enum):
    """Public state (published in events)."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    CANCELING = "canceling"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    ERROR = "error"


class CommandStateMachine:
    """
    State machine for navigation commands.
    
    Transitions:
    IDLE → NAVIGATING → SUCCEEDED | ABORTED | CANCELED | ERROR → IDLE
    IDLE → CANCELING → CANCELED → IDLE
    NAVIGATING → CANCELING → CANCELED → IDLE
    """
    
    def __init__(self):
        """Initialize state machine."""
        self._state = InternalState.IDLE
        self._current_command_id: Optional[str] = None
        self._current_target_id: Optional[str] = None
    
    def get_state(self) -> InternalState:
        """Get current internal state."""
        return self._state
    
    def get_public_state(self) -> PublicState:
        """Get public state (for events)."""
        return PublicState(self._state.value)
    
    def start_navigation(self, command_id: str, target_id: Optional[str] = None) -> bool:
        """
        Start navigation.
        
        Args:
            command_id: Command ID
            target_id: Optional target ID
            
        Returns:
            True if transition successful, False otherwise
        """
        if self._state != InternalState.IDLE:
            return False
        
        self._state = InternalState.NAVIGATING
        self._current_command_id = command_id
        self._current_target_id = target_id
        return True
    
    def start_cancel(self) -> bool:
        """
        Start cancel operation.
        
        Can be called from IDLE or NAVIGATING.
        
        Returns:
            True if transition successful, False otherwise
        """
        if self._state == InternalState.IDLE:
            self._state = InternalState.CANCELING
            return True
        elif self._state == InternalState.NAVIGATING:
            self._state = InternalState.CANCELING
            return True
        
        return False
    
    def on_nav2_succeeded(self) -> bool:
        """
        Handle Nav2 success.
        
        Only valid from NAVIGATING state.
        
        Returns:
            True if transition successful
        """
        if self._state == InternalState.NAVIGATING:
            self._state = InternalState.SUCCEEDED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def on_nav2_aborted(self) -> bool:
        """
        Handle Nav2 abort.
        
        Only valid from NAVIGATING state.
        
        Returns:
            True if transition successful
        """
        if self._state == InternalState.NAVIGATING:
            self._state = InternalState.ABORTED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def on_cancel_completed(self) -> bool:
        """
        Handle cancel completion.
        
        Only valid from CANCELING state.
        
        Returns:
            True if transition successful
        """
        if self._state == InternalState.CANCELING:
            self._state = InternalState.CANCELED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def on_error(self) -> bool:
        """
        Handle error.
        
        Can be called from any state.
        
        Returns:
            True if transition successful
        """
        self._state = InternalState.ERROR
        self._current_command_id = None
        self._current_target_id = None
        return True
    
    def complete_succeeded(self) -> bool:
        """Complete with success."""
        if self._state == InternalState.NAVIGATING:
            self._state = InternalState.SUCCEEDED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def complete_canceled(self) -> bool:
        """Complete with canceled."""
        if self._state in (InternalState.NAVIGATING, InternalState.CANCELING):
            self._state = InternalState.CANCELED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def complete_aborted(self) -> bool:
        """Complete with aborted."""
        if self._state == InternalState.NAVIGATING:
            self._state = InternalState.ABORTED
            self._current_command_id = None
            self._current_target_id = None
            return True
        return False
    
    def complete_error(self, error_message: Optional[str] = None) -> bool:
        """Complete with error."""
        self._state = InternalState.ERROR
        self._current_command_id = None
        self._current_target_id = None
        return True
    
    def reset_to_idle(self):
        """Reset to IDLE state (called after publishing terminal result)."""
        self._state = InternalState.IDLE
        self._current_command_id = None
        self._current_target_id = None
    
    def get_current_command_id(self) -> Optional[str]:
        """Get current command ID (if any)."""
        return self._current_command_id
    
    def get_current_target_id(self) -> Optional[str]:
        """Get current target ID (if any)."""
        return self._current_target_id
    
    def snapshot(self):
        """Get current state snapshot for publishing."""
        from dataclasses import dataclass
        
        @dataclass
        class StateSnapshot:
            public_state: str
            internal_state: str
            command_id: Optional[str]
            target_id: Optional[str]
        
        return StateSnapshot(
            public_state=self.get_public_state().value,
            internal_state=self.get_state().value,
            command_id=self._current_command_id,
            target_id=self._current_target_id,
        )
