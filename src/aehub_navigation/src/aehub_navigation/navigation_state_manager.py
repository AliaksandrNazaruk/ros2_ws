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
Navigation State Manager

Manages navigation state based on Nav2 Action lifecycle.
States (SPECIFICATION.md): idle, navigating, paused, error, canceling, succeeded, aborted

AE.HUB MVP requirement: Source of truth is Nav2 Action lifecycle,
NOT diagnostics, NOT internal Nav2 state machine.

NOTE: This is a Python module, NOT a ROS2 Node.
"""

from enum import Enum
from typing import Optional, Callable
import threading


class NavigationState(Enum):
    """Navigation states"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    PAUSED = "paused"
    CANCELING = "canceling"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    ERROR = "error"


class NavigationStateManager:
    """
    Navigation State Manager (Python module, not ROS2 Node).
    
    Manages navigation state based on Nav2 Action lifecycle.
    """
    def __init__(self, logger=None):
        """
        Initialize state manager.
        
        Args:
            logger: Optional logger object with info(), warn(), error(), debug() methods.
                   If None, uses print() for logging.
        """
        self.logger = logger
        
        # Thread-safe state access
        self._state_lock = threading.Lock()
        self.current_state = NavigationState.IDLE
        self.current_target_id = None
        self.error_code = None
        self.error_message = None
        
        # State change callback
        self.state_change_callback: Optional[Callable] = None
        
        self._log('info', 'NavigationStateManager initialized')
    
    def _log(self, level: str, message: str):
        """Log message using provided logger or print"""
        if self.logger:
            if level == 'info':
                self.logger.info(message)
            elif level == 'warn':
                self.logger.warn(message)
            elif level == 'error':
                self.logger.error(message)
            elif level == 'debug':
                self.logger.debug(message)
        else:
            print(f'[{level.upper()}] {message}')
    
    def setState(
        self,
        state: NavigationState,
        target_id: str = None,
        error_code: str = None,
        error_message: str = None,
    ):
        """Set navigation state"""
        with self._state_lock:
            if self.current_state == state:
                return
            
            old_state = self.current_state
            self.current_state = state
            self.current_target_id = target_id
            self.error_code = error_code
            self.error_message = error_message
        
        self._log(
            'info',
            f'State changed: {old_state.value} -> {state.value} [target_id: {target_id}]'
        )
        
        if self.state_change_callback:
            self.state_change_callback(state, target_id, error_code, error_message)
    
    def getState(self) -> NavigationState:
        """Get current state"""
        with self._state_lock:
            return self.current_state
    
    def getTargetId(self) -> str:
        """Get current target ID"""
        with self._state_lock:
            return self.current_target_id
    
    def getError(self) -> tuple:
        """Get error info (error_code, error_message)"""
        with self._state_lock:
            return (self.error_code, self.error_message)
    
    def setStateChangeCallback(self, callback):
        """Set callback for state changes"""
        self.state_change_callback = callback
    
    def onGoalSent(self, target_id: str):
        """Called when goal is sent"""
        self.setState(NavigationState.NAVIGATING, target_id=target_id)
    
    def onGoalSucceeded(self, target_id: str):
        """Called when goal succeeded"""
        self.setState(NavigationState.SUCCEEDED, target_id=target_id)
        # Transition to IDLE after a short delay
        # (can be handled by external logic)
    
    def onGoalAborted(self, target_id: str, error_code: str, error_message: str):
        """Called when goal aborted"""
        self.setState(
            NavigationState.ABORTED,
            target_id=target_id,
            error_code=error_code,
            error_message=error_message
        )
    
    def onGoalCanceled(self, target_id: str):
        """Called when goal canceled"""
        # Cancellation is reflected as a terminal RESULT event; status returns to IDLE.
        self.setState(NavigationState.IDLE, target_id=target_id)

    def onCancelRequested(self, target_id: Optional[str] = None):
        """Called when a cancel command is accepted and cancel is in progress."""
        self.setState(NavigationState.CANCELING, target_id=target_id)
    
    def resetToIdle(self):
        """Reset to idle state"""
        self.setState(NavigationState.IDLE)

