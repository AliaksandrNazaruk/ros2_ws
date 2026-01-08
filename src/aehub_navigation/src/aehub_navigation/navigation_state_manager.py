#!/usr/bin/env python3

"""
Navigation State Manager

Manages navigation state based on Nav2 Action lifecycle.
States: idle, navigating, arrived, error

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
    ARRIVED = "arrived"
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
    
    def setState(self, state: NavigationState, target_id: str = None, 
                 error_code: str = None, error_message: str = None):
        """Set navigation state"""
        with self._state_lock:
            if self.current_state == state:
                return
            
            old_state = self.current_state
            self.current_state = state
            self.current_target_id = target_id
            self.error_code = error_code
            self.error_message = error_message
        
        self._log('info',
            f'State changed: {old_state.value} -> {state.value} '
            f'[target_id: {target_id}]'
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
        self.setState(NavigationState.ARRIVED, target_id=target_id)
        # Transition to IDLE after a short delay
        # (can be handled by external logic)
    
    def onGoalAborted(self, target_id: str, error_code: str, error_message: str):
        """Called when goal aborted"""
        self.setState(
            NavigationState.ERROR,
            target_id=target_id,
            error_code=error_code,
            error_message=error_message
        )
    
    def onGoalCanceled(self, target_id: str):
        """Called when goal canceled"""
        self.setState(NavigationState.IDLE, target_id=target_id)
    
    def resetToIdle(self):
        """Reset to idle state"""
        self.setState(NavigationState.IDLE)

