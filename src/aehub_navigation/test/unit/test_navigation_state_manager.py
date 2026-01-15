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
Unit tests for NavigationStateManager

Tests state machine transitions and state management logic.
"""

import pytest
from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState


class TestNavigationStateManager:
    """Test suite for NavigationStateManager"""
    
    @pytest.fixture
    def state_manager(self):
        """Create a NavigationStateManager instance"""
        # NavigationStateManager is now a simple Python class, not a ROS2 Node
        manager = NavigationStateManager(logger=None)
        yield manager
    
    def test_initialization_idle(self, state_manager):
        """Test that state manager initializes in IDLE state"""
        assert state_manager.getState() == NavigationState.IDLE
        assert state_manager.getTargetId() is None
        error_code, error_message = state_manager.getError()
        assert error_code is None
        assert error_message is None
    
    def test_on_goal_sent(self, state_manager):
        """Test transition IDLE → NAVIGATING"""
        callback_called = []
        callback_target_id = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append((state, target_id, error_code, error_message))
            callback_target_id.append(target_id)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        
        assert state_manager.getState() == NavigationState.NAVIGATING
        assert state_manager.getTargetId() == target_id
        assert len(callback_called) == 1
        assert callback_called[0][0] == NavigationState.NAVIGATING
        assert callback_called[0][1] == target_id
    
    def test_on_goal_succeeded(self, state_manager):
        """Test transition NAVIGATING → SUCCEEDED"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        
        callback_called = []
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append((state, target_id))
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        state_manager.onGoalSucceeded(target_id)
        
        assert state_manager.getState() == NavigationState.SUCCEEDED
        assert state_manager.getTargetId() == target_id
        assert len(callback_called) == 1
        assert callback_called[0][0] == NavigationState.SUCCEEDED
    
    def test_on_goal_aborted(self, state_manager):
        """Test transition NAVIGATING → ABORTED"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        
        callback_called = []
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append((state, target_id, error_code, error_message))
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        error_code = "NAV_GOAL_ABORTED"
        error_message = "Test error"
        state_manager.onGoalAborted(target_id, error_code, error_message)
        
        assert state_manager.getState() == NavigationState.ABORTED
        assert state_manager.getTargetId() == target_id
        returned_error_code, returned_error_message = state_manager.getError()
        assert returned_error_code == error_code
        assert returned_error_message == error_message
        assert len(callback_called) == 1
        assert callback_called[0][0] == NavigationState.ABORTED
        assert callback_called[0][2] == error_code
        assert callback_called[0][3] == error_message
    
    def test_on_goal_canceled(self, state_manager):
        """Test transition NAVIGATING → IDLE (via cancel)"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        
        callback_called = []
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append((state, target_id))
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        state_manager.onGoalCanceled(target_id)
        
        assert state_manager.getState() == NavigationState.IDLE
        assert state_manager.getTargetId() == target_id  # target_id may still be set
        assert len(callback_called) == 1
        assert callback_called[0][0] == NavigationState.IDLE
    
    def test_reset_to_idle(self, state_manager):
        """Test reset to IDLE from any state"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        state_manager.onGoalSucceeded(target_id)
        
        # Should be in SUCCEEDED state
        assert state_manager.getState() == NavigationState.SUCCEEDED
        
        callback_called = []
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append((state, target_id))
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        state_manager.resetToIdle()
        
        assert state_manager.getState() == NavigationState.IDLE
        assert len(callback_called) == 1
        assert callback_called[0][0] == NavigationState.IDLE
    
    def test_reset_to_idle_from_error(self, state_manager):
        """Test reset to IDLE from ERROR state"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        # ERROR is a distinct state used for validation/system errors (not Nav2 ABORTED)
        state_manager.setState(NavigationState.ERROR, target_id=target_id, error_code="NAV_INVALID_COMMAND", error_message="Test error")
        
        # Should be in ERROR state
        assert state_manager.getState() == NavigationState.ERROR
        
        state_manager.resetToIdle()
        
        assert state_manager.getState() == NavigationState.IDLE
    
    def test_same_state_no_callback(self, state_manager):
        """Test that setState to the same state doesn't trigger callback"""
        target_id = "position_A"
        state_manager.onGoalSent(target_id)
        
        callback_called = []
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        # Set to same state (NAVIGATING)
        state_manager.setState(NavigationState.NAVIGATING, target_id=target_id)
        
        # Callback should not be called
        assert len(callback_called) == 0
    
    def test_state_transition_sequence(self, state_manager):
        """Test complete state transition sequence"""
        callback_states = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_states.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        target_id = "position_A"
        
        # IDLE → NAVIGATING
        state_manager.onGoalSent(target_id)
        assert state_manager.getState() == NavigationState.NAVIGATING
        
        # NAVIGATING → SUCCEEDED
        state_manager.onGoalSucceeded(target_id)
        assert state_manager.getState() == NavigationState.SUCCEEDED
        
        # SUCCEEDED → IDLE
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
        
        # Verify callbacks were called
        assert len(callback_states) == 3
        assert callback_states[0] == NavigationState.NAVIGATING
        assert callback_states[1] == NavigationState.SUCCEEDED
        assert callback_states[2] == NavigationState.IDLE
    
    def test_error_state_preserves_target_id(self, state_manager):
        """Test that ERROR state preserves target_id and error info"""
        target_id = "position_B"
        error_code = "NAV_GOAL_REJECTED"
        error_message = "Nav2 rejected the goal"
        
        state_manager.onGoalSent(target_id)
        state_manager.setState(NavigationState.ERROR, target_id=target_id, error_code=error_code, error_message=error_message)
        
        assert state_manager.getState() == NavigationState.ERROR
        assert state_manager.getTargetId() == target_id
        returned_error_code, returned_error_message = state_manager.getError()
        assert returned_error_code == error_code
        assert returned_error_message == error_message


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

