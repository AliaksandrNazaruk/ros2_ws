#!/usr/bin/env python3

"""
Integration tests for state machine transitions

Tests state machine behavior in integration context.
"""

import pytest
import time
from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState


class TestStateMachineIntegration:
    """Test suite for state machine integration"""
    
    @pytest.fixture
    def state_manager(self):
        """Create a NavigationStateManager instance"""
        # NavigationStateManager is now a simple Python class, not a ROS2 Node
        manager = NavigationStateManager(logger=None)
        yield manager
    
    def test_complete_cycle_idle_to_arrived_to_idle(self, state_manager):
        """Test complete cycle: IDLE → NAVIGATING → ARRIVED → IDLE"""
        callback_states = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_states.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        target_id = "position_A"
        
        # IDLE → NAVIGATING
        state_manager.onGoalSent(target_id)
        assert state_manager.getState() == NavigationState.NAVIGATING
        
        # NAVIGATING → ARRIVED
        state_manager.onGoalSucceeded(target_id)
        assert state_manager.getState() == NavigationState.ARRIVED
        
        # ARRIVED → IDLE
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
        
        # Verify all callbacks were called
        assert len(callback_states) == 3
        assert callback_states[0] == NavigationState.NAVIGATING
        assert callback_states[1] == NavigationState.ARRIVED
        assert callback_states[2] == NavigationState.IDLE
    
    def test_error_cycle_navigating_to_error_to_idle(self, state_manager):
        """Test error cycle: NAVIGATING → ERROR → IDLE"""
        callback_states = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_states.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        target_id = "position_B"
        
        # IDLE → NAVIGATING
        state_manager.onGoalSent(target_id)
        assert state_manager.getState() == NavigationState.NAVIGATING
        
        # NAVIGATING → ERROR
        error_code = "NAV_GOAL_ABORTED"
        error_message = "Test error"
        state_manager.onGoalAborted(target_id, error_code, error_message)
        assert state_manager.getState() == NavigationState.ERROR
        returned_error_code, returned_error_message = state_manager.getError()
        assert returned_error_code == error_code
        assert returned_error_message == error_message
        
        # ERROR → IDLE
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
        
        # Verify callbacks
        assert len(callback_states) == 3
        assert callback_states[0] == NavigationState.NAVIGATING
        assert callback_states[1] == NavigationState.ERROR
        assert callback_states[2] == NavigationState.IDLE
    
    def test_cancel_cycle_navigating_to_idle(self, state_manager):
        """Test cancel cycle: NAVIGATING → IDLE"""
        callback_states = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_states.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        target_id = "position_C"
        
        # IDLE → NAVIGATING
        state_manager.onGoalSent(target_id)
        assert state_manager.getState() == NavigationState.NAVIGATING
        
        # NAVIGATING → IDLE (via cancel)
        state_manager.onGoalCanceled(target_id)
        assert state_manager.getState() == NavigationState.IDLE
        
        # Verify callbacks
        assert len(callback_states) == 2
        assert callback_states[0] == NavigationState.NAVIGATING
        assert callback_states[1] == NavigationState.IDLE
    
    def test_cancel_from_arrived_state(self, state_manager):
        """Test cancel from ARRIVED state → IDLE"""
        target_id = "position_D"
        
        # Navigate to ARRIVED
        state_manager.onGoalSent(target_id)
        state_manager.onGoalSucceeded(target_id)
        assert state_manager.getState() == NavigationState.ARRIVED
        
        # Cancel should transition to IDLE
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
    
    def test_cancel_from_error_state(self, state_manager):
        """Test cancel from ERROR state → IDLE"""
        target_id = "position_E"
        
        # Navigate to ERROR
        state_manager.onGoalSent(target_id)
        state_manager.onGoalAborted(target_id, "NAV_GOAL_ABORTED", "Test error")
        assert state_manager.getState() == NavigationState.ERROR
        
        # Cancel should transition to IDLE
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
    
    def test_cancel_idempotent_in_idle(self, state_manager):
        """Test that cancel is idempotent when already in IDLE"""
        callback_called = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            callback_called.append(state)
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        # Already in IDLE
        assert state_manager.getState() == NavigationState.IDLE
        
        # Cancel should not change state or call callback
        state_manager.resetToIdle()
        assert state_manager.getState() == NavigationState.IDLE
        # Callback should not be called (same state)
        assert len(callback_called) == 0
    
    def test_multiple_state_transitions(self, state_manager):
        """Test multiple sequential state transitions"""
        transitions = []
        
        def state_change_callback(state, target_id, error_code, error_message):
            transitions.append((state, target_id))
        
        state_manager.setStateChangeCallback(state_change_callback)
        
        # First navigation cycle
        state_manager.onGoalSent("position_A")
        state_manager.onGoalSucceeded("position_A")
        state_manager.resetToIdle()
        
        # Second navigation cycle
        state_manager.onGoalSent("position_B")
        state_manager.onGoalCanceled("position_B")
        
        # Verify transitions
        assert len(transitions) == 5
        assert transitions[0][0] == NavigationState.NAVIGATING
        assert transitions[0][1] == "position_A"
        assert transitions[1][0] == NavigationState.ARRIVED
        assert transitions[2][0] == NavigationState.IDLE
        assert transitions[3][0] == NavigationState.NAVIGATING
        assert transitions[3][1] == "position_B"
        assert transitions[4][0] == NavigationState.IDLE


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

