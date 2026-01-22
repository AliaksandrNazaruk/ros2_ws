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
Unit tests for state machine.

Traceability: SRS state machine requirements
"""

import pytest
from aehub_navigation_executor.command_state_machine import CommandStateMachine, InternalState, PublicState


def test_state_transitions():
    """Test state transitions (IDLE → NAVIGATING → SUCCEEDED → IDLE)."""
    sm = CommandStateMachine()
    
    # Start in IDLE
    assert sm.get_state() == InternalState.IDLE
    assert sm.get_public_state() == PublicState.IDLE
    
    # Start navigation
    assert sm.start_navigation("cmd-1") == True
    assert sm.get_state() == InternalState.NAVIGATING
    assert sm.get_public_state() == PublicState.NAVIGATING
    assert sm.get_current_command_id() == "cmd-1"
    
    # Navigation succeeded
    assert sm.on_nav2_succeeded() == True
    assert sm.get_state() == InternalState.SUCCEEDED
    assert sm.get_public_state() == PublicState.SUCCEEDED
    assert sm.get_current_command_id() is None  # Cleared after terminal state
    
    # Reset to idle
    sm.reset_to_idle()
    assert sm.get_state() == InternalState.IDLE
    assert sm.get_public_state() == PublicState.IDLE
    assert sm.get_current_command_id() is None


def test_cancel_transitions():
    """Test cancel transitions (IDLE → CANCELING → CANCELED → IDLE)."""
    sm = CommandStateMachine()
    
    # Cancel from IDLE
    assert sm.start_cancel() == True
    assert sm.get_state() == InternalState.CANCELING
    assert sm.get_public_state() == PublicState.CANCELING
    
    # Complete cancel
    assert sm.on_cancel_completed() == True
    assert sm.get_state() == InternalState.CANCELED
    assert sm.get_public_state() == PublicState.CANCELED
    assert sm.get_current_command_id() is None
    
    # Reset
    sm.reset_to_idle()
    assert sm.get_state() == InternalState.IDLE
    
    # Cancel during navigation (NAVIGATING → CANCELING)
    sm.start_navigation("cmd-1")
    assert sm.get_state() == InternalState.NAVIGATING
    
    assert sm.start_cancel() == True
    assert sm.get_state() == InternalState.CANCELING
    assert sm.get_public_state() == PublicState.CANCELING


def test_aborted_transition():
    """Test aborted transition (NAVIGATING → ABORTED → IDLE)."""
    sm = CommandStateMachine()
    
    sm.start_navigation("cmd-1")
    assert sm.get_state() == InternalState.NAVIGATING
    
    # Nav2 aborts
    assert sm.on_nav2_aborted() == True
    assert sm.get_state() == InternalState.ABORTED
    assert sm.get_public_state() == PublicState.ABORTED
    assert sm.get_current_command_id() is None
    
    sm.reset_to_idle()
    assert sm.get_state() == InternalState.IDLE


def test_error_transition():
    """Test error transition."""
    sm = CommandStateMachine()
    
    # Error from any state
    sm.start_navigation("cmd-1")
    assert sm.on_error() == True
    assert sm.get_state() == InternalState.ERROR
    assert sm.get_current_command_id() is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
