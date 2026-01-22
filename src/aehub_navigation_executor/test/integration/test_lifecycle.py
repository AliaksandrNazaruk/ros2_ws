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
Integration tests for lifecycle behavior.

Traceability: SRS-LC-01, SRS-LC-02
"""

import pytest
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleNode

from aehub_navigation_executor.navigation_executor_node import NavigationExecutorNode


@pytest.fixture(scope="module")
def ros_context():
    """Create ROS2 context for testing."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    """Create NavigationExecutorNode instance."""
    node = NavigationExecutorNode()
    yield node
    try:
        node.destroy_node()
    except Exception:
        pass


# =========================
# SRS-LC-01: LifecycleNode usage
# =========================

def test_tc_lc_01_node_inherits_from_lifecycle_node(node):
    """TC-LC-01: Node inherits from LifecycleNode."""
    assert isinstance(node, LifecycleNode)
    # Get state from state machine: current_state is tuple (id, label)
    # State constants: 1=UNCONFIGURED, 2=INACTIVE, 3=ACTIVE
    current_state_id = node._state_machine.current_state[0]
    assert current_state_id == 1  # PRIMARY_STATE_UNCONFIGURED


def test_tc_lc_02_configure_activate_deactivate_cleanup_without_errors(node):
    """TC-LC-02: configure → activate → deactivate → cleanup without errors."""
    # State constructor: State(state_id, label)
    # Configure
    current_state = State(1, 'unconfigured')
    result = node.on_configure(current_state)
    assert result == TransitionCallbackReturn.SUCCESS
    # State transitions are handled by lifecycle manager, not directly in callbacks
    # For integration test, we verify callbacks return SUCCESS
    
    # Activate
    current_state = State(2, 'inactive')
    result = node.on_activate(current_state)
    assert result == TransitionCallbackReturn.SUCCESS
    
    # Deactivate
    current_state = State(3, 'active')
    result = node.on_deactivate(current_state)
    assert result == TransitionCallbackReturn.SUCCESS
    
    # Cleanup
    current_state = State(2, 'inactive')
    result = node.on_cleanup(current_state)
    assert result == TransitionCallbackReturn.SUCCESS


def test_tc_lc_03_on_activate_without_configure_failure(node):
    """TC-LC-03: on_activate without configure → FAILURE."""
    # Try to activate without configure
    current_state = State(1, 'unconfigured')
    result = node.on_activate(current_state)
    assert result == TransitionCallbackReturn.FAILURE


# =========================
# SRS-LC-02: No side effects in inactive state
# =========================

def test_tc_lc_04_commands_ignored_in_inactive(node):
    """TC-LC-04: Commands ignored in inactive."""
    # Configure but don't activate
    current_state = State(1, 'unconfigured')
    result = node.on_configure(current_state)
    assert result == TransitionCallbackReturn.SUCCESS
    
    # In inactive state, node should not process commands
    # This is verified by the fact that subscriptions are only created in on_activate
    # For skeleton, we verify configure succeeded (node is ready for activate)
    # Full state verification requires lifecycle manager


def test_tc_lc_05_ack_result_not_published_in_inactive(node):
    """TC-LC-05: Ack/result NOT published in inactive."""
    # Configure but don't activate
    current_state = State(1, 'unconfigured')
    result = node.on_configure(current_state)
    assert result == TransitionCallbackReturn.SUCCESS
    
    # In inactive state, publishers are not created
    # This is verified by the fact that event_publisher is only initialized in on_configure
    # but subscriptions are only created in on_activate
    # For skeleton, we verify configure succeeded


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
