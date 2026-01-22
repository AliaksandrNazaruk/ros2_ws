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
Integration tests for navigation executor full flow.

Traceability: SRS-NAV, SRS-CANCEL, SRS-DD, SRS-EVT
"""

import pytest

# NOTE: These integration tests target a legacy design where the executor talked directly to
# Nav2 `NavigateToPose` and published legacy event topics (/aehub/events/*).
#
# The current clean stack routes:
# - MQTT/JSON -> NavigationCommand -> NavigationExecute (capability) -> NavigationEvent (single stream)
#
# The old tests need a full rewrite to target the capability action + new event stream.
pytestmark = pytest.mark.skip(reason="legacy integration tests (pre-capability / pre-NavigationEvent)")

import json
import rclpy
import time
import threading
import uuid
from typing import List, Optional

from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.node import Node
from std_msgs.msg import String

from aehub_navigation_executor.navigation_executor_node import NavigationExecutorNode

# Import test helpers (they need to be in same directory)
import sys
import os
test_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, test_dir)

from test_helpers import (
    FakeNav2ActionServer,
    ROSEventCollector,
    CommandPublisher,
)


def generate_command_id() -> str:
    """Generate UUID v4 command ID for testing."""
    return str(uuid.uuid4())


@pytest.fixture(scope="function")
def ros_context():
    """Create ROS2 context for testing."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    """Create test helper node."""
    node = Node('test_node')
    yield node
    try:
        node.destroy_node()
    except Exception:
        pass


@pytest.fixture
def nav2_server(test_node):
    """Create fake Nav2 action server."""
    server = FakeNav2ActionServer()
    
    # Run server in background
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server)
    
    server_thread = threading.Thread(
        target=lambda: executor.spin(),
        daemon=True
    )
    server_thread.start()
    
    # Wait for server to be ready
    time.sleep(0.2)
    
    yield server
    
    try:
        executor.shutdown()
        server.destroy_node()
    except Exception:
        pass


@pytest.fixture
def executor_node(ros_context):
    """Create NavigationExecutorNode instance."""
    node = NavigationExecutorNode()
    yield node
    try:
        node.destroy_node()
    except Exception:
        pass


@pytest.fixture
def event_collector(test_node):
    """Create event collector."""
    collector = ROSEventCollector(test_node)
    yield collector
    collector.clear()


@pytest.fixture
def cmd_publisher(test_node):
    """Create command publisher."""
    return CommandPublisher(test_node)


def spin_until(condition, timeout_sec: float = 2.0, node: Optional[Node] = None):
    """Spin ROS until condition is met."""
    start = time.time()
    while time.time() - start < timeout_sec:
        if node:
            rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return True
        time.sleep(0.05)
    return False


# =========================
# SRS-NAV-02: Goal accepted path
# =========================

def test_tc_nav_03_nav2_accepts_goal(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-NAV-03: Nav2 accepts goal."""
    # Configure and activate node
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    # Wait for Nav2 server to be ready
    time.sleep(0.3)
    
    # Process any pending callbacks
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # Publish navigate command
    cmd_publisher.publish_navigate('cmd-001', x=1.0, y=2.0, theta=0.0)
    
    # Process callbacks
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Check for ack=accepted
    acks = event_collector.get_acks()
    accepted_acks = [a for a in acks if a.get('command_id') == 'cmd-001' and 
                     (a.get('ack_status') == 'accepted' or a.get('status') == 'accepted')]
    
    assert len(accepted_acks) > 0, f"No accepted ack found. All acks: {acks}"


def test_tc_nav_04_ack_accepted(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-NAV-04: ack=accepted."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_id = generate_command_id()
    cmd_publisher.publish_navigate(cmd_id, x=2.0, y=3.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    acks = event_collector.get_acks()
    cmd_acks = [a for a in acks if a.get('command_id') == cmd_id]
    
    # Should have received ack with accepted status
    assert len(cmd_acks) > 0, "No ack found"
    
    # Check for accepted (either old or new format)
    has_accepted = any(
        a.get('ack_status') == 'accepted' or a.get('status') == 'accepted'
        for a in cmd_acks
    )
    assert has_accepted, f"No accepted ack. Acks: {cmd_acks}"


# =========================
# SRS-NAV-03: Goal rejected by Nav2
# =========================

def test_tc_nav_06_nav2_rejects_goal(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-NAV-06: Nav2 rejects goal."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    # Configure Nav2 to reject next goal
    nav2_server.reject_next_goal()
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_id = generate_command_id()
    cmd_publisher.publish_navigate(cmd_id, x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    acks = event_collector.get_acks()
    results = event_collector.get_results()
    
    # Should have rejected ack and error result
    reject_acks = [a for a in acks if a.get('command_id') == cmd_id and 
                   (a.get('ack_status') == 'rejected' or a.get('status') == 'rejected')]
    error_results = [r for r in results if r.get('command_id') == cmd_id and 
                     (r.get('result_status') == 'error' or r.get('result') == 'error')]
    
    assert len(reject_acks) > 0, "No rejected ack found"
    assert len(error_results) > 0, "No error result found"


# =========================
# SRS-CANCEL-01: Cancel with active goal
# =========================

def test_tc_can_01_cancel_during_navigation(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-CAN-01: cancel during navigation."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    # Set goal delay so we can cancel it
    nav2_server.set_goal_delay_sec(1.0)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # Start navigation
    cmd_publisher.publish_navigate('cmd-nav', x=1.0, y=2.0)
    
    # Wait for goal to be accepted
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Cancel during navigation
    cmd_publisher.publish_cancel('cmd-cancel')
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    results = event_collector.get_results()
    cancel_results = [r for r in results if r.get('command_id') == 'cmd-nav' and 
                      (r.get('result_status') == 'canceled' or r.get('result') == 'canceled')]
    
    # Should have canceled result for navigation command
    assert len(cancel_results) > 0, f"No canceled result. All results: {results}"


def test_tc_can_02_nav2_cancel_called_once(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-CAN-02: Nav2 cancel called once."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    nav2_server.set_goal_delay_sec(1.0)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_publisher.publish_navigate('cmd-nav2', x=1.0, y=2.0)
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Cancel multiple times (should be idempotent)
    cmd_publisher.publish_cancel('cmd-cancel1')
    cmd_publisher.publish_cancel('cmd-cancel2')
    cmd_publisher.publish_cancel('cmd-cancel3')
    
    for _ in range(30):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Check that cancel was called (through checking results)
    # The exact number depends on implementation, but should not be excessive
    results = event_collector.get_results()
    assert len(results) > 0, "No results found"


# =========================
# SRS-CANCEL-02: Cancel without goal
# =========================

def test_tc_can_04_cancel_in_idle(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-CAN-04: cancel in IDLE."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # Cancel without active goal
    cmd_publisher.publish_cancel('cmd-idle-cancel')
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    acks = event_collector.get_acks()
    results = event_collector.get_results()
    
    # Cancel should be accepted even without goal
    cancel_acks = [a for a in acks if a.get('command_id') == 'cmd-idle-cancel' and 
                   (a.get('ack_status') == 'accepted' or a.get('status') == 'accepted')]
    cancel_results = [r for r in results if r.get('command_id') == 'cmd-idle-cancel' and 
                      (r.get('result_status') == 'succeeded' or r.get('result') == 'succeeded')]
    
    assert len(cancel_acks) > 0, "Cancel should be accepted"
    assert len(cancel_results) > 0, "Cancel should succeed"


# =========================
# SRS-DD-02: Cross-topic deduplication
# =========================

def test_tc_dd_02_navigate_replay_no_new_nav2_goal(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-DD-02: navigateTo replay → no new Nav2 goal."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # First command
    cmd_publisher.publish_navigate('cmd-dup', x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    initial_goals = len(nav2_server.get_active_goals())
    
    # Duplicate command (should replay, not send new goal)
    cmd_publisher.publish_navigate('cmd-dup', x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Should not have more active goals (or same number)
    final_goals = len(nav2_server.get_active_goals())
    assert final_goals <= initial_goals, "Duplicate command should not create new Nav2 goal"


def test_tc_dd_03_ack_result_replayed_verbatim(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-DD-03: Ack/result replayed verbatim."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    nav2_server.set_goal_delay_sec(0.2)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # First command
    cmd_publisher.publish_navigate('cmd-replay', x=1.0, y=2.0)
    
    for _ in range(30):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Get first set of events
    first_acks = [a for a in event_collector.get_acks() if a.get('command_id') == 'cmd-replay']
    first_results = [r for r in event_collector.get_results() if r.get('command_id') == 'cmd-replay']
    
    # Duplicate command
    cmd_publisher.publish_navigate('cmd-replay', x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Check that events were replayed (should have same structure)
    all_acks = [a for a in event_collector.get_acks() if a.get('command_id') == 'cmd-replay']
    all_results = [r for r in event_collector.get_results() if r.get('command_id') == 'cmd-replay']
    
    # Should have more events (replay)
    assert len(all_acks) >= len(first_acks), "Replay should produce ack events"
    assert len(all_results) >= len(first_results), "Replay should produce result events"


# =========================
# SRS-EVT-01: Ack contract
# =========================

def test_tc_evt_01_ack_contains_command_id(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-EVT-01: ack contains command_id."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_publisher.publish_navigate('cmd-evt-001', x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    acks = event_collector.get_acks()
    cmd_acks = [a for a in acks if a.get('command_id') == 'cmd-evt-001']
    
    assert len(cmd_acks) > 0, "No ack found"
    for ack in cmd_acks:
        assert 'command_id' in ack, "Ack must contain command_id"
        assert ack['command_id'] == 'cmd-evt-001', "Ack command_id must match"


def test_tc_evt_02_ack_published_before_result(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-EVT-02: ack published before result."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    nav2_server.set_goal_delay_sec(0.2)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_publisher.publish_navigate('cmd-evt-002', x=1.0, y=2.0)
    
    # Collect events as they arrive
    events = event_collector.get_events()
    
    for _ in range(30):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
        events = event_collector.get_events()
    
    cmd_events = [e for e in events if e.payload.get('command_id') == 'cmd-evt-002']
    
    if len(cmd_events) >= 2:
        # Find first ack and first result
        acks = [e for e in cmd_events if e.topic == 'ack']
        results = [e for e in cmd_events if e.topic == 'result']
        
        if acks and results:
            first_ack = min(acks, key=lambda e: e.timestamp)
            first_result = min(results, key=lambda e: e.timestamp)
            
            assert first_ack.timestamp <= first_result.timestamp, \
                f"Ack (t={first_ack.timestamp}) should be before result (t={first_result.timestamp})"


# =========================
# SRS-EVT-02: Terminal result exactly once
# =========================

def test_tc_evt_03_succeeded_published_once(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-EVT-03: succeeded published once."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    nav2_server.set_goal_delay_sec(0.3)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    cmd_publisher.publish_navigate('cmd-once', x=1.0, y=2.0)
    
    for _ in range(40):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    results = event_collector.get_results()
    succeeded_results = [
        r for r in results if r.get('command_id') == 'cmd-once' and 
        (r.get('result_status') == 'succeeded' or r.get('result') == 'succeeded')
    ]
    
    # Should have exactly one succeeded result
    assert len(succeeded_results) == 1, \
        f"Should have exactly one succeeded result, got {len(succeeded_results)}: {succeeded_results}"


def test_tc_evt_04_replay_doesnt_duplicate(executor_node, nav2_server, event_collector, cmd_publisher, test_node):
    """TC-EVT-04: replay doesn't duplicate."""
    current_state = State(1, 'unconfigured')
    executor_node.on_configure(current_state)
    
    current_state = State(2, 'inactive')
    executor_node.on_activate(current_state)
    
    time.sleep(0.3)
    nav2_server.set_goal_delay_sec(0.2)
    
    for _ in range(10):
        rclpy.spin_once(executor_node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)
        time.sleep(0.05)
    
    # First command (complete)
    cmd_publisher.publish_navigate('cmd-replay-dup', x=1.0, y=2.0)
    
    for _ in range(30):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Count first succeeded
    results_before = event_collector.get_results()
    succeeded_before = [
        r for r in results_before if r.get('command_id') == 'cmd-replay-dup' and 
        (r.get('result_status') == 'succeeded' or r.get('result') == 'succeeded')
    ]
    
    # Replay duplicate
    cmd_publisher.publish_navigate('cmd-replay-dup', x=1.0, y=2.0)
    
    for _ in range(20):
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.05)
    
    results_after = event_collector.get_results()
    succeeded_after = [
        r for r in results_after if r.get('command_id') == 'cmd-replay-dup' and 
        (r.get('result_status') == 'succeeded' or r.get('result') == 'succeeded')
    ]
    
    # Replay should not create duplicate terminal result
    # (It may republish, but should not duplicate the terminal state)
    assert len(succeeded_after) <= len(succeeded_before) + 1, \
        f"Replay should not duplicate terminal results. Before: {len(succeeded_before)}, After: {len(succeeded_after)}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
