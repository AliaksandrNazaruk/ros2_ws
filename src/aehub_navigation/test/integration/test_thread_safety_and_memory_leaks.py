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
Thread Safety and Memory Leak Tests

High-value tests for:
- Thread safety of shared state
- Memory leak detection
- File locking correctness
"""

import pytest
import threading
import time
import tempfile
import os
import gc
import sys
from unittest.mock import Mock, MagicMock, patch
import rclpy
from rclpy.node import Node

from aehub_navigation.position_registry import PositionRegistry
from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState
from aehub_navigation.broker_config_provider import BrokerConfigProvider, BrokerConfig
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager


class TestPositionRegistryThreadSafety:
    """
    Test Strategy: PositionRegistry now has locks for thread-safe access.
    Multiple threads should be able to read/write positions concurrently
    without data corruption.
    
    Risks Covered:
    - Concurrent add/remove operations
    - Concurrent read/write operations
    - File locking during concurrent saves
    """
    
    def test_concurrent_add_remove_positions_thread_safe(self):
        """
        Bug it would catch: Concurrent add/remove operations without locks
        cause KeyError, data corruption, or inconsistent state.
        
        Real-world scenario: Multiple API requests modify positions
        simultaneously, causing crashes or data loss.
        """
        registry = PositionRegistry()
        errors = []
        results = []
        
        def add_positions():
            for i in range(100):
                try:
                    result = registry.addPosition(f"pos_{i}", float(i), float(i+1), 0.0)
                    results.append(('add', i, result))
                except Exception as e:
                    errors.append(('add', i, str(e)))
        
        def remove_positions():
            for i in range(50, 100):
                try:
                    result = registry.removePosition(f"pos_{i}")
                    results.append(('remove', i, result))
                except Exception as e:
                    errors.append(('remove', i, str(e)))
        
        # Run concurrently
        threads = []
        for _ in range(5):
            threads.append(threading.Thread(target=add_positions))
        for _ in range(3):
            threads.append(threading.Thread(target=remove_positions))
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # Then: No errors should occur
        assert len(errors) == 0, f"Concurrent operations should not cause errors: {errors}"
        
        # Verify final state is consistent
        count = registry.getPositionCount()
        assert count >= 0, "Position count should be non-negative"
    
    def test_concurrent_file_save_with_locking(self):
        """
        Bug it would catch: Concurrent saves without file locking cause
        file corruption or data loss.
        
        Real-world scenario: Multiple processes save positions simultaneously,
        file gets corrupted.
        """
        registry = PositionRegistry()
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        registry.addPosition("pos_B", 3.0, 4.0, 1.57)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            errors = []
            
            def save_positions(thread_id):
                try:
                    # Each thread adds a position and saves
                    registry.addPosition(f"pos_thread_{thread_id}", float(thread_id), float(thread_id+1), 0.0)
                    result = registry.saveToYAML(temp_file)
                    if not result:
                        errors.append(f"Thread {thread_id} save failed")
                except Exception as e:
                    errors.append(f"Thread {thread_id} error: {e}")
            
            # Run 10 concurrent saves
            threads = []
            for i in range(10):
                t = threading.Thread(target=save_positions, args=(i,))
                threads.append(t)
                t.start()
            
            for t in threads:
                t.join()
            
            # Then: All saves should succeed (file locking prevents corruption)
            assert len(errors) == 0, f"Concurrent saves should succeed with file locking: {errors}"
            
            # Verify file is valid and loadable
            new_registry = PositionRegistry()
            load_result = new_registry.loadFromYAML(temp_file)
            assert load_result is True, "File should be valid after concurrent saves"
            assert new_registry.getPositionCount() > 0, "Positions should be preserved"
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)


class TestNavigationStateManagerThreadSafety:
    """
    Test Strategy: NavigationStateManager now has locks for state access.
    Multiple threads should be able to read/write state concurrently.
    
    Risks Covered:
    - Concurrent state updates
    - Concurrent getters
    - State consistency
    """
    
    def test_concurrent_state_updates_thread_safe(self):
        """
        Bug it would catch: Concurrent state updates without locks cause
        inconsistent state or lost updates.
        
        Real-world scenario: Multiple callbacks update state simultaneously,
        state becomes inconsistent.
        """
        manager = NavigationStateManager()
        errors = []
        
        def update_state(thread_id):
            try:
                for i in range(50):
                    manager.setState(NavigationState.NAVIGATING, target_id=f"target_{thread_id}_{i}")
                    state = manager.getState()
                    target = manager.getTargetId()
                    # Verify state consistency
                    if state == NavigationState.NAVIGATING and target is None:
                        errors.append(f"Thread {thread_id}: Inconsistent state")
            except Exception as e:
                errors.append(f"Thread {thread_id}: {e}")
        
        # Run 5 concurrent state updates
        threads = []
        for i in range(5):
            t = threading.Thread(target=update_state, args=(i,))
            threads.append(t)
            t.start()
        
        for t in threads:
            t.join()
        
        # Then: No errors should occur
        assert len(errors) == 0, f"Concurrent state updates should not cause errors: {errors}"
        
        # Verify final state is valid
        state = manager.getState()
        assert state in NavigationState, "Final state should be valid"


class TestMemoryLeaks:
    """
    Test Strategy: Check for memory leaks in:
    - CA certificate temp files
    - Callback registrations
    - Resource cleanup
    
    Risks Covered:
    - Temp files not cleaned up
    - Callbacks accumulating
    - Resource leaks
    """
    
    def test_ca_cert_file_cleanup_on_exit(self):
        """
        Bug it would catch: CA certificate temp file not cleaned up on process exit,
        causing disk space leak.
        
        Real-world scenario: Node restarts frequently, temp files accumulate,
        disk fills up.
        """
        import atexit
        
        # Count registered cleanup functions
        initial_handlers = len(atexit._exitfuncs) if hasattr(atexit, '_exitfuncs') else 0
        
        # Create manager (registers atexit handler)
        manager = MQTTConnectionManager()
        
        # Simulate fetching CA cert (creates temp file)
        with patch('requests.get') as mock_get:
            mock_response = MagicMock()
            mock_response.status_code = 200
            mock_response.text = "-----BEGIN CERTIFICATE-----\nTEST\n-----END CERTIFICATE-----"
            mock_get.return_value = mock_response
            
            ca_path = manager._fetch_ca_certificate()
        
        # Verify temp file was created
        if ca_path:
            assert os.path.exists(ca_path), "CA cert temp file should exist"
            
            # Manually call cleanup (simulating exit)
            manager._cleanup_ca_cert()
            
            # Verify file was deleted
            assert not os.path.exists(ca_path), "CA cert temp file should be cleaned up"
    
    def test_callback_unwatch_prevents_memory_leak(self):
        """
        Bug it would catch: Callbacks accumulate in BrokerConfigProvider,
        causing memory leak.
        
        Real-world scenario: Node reconnects frequently, callbacks accumulate,
        memory usage grows.
        """
        provider = BrokerConfigProvider(
            config_service_url="http://localhost:7900",
            api_key="test_key"
        )
        
        # Register multiple callbacks
        callbacks = []
        for i in range(100):
            callback = Mock()
            provider.watch(callback)
            callbacks.append(callback)
        
        # Verify callbacks are registered
        with provider._lock:
            initial_count = len(provider._change_callbacks)
        
        assert initial_count == 100, "All callbacks should be registered"
        
        # Unwatch half of them
        for callback in callbacks[:50]:
            provider.unwatch(callback)
        
        # Verify callbacks are removed
        with provider._lock:
            final_count = len(provider._change_callbacks)
        
        assert final_count == 50, "Unwatched callbacks should be removed"
        
        # Verify remaining callbacks are correct
        with provider._lock:
            remaining_callbacks = provider._change_callbacks.copy()
        
        assert all(cb in remaining_callbacks for cb in callbacks[50:]), \
            "Remaining callbacks should be correct"


class TestFileLockingAndAtomicWrite:
    """
    Test Strategy: PositionRegistry now uses file locking and atomic writes.
    Tests verify that concurrent file operations are safe.
    
    Risks Covered:
    - Concurrent file reads/writes
    - Partial writes causing corruption
    - File locking correctness
    """
    
    def test_atomic_write_prevents_corruption(self):
        """
        Bug it would catch: Non-atomic write causes file corruption on crash.
        
        Real-world scenario: Process crashes during save, file corrupted,
        positions lost.
        """
        registry = PositionRegistry()
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        registry.addPosition("pos_B", 3.0, 4.0, 1.57)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            # Save using atomic write
            result = registry.saveToYAML(temp_file)
            assert result is True, "Save should succeed"
            
            # Verify file is valid (atomic write ensures it's never corrupted)
            new_registry = PositionRegistry()
            load_result = new_registry.loadFromYAML(temp_file)
            assert load_result is True, "Saved file should be loadable"
            assert new_registry.getPositionCount() == 2, "All positions should be preserved"
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_path_validation_prevents_traversal(self):
        """
        Bug it would catch: Path traversal attack allows writing to arbitrary files.
        
        Real-world scenario: Attacker provides path like "../../etc/passwd",
        system writes to sensitive files.
        """
        # Create test directory
        test_dir = "/tmp/test_positions"
        os.makedirs(test_dir, exist_ok=True)
        
        registry = PositionRegistry(allowed_base_dir=test_dir)
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        
        # Try path traversal attack (normalized path will be /etc/passwd)
        # Get original file mtime if it exists
        original_mtime = None
        if os.path.exists("/etc/passwd"):
            original_mtime = os.path.getmtime("/etc/passwd")
        
        # saveToYAML catches ValueError and returns False
        result = registry.saveToYAML("/tmp/test_positions/../../etc/passwd")
        assert result is False, "Path traversal should be rejected"
        
        # Verify file was not modified (if it existed)
        if original_mtime is not None:
            current_mtime = os.path.getmtime("/etc/passwd")
            assert current_mtime == original_mtime, \
                "Sensitive file should not be modified"
        
        # Valid path should work
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, dir=test_dir) as f:
            valid_file = f.name
        
        try:
            result = registry.saveToYAML(valid_file)
            assert result is True, "Valid path should work"
        finally:
            if os.path.exists(valid_file):
                os.unlink(valid_file)
            # Cleanup test directory
            try:
                os.rmdir(test_dir)
            except:
                pass


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])

