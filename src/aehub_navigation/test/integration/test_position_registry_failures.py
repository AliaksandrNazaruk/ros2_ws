#!/usr/bin/env python3
"""
Position Registry Failure Mode Tests

High-value tests for file I/O, data corruption, and concurrent access scenarios.
"""

import pytest
import tempfile
import os
import yaml
import threading
import time
from aehub_navigation.position_registry import PositionRegistry


class TestPositionRegistryFileCorruption:
    """
    System Understanding:
    - PositionRegistry loads from YAML file
    - File can be corrupted, partially written, or locked
    - Registry must handle file errors gracefully
    
    Failure Analysis:
    - Partial write: File written during save, process crashes
    - File corruption: Invalid YAML syntax
    - Permission errors: File not readable/writable
    - Concurrent access: Multiple processes modify file simultaneously
    """
    
    def test_partial_yaml_write_causes_corruption(self):
        """
        Bug it would catch: saveToYAML() writes file, but process crashes mid-write,
        leaving corrupted YAML that cannot be loaded.
        
        Real-world scenario: System crash during position save, file corrupted,
        next load fails, positions lost.
        """
        registry = PositionRegistry()
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        registry.addPosition("pos_B", 3.0, 4.0, 1.57)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            # Simulate partial write (file truncated mid-write)
            with open(temp_file, 'w') as f:
                f.write('positions:\n')
                f.write('  pos_A:\n')
                f.write('    x: 1.0\n')
                f.write('    y: 2.0\n')
                # File truncated here (simulating crash)
                f.truncate()
            
            # When: Loading corrupted file
            result = registry.loadFromYAML(temp_file)
            
            # Then: Should handle gracefully, not crash
            # BUG: If we don't handle YAML parse errors, system crashes
            assert result is False, "Corrupted file should be rejected, not crash"
            assert len(registry.positions) == 0 or len(registry.positions) == 2, \
                "Registry should either be empty or preserve old state"
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_invalid_yaml_syntax_does_not_crash(self):
        """
        Bug it would catch: Invalid YAML syntax in file causes YAML parser to raise
        exception, crashing the node.
        
        Real-world scenario: Admin edits file manually, makes syntax error,
        node crashes on next load.
        """
        registry = PositionRegistry()
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
            f.write('positions:\n')
            f.write('  pos_A:\n')
            f.write('    x: 1.0\n')
            f.write('    y: 2.0  # Missing comma or invalid syntax\n')
            f.write('    invalid: [unclosed list\n')  # Invalid YAML
        
        try:
            # When: Loading invalid YAML
            result = registry.loadFromYAML(temp_file)
            
            # Then: Should return False, not raise exception
            # BUG: If exception is not caught, node crashes
            assert result is False, "Invalid YAML should be rejected gracefully"
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_file_permission_error_handled_gracefully(self):
        """
        Bug it would catch: File exists but not readable, or directory not writable,
        causing unhandled exception.
        
        Real-world scenario: File permissions changed, node cannot read/write,
        crashes instead of logging error.
        """
        registry = PositionRegistry()
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
            f.write('positions:\n')
            f.write('  pos_A:\n')
            f.write('    x: 1.0\n')
            f.write('    y: 2.0\n')
            f.write('    theta: 0.0\n')
        
        try:
            # Remove read permission (simulate permission error)
            os.chmod(temp_file, 0o000)
            
            # When: Loading file without permission
            result = registry.loadFromYAML(temp_file)
            
            # Then: Should return False, not raise PermissionError
            # BUG: If PermissionError is not caught, node crashes
            assert result is False, "Permission error should be handled gracefully"
        
        finally:
            # Restore permission for cleanup
            os.chmod(temp_file, 0o644)
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_concurrent_file_modification_detection(self):
        """
        Bug it would catch: File modified by another process during load,
        causing inconsistent state or data loss.
        
        Real-world scenario: Admin edits file while node is loading,
        node loads partial/corrupted data.
        """
        registry1 = PositionRegistry()
        registry2 = PositionRegistry()
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            # Initial file
            with open(temp_file, 'w') as f:
                yaml.dump({
                    'positions': {
                        'pos_A': {'x': 1.0, 'y': 2.0, 'theta': 0.0}
                    }
                }, f)
            
            # Registry1 starts loading
            # Simulate: File is modified during load (by registry2)
            def modify_file():
                time.sleep(0.01)  # Small delay
                with open(temp_file, 'w') as f:
                    yaml.dump({
                        'positions': {
                            'pos_B': {'x': 3.0, 'y': 4.0, 'theta': 1.57}
                        }
                    }, f)
            
            # Start modification in background
            thread = threading.Thread(target=modify_file)
            thread.start()
            
            # Load file (may be modified during load)
            result = registry1.loadFromYAML(temp_file)
            
            thread.join()
            
            # Then: Should either load successfully or fail gracefully
            # BUG: If we don't handle concurrent modification, we get corrupted data
            if result:
                # If load succeeded, data should be valid
                assert len(registry1.positions) > 0, "Loaded positions should be valid"
                # Check that all positions have required fields
                for pos_id, pos_data in registry1.positions.items():
                    assert 'x' in pos_data
                    assert 'y' in pos_data
                    assert 'theta' in pos_data
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)


class TestPositionRegistryDataIntegrity:
    """
    System Understanding:
    - Positions stored in memory dict
    - addPosition/removePosition modify dict
    - getPosition must return consistent data
    
    Failure Analysis:
    - Data corruption: Invalid values in dict
    - Type errors: Wrong types in position data
    - Missing fields: Required fields missing after load
    """
    
    def test_add_position_with_invalid_data_corrupts_registry(self):
        """
        Bug it would catch: addPosition() accepts invalid data (NaN, inf, None),
        corrupting registry, causing getPosition() to return invalid poses.
        
        Real-world scenario: API receives invalid coordinates, adds to registry,
        subsequent navigation commands fail with invalid poses.
        """
        registry = PositionRegistry()
        
        # When: Adding position with invalid data
        # BUG: If we don't validate, invalid data corrupts registry
        try:
            result = registry.addPosition("bad_pos", float('nan'), float('inf'), None)
            # Should reject invalid data
            assert result is False, "Invalid data (NaN, inf, None) should be rejected"
        except (ValueError, TypeError):
            # Also acceptable: raise exception
            pass
        
        # Then: Registry should not contain invalid data
        pos = registry.getPosition("bad_pos")
        assert pos is None, "Invalid position should not be in registry"
    
    def test_remove_position_during_get_causes_inconsistency(self):
        """
        Bug it would catch: removePosition() called while getPosition() is executing,
        causing KeyError or inconsistent state.
        
        Real-world scenario: API delete request arrives while navigation command
        is resolving position, causing race condition.
        """
        registry = PositionRegistry()
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        
        # Simulate concurrent access
        get_result = None
        remove_result = None
        
        def get_position():
            nonlocal get_result
            get_result = registry.getPosition("pos_A")
        
        def remove_position():
            nonlocal remove_result
            remove_result = registry.removePosition("pos_A")
        
        # Execute concurrently
        thread1 = threading.Thread(target=get_position)
        thread2 = threading.Thread(target=remove_position)
        
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        
        # Then: Should handle gracefully
        # BUG: If not thread-safe, KeyError or inconsistent state
        # getPosition should either return None or valid pose, not crash
        # Check that result is either None or a valid PoseStamped
        from geometry_msgs.msg import PoseStamped
        assert get_result is None or isinstance(get_result, PoseStamped), \
            "Concurrent access should not cause crash, result should be None or PoseStamped"
    
    def test_save_to_yaml_preserves_all_positions(self):
        """
        Bug it would catch: saveToYAML() fails partway through, only some positions saved,
        causing data loss on next load.
        
        Real-world scenario: Disk full or I/O error during save, partial write,
        positions lost.
        """
        registry = PositionRegistry()
        registry.addPosition("pos_A", 1.0, 2.0, 0.0)
        registry.addPosition("pos_B", 3.0, 4.0, 1.57)
        registry.addPosition("pos_C", 5.0, 6.0, 3.14)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            # When: Saving positions
            result = registry.saveToYAML(temp_file)
            
            # Then: All positions should be saved
            assert result is True, "Save should succeed"
            
            # Verify by loading
            new_registry = PositionRegistry()
            load_result = new_registry.loadFromYAML(temp_file)
            
            assert load_result is True, "Saved file should be loadable"
            assert len(new_registry.positions) == 3, "All positions should be preserved"
            assert "pos_A" in new_registry.positions
            assert "pos_B" in new_registry.positions
            assert "pos_C" in new_registry.positions
        
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])

