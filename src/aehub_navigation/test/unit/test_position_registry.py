#!/usr/bin/env python3

"""
Unit tests for PositionRegistry

Tests loading positions from YAML and accessing them.
"""

import pytest
import yaml
import os
import tempfile
import math
from geometry_msgs.msg import PoseStamped
from aehub_navigation.position_registry import PositionRegistry


class TestPositionRegistry:
    """Test suite for PositionRegistry"""
    
    @pytest.fixture
    def registry(self):
        """Create a PositionRegistry instance"""
        return PositionRegistry()
    
    @pytest.fixture
    def valid_yaml_file(self):
        """Create a valid YAML file with 5 positions"""
        positions = {
            'positions': {
                'position_A': {
                    'x': 1.0,
                    'y': 2.0,
                    'theta': 0.0,
                    'description': 'Test Position A'
                },
                'position_B': {
                    'x': 2.0,
                    'y': 3.0,
                    'theta': 1.57,
                    'description': 'Test Position B'
                },
                'position_C': {
                    'x': 3.0,
                    'y': 4.0,
                    'theta': 0.0,
                    'description': 'Test Position C'
                },
                'position_D': {
                    'x': 4.0,
                    'y': 5.0,
                    'theta': -1.57,
                    'description': 'Test Position D'
                },
                'position_E': {
                    'x': 5.0,
                    'y': 6.0,
                    'theta': 3.14,
                    'description': 'Test Position E'
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(positions, f)
            temp_file = f.name
        
        yield temp_file
        
        # Cleanup
        if os.path.exists(temp_file):
            os.unlink(temp_file)
    
    @pytest.fixture
    def small_yaml_file(self):
        """Create a YAML file with 2 positions (dynamic positions support)"""
        positions = {
            'positions': {
                'position_A': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
                'position_B': {'x': 2.0, 'y': 3.0, 'theta': 1.57}
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(positions, f)
            temp_file = f.name
        
        yield temp_file
        
        if os.path.exists(temp_file):
            os.unlink(temp_file)
    
    def test_load_valid_yaml(self, registry, valid_yaml_file):
        """Test loading valid YAML with 5 positions"""
        result = registry.loadFromYAML(valid_yaml_file)
        assert result is True
        assert registry.getPositionCount() == 5
    
    def test_load_dynamic_count(self, registry, small_yaml_file):
        """Test loading YAML with any number of positions (dynamic support)"""
        result = registry.loadFromYAML(small_yaml_file)
        assert result is True
        assert registry.getPositionCount() == 2
    
    def test_load_nonexistent_file(self, registry):
        """Test loading non-existent file"""
        result = registry.loadFromYAML('/nonexistent/path/positions.yaml')
        assert result is False
    
    def test_load_invalid_yaml_format(self, registry):
        """Test loading file with invalid YAML format"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write('invalid yaml content: [[[[')
            temp_file = f.name
        
        try:
            result = registry.loadFromYAML(temp_file)
            assert result is False
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_load_missing_positions_key(self, registry):
        """Test loading YAML without 'positions' key"""
        invalid_yaml = {'other_key': {}}
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(invalid_yaml, f)
            temp_file = f.name
        
        try:
            result = registry.loadFromYAML(temp_file)
            assert result is False
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)
    
    def test_get_position_valid_id(self, registry, valid_yaml_file):
        """Test getting position by valid ID"""
        registry.loadFromYAML(valid_yaml_file)
        
        pose = registry.getPosition('position_A')
        assert pose is not None
        assert isinstance(pose, PoseStamped)
        assert pose.header.frame_id == 'map'
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 2.0
        assert pose.pose.position.z == 0.0
    
    def test_get_position_invalid_id(self, registry, valid_yaml_file):
        """Test getting position by invalid ID"""
        registry.loadFromYAML(valid_yaml_file)
        
        pose = registry.getPosition('position_Z')
        assert pose is None
    
    def test_has_position_valid(self, registry, valid_yaml_file):
        """Test hasPosition with valid ID"""
        registry.loadFromYAML(valid_yaml_file)
        
        assert registry.hasPosition('position_A') is True
        assert registry.hasPosition('position_B') is True
        assert registry.hasPosition('position_C') is True
        assert registry.hasPosition('position_D') is True
        assert registry.hasPosition('position_E') is True
    
    def test_has_position_invalid(self, registry, valid_yaml_file):
        """Test hasPosition with invalid ID"""
        registry.loadFromYAML(valid_yaml_file)
        
        assert registry.hasPosition('position_Z') is False
        assert registry.hasPosition('invalid') is False
    
    def test_get_all_position_ids(self, registry, valid_yaml_file):
        """Test getting all position IDs"""
        registry.loadFromYAML(valid_yaml_file)
        
        ids = registry.getAllPositionIds()
        assert len(ids) == 5
        assert 'position_A' in ids
        assert 'position_B' in ids
        assert 'position_C' in ids
        assert 'position_D' in ids
        assert 'position_E' in ids
    
    def test_get_position_count(self, registry, valid_yaml_file):
        """Test getting position count"""
        registry.loadFromYAML(valid_yaml_file)
        
        assert registry.getPositionCount() == 5
    
    def test_theta_to_quaternion_conversion(self, registry, valid_yaml_file):
        """Test that theta is correctly converted to quaternion"""
        registry.loadFromYAML(valid_yaml_file)
        
        # Test position_B with theta = 1.57 (90 degrees)
        pose = registry.getPosition('position_B')
        assert pose.pose.orientation.x == 0.0
        assert pose.pose.orientation.y == 0.0
        
        # Check quaternion values for 90 degrees
        expected_z = math.sin(1.57 / 2.0)
        expected_w = math.cos(1.57 / 2.0)
        assert abs(pose.pose.orientation.z - expected_z) < 0.001
        assert abs(pose.pose.orientation.w - expected_w) < 0.001
    
    def test_theta_zero_conversion(self, registry, valid_yaml_file):
        """Test quaternion conversion for theta = 0"""
        registry.loadFromYAML(valid_yaml_file)
        
        pose = registry.getPosition('position_A')  # theta = 0.0
        assert pose.pose.orientation.z == 0.0
        assert abs(pose.pose.orientation.w - 1.0) < 0.001
    
    def test_frame_id_is_map(self, registry, valid_yaml_file):
        """Test that frame_id is always 'map'"""
        registry.loadFromYAML(valid_yaml_file)
        
        for position_id in registry.getAllPositionIds():
            pose = registry.getPosition(position_id)
            assert pose.header.frame_id == 'map'
    
    def test_z_always_zero(self, registry, valid_yaml_file):
        """Test that z position is always 0.0"""
        registry.loadFromYAML(valid_yaml_file)
        
        for position_id in registry.getAllPositionIds():
            pose = registry.getPosition(position_id)
            assert pose.pose.position.z == 0.0
    
    def test_all_positions_have_required_fields(self, registry, valid_yaml_file):
        """Test that all loaded positions have required fields"""
        registry.loadFromYAML(valid_yaml_file)
        
        for position_id in registry.getAllPositionIds():
            pos_data = registry.positions[position_id]
            assert 'x' in pos_data
            assert 'y' in pos_data
            assert 'theta' in pos_data
            assert isinstance(pos_data['x'], (int, float))
            assert isinstance(pos_data['y'], (int, float))
            assert isinstance(pos_data['theta'], (int, float))
    
    def test_load_replaces_existing_positions(self, registry, valid_yaml_file):
        """Test that loading new YAML replaces existing positions"""
        # Load first time
        registry.loadFromYAML(valid_yaml_file)
        assert registry.getPositionCount() == 5
        
        # Create new YAML with different positions (3 positions instead of 5)
        new_positions = {
            'positions': {
                'position_1': {'x': 10.0, 'y': 20.0, 'theta': 0.0, 'description': ''},
                'position_2': {'x': 20.0, 'y': 30.0, 'theta': 1.57, 'description': ''},
                'position_3': {'x': 30.0, 'y': 40.0, 'theta': 0.0, 'description': ''}
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(new_positions, f)
            new_file = f.name
        
        try:
            # Load new file
            result = registry.loadFromYAML(new_file)
            assert result is True
            assert registry.getPositionCount() == 3  # Dynamic count
            
            # Old positions should be gone
            assert not registry.hasPosition('position_A')
            # New positions should exist
            assert registry.hasPosition('position_1')
        finally:
            if os.path.exists(new_file):
                os.unlink(new_file)
    
    def test_add_position(self, registry, valid_yaml_file):
        """Test adding a new position"""
        registry.loadFromYAML(valid_yaml_file)
        initial_count = registry.getPositionCount()
        
        result = registry.addPosition('position_F', 10.0, 20.0, 1.57, 'New position')
        assert result is True
        assert registry.getPositionCount() == initial_count + 1
        assert registry.hasPosition('position_F')
    
    def test_remove_position(self, registry, valid_yaml_file):
        """Test removing a position"""
        registry.loadFromYAML(valid_yaml_file)
        initial_count = registry.getPositionCount()
        
        result = registry.removePosition('position_A')
        assert result is True
        assert registry.getPositionCount() == initial_count - 1
        assert not registry.hasPosition('position_A')
    
    def test_remove_nonexistent_position(self, registry, valid_yaml_file):
        """Test removing a non-existent position"""
        registry.loadFromYAML(valid_yaml_file)
        initial_count = registry.getPositionCount()
        
        result = registry.removePosition('position_Z')
        assert result is False
        assert registry.getPositionCount() == initial_count
    
    def test_save_to_yaml(self, registry, valid_yaml_file):
        """Test saving positions to YAML file"""
        registry.loadFromYAML(valid_yaml_file)
        
        # Add a new position
        registry.addPosition('position_F', 10.0, 20.0, 1.57, 'New position')
        
        # Save to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name
        
        try:
            result = registry.saveToYAML(temp_file)
            assert result is True
            
            # Load back and verify
            new_registry = PositionRegistry()
            result = new_registry.loadFromYAML(temp_file)
            assert result is True
            assert new_registry.getPositionCount() == 6
            assert new_registry.hasPosition('position_F')
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

