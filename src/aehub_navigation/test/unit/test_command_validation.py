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
Unit tests for command validation

Tests validation logic for navigateTo and cancel commands.
"""

import pytest
import rclpy
from unittest.mock import Mock, MagicMock
from aehub_navigation.navigation_integrated_node import NavigationIntegratedNode
from aehub_navigation.position_registry import PositionRegistry
import os
import tempfile
import yaml


class TestCommandValidation:
    """Test suite for command validation"""
    
    @pytest.fixture(scope="class")
    def init_rclpy(self):
        """Initialize rclpy once for all tests"""
        try:
            rclpy.init()
            yield
        finally:
            rclpy.shutdown()
    
    @pytest.fixture
    def temp_positions_file(self):
        """Create a temporary positions file for testing"""
        positions = {
            'positions': {
                'position_A': {'x': 1.0, 'y': 2.0, 'theta': 0.0, 'description': 'Test A'},
                'position_B': {'x': 2.0, 'y': 3.0, 'theta': 1.57, 'description': 'Test B'},
                'position_C': {'x': 3.0, 'y': 4.0, 'theta': 0.0, 'description': 'Test C'},
                'position_D': {'x': 4.0, 'y': 5.0, 'theta': -1.57, 'description': 'Test D'},
                'position_E': {'x': 5.0, 'y': 6.0, 'theta': 3.14, 'description': 'Test E'},
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
    def nav_node(self, init_rclpy, temp_positions_file):
        """Create a NavigationIntegratedNode instance with mocked dependencies"""
        # We'll create a minimal node just for testing validation
        # Mock the complex dependencies
        node = NavigationIntegratedNode.__new__(NavigationIntegratedNode)
        rclpy.node.Node.__init__(node, 'test_navigation_node')
        node.robot_id = "test_robot"
        
        # Setup position registry
        node.position_registry = PositionRegistry()
        node.position_registry.loadFromYAML(temp_positions_file)
        
        # Setup command validator (required for validateCommand method)
        from aehub_navigation.command_validator import CommandValidator
        node.command_validator = CommandValidator(max_processed_ids=1000, ttl_seconds=3600)
        
        yield node
        node.destroy_node()
    
    # Tests for navigateTo command validation
    
    def test_valid_navigate_to_command(self, nav_node):
        """Test valid navigateTo command"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is True
        assert error_msg is None
    
    def test_missing_command_id(self, nav_node):
        """Test navigateTo command without command_id"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'command_id' in error_msg
    
    def test_missing_timestamp(self, nav_node):
        """Test navigateTo command without timestamp"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'timestamp' in error_msg
    
    def test_missing_target_id(self, nav_node):
        """Test navigateTo command without target_id"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'target_id' in error_msg
    
    def test_missing_priority(self, nav_node):
        """Test navigateTo command without priority"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'priority' in error_msg
    
    def test_invalid_priority_low(self, nav_node):
        """Test navigateTo command with invalid priority 'low' (forbidden)"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'low'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'low' in error_msg.lower() or 'invalid' in error_msg.lower()
    
    def test_invalid_priority_random(self, nav_node):
        """Test navigateTo command with invalid priority value"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'invalid_priority'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'invalid' in error_msg.lower()
    
    def test_valid_priority_normal(self, nav_node):
        """Test navigateTo command with valid priority 'normal'"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is True
    
    def test_valid_priority_high(self, nav_node):
        """Test navigateTo command with valid priority 'high'"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'high'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is True
    
    def test_valid_priority_emergency(self, nav_node):
        """Test navigateTo command with valid priority 'emergency'"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'emergency'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is True
    
    def test_invalid_target_id(self, nav_node):
        """Test navigateTo command with unknown target_id"""
        command = {
            'schema_version': '1.0',
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_Z',  # Doesn't exist
            'priority': 'normal'
        }
        
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'position_Z' in error_msg or 'unknown' in error_msg.lower()
    
    def test_all_valid_target_ids(self, nav_node):
        """Test navigateTo command with all valid target_ids"""
        valid_targets = ['position_A', 'position_B', 'position_C', 'position_D', 'position_E']
        
        for target_id in valid_targets:
            command = {
                'schema_version': '1.0',
                'robot_id': nav_node.robot_id,
                'command_id': '550e8400-e29b-41d4-a716-446655440000',
                'timestamp': '2025-12-29T12:00:00Z',
                'target_id': target_id,
                'priority': 'normal'
            }
            
            is_valid, error_msg = nav_node.validateCommand(command)
            assert is_valid is True, f"Target {target_id} should be valid"

    def test_missing_schema_version_rejected(self, nav_node):
        command = {
            'robot_id': nav_node.robot_id,
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'schema_version' in (error_msg or '')

    def test_robot_id_mismatch_rejected(self, nav_node):
        command = {
            'schema_version': '1.0',
            'robot_id': 'other_robot',
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        is_valid, error_msg = nav_node.validateCommand(command)
        assert is_valid is False
        assert 'robot_id mismatch' in (error_msg or '').lower()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

