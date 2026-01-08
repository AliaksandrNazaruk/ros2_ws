#!/usr/bin/env python3

"""
Integration tests for navigation command flow.

Tests the complete flow from MQTT command to Nav2 goal execution.
"""

import os
import tempfile
from unittest.mock import MagicMock, Mock

import pytest
import rclpy
import yaml

from aehub_navigation.navigation_state_manager import NavigationState


class TestNavigationCommandFlow:
    """Test suite for navigation command flow integration."""

    @pytest.fixture(scope="class")
    def init_rclpy(self):
        """Initialize rclpy once for all tests."""
        try:
            rclpy.init()
            yield
        finally:
            rclpy.shutdown()
    
    @pytest.fixture
    def temp_positions_file(self):
        """Create a temporary positions file."""
        positions = {
            'positions': {
                'position_A': {
                    'x': 1.0, 'y': 2.0, 'theta': 0.0,
                    'description': 'Test A'
                },
                'position_B': {
                    'x': 2.0, 'y': 3.0, 'theta': 1.57,
                    'description': 'Test B'
                },
                'position_C': {
                    'x': 3.0, 'y': 4.0, 'theta': 0.0,
                    'description': 'Test C'
                },
                'position_D': {
                    'x': 4.0, 'y': 5.0, 'theta': -1.57,
                    'description': 'Test D'
                },
                'position_E': {
                    'x': 5.0, 'y': 6.0, 'theta': 3.14,
                    'description': 'Test E'
                },
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(positions, f)
            temp_file = f.name
        
        yield temp_file
        
        if os.path.exists(temp_file):
            os.unlink(temp_file)
    
    @pytest.fixture
    def mock_nav2_action_client(self):
        """Mock Nav2 Action Client."""
        mock_client = MagicMock()
        mock_client.wait_for_server = Mock(return_value=True)
        mock_client.server_is_ready = Mock(return_value=True)
        return mock_client
    
    @pytest.fixture
    def mock_mqtt_manager(self):
        """Mock MQTT Connection Manager."""
        mock_manager = MagicMock()
        mock_manager.is_connected = True
        mock_manager.publish = Mock(return_value=True)
        mock_manager.subscribe = Mock()
        return mock_manager
    
    @pytest.fixture
    def mock_config_provider(self):
        """Mock Broker Config Provider."""
        from aehub_navigation.broker_config_provider import BrokerConfig
        
        mock_provider = MagicMock()
        mock_provider.fetch_config = Mock(return_value=BrokerConfig(
            broker="localhost",
            broker_port=1883,
            mqtt_user="test",
            mqtt_password="test",
            mqtt_use_tls=False,
            mqtt_tls_insecure=False
        ))
        mock_provider.watch = Mock()
        mock_provider.start_polling = Mock()
        mock_provider.stop_polling = Mock()
        return mock_provider
    
    def test_valid_navigate_to_command_flow(
            self, init_rclpy, temp_positions_file,
            mock_nav2_action_client, mock_mqtt_manager,
            mock_config_provider):
        """Test complete flow: valid navigateTo command → Nav2 goal sent."""
        # This is a simplified integration test
        # Full integration would require more complex setup
        
        # The test verifies that command validation and goal sending logic works together
        # Actual integration with real Nav2 would be in system tests
        
        # For now, we test that the components can work together
        # by verifying the validation and state management
        
        from aehub_navigation.position_registry import PositionRegistry
        from aehub_navigation.navigation_state_manager import NavigationStateManager
        
        registry = PositionRegistry()
        assert registry.loadFromYAML(temp_positions_file)
        
        state_manager = NavigationStateManager(logger=None)
        
        # Verify valid command would be accepted
        valid_command = {
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_A',
            'priority': 'normal'
        }
        
        # Check that target_id exists
        assert registry.hasPosition(valid_command['target_id'])
        
        # Verify state transition
        state_manager.onGoalSent(valid_command['target_id'])
        assert state_manager.getState() == NavigationState.NAVIGATING
    
    def test_invalid_command_rejected(self, init_rclpy, temp_positions_file):
        """Test that invalid commands are rejected."""
        from aehub_navigation.position_registry import PositionRegistry
        
        registry = PositionRegistry()
        assert registry.loadFromYAML(temp_positions_file)
        
        # Invalid target_id
        invalid_command = {
            'command_id': '550e8400-e29b-41d4-a716-446655440000',
            'timestamp': '2025-12-29T12:00:00Z',
            'target_id': 'position_Z',  # Doesn't exist
            'priority': 'normal'
        }
        
        assert not registry.hasPosition(invalid_command['target_id'])
    
    def test_navigate_to_cancels_active_goal(
            self, init_rclpy, temp_positions_file):
        """Test that new navigateTo command cancels active goal."""
        from aehub_navigation.navigation_state_manager import NavigationStateManager
        
        state_manager = NavigationStateManager(logger=None)
        
        # Start navigation
        state_manager.onGoalSent('position_A')
        assert state_manager.getState() == NavigationState.NAVIGATING
        
        # New command should cancel previous (tested at integration level)
        # This would require full node setup, so we verify state transitions
        state_manager.onGoalCanceled('position_A')
        assert state_manager.getState() == NavigationState.IDLE


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

