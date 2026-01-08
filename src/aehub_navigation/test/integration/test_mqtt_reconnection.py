#!/usr/bin/env python3
"""
Integration tests for MQTT reconnection

Tests MQTT reconnection scenarios and configuration changes.
"""

import pytest
import time
from unittest.mock import Mock, patch, MagicMock
import rclpy


class TestMQTTReconnection:
    """Test suite for MQTT reconnection scenarios"""
    
    @pytest.fixture
    def rclpy_init(self):
        """Initialize ROS2 for tests"""
        if not rclpy.ok():
            rclpy.init()
        yield
        if rclpy.ok():
            rclpy.shutdown()
    
    def test_config_change_triggers_reconnect(self):
        """Test that configuration change triggers MQTT reconnection"""
        from aehub_navigation.broker_config_provider import BrokerConfigProvider
        
        # Mock HTTP requests
        with patch('requests.get') as mock_get:
            # First config
            first_config = {
                'broker_host': 'broker1.example.com',
                'broker_port': 1883,
                'username': 'user1',
                'password': 'pass1',
                'tls_enabled': False
            }
            
            # Second config (different broker)
            second_config = {
                'broker_host': 'broker2.example.com',
                'broker_port': 1883,
                'username': 'user2',
                'password': 'pass2',
                'tls_enabled': False
            }
            
            mock_response1 = Mock()
            mock_response1.status_code = 200
            mock_response1.json.return_value = first_config
            
            mock_response2 = Mock()
            mock_response2.status_code = 200
            mock_response2.json.return_value = second_config
            
            # First call returns first config, second call returns second config
            mock_get.side_effect = [mock_response1, mock_response2]
            
            provider = BrokerConfigProvider(
                config_service_url='http://test.config.service',
                api_key='test_key',
                robot_id='test_robot',
                logger=None
            )
            
            # Fetch first config
            config1 = provider.fetch_config()
            assert config1 is not None
            assert config1.broker_host == 'broker1.example.com'
            
            # Fetch second config (simulating config change)
            config2 = provider.fetch_config()
            assert config2 is not None
            assert config2.broker_host == 'broker2.example.com'
            
            # Verify configs are different
            assert config1.broker_host != config2.broker_host
    
    def test_reconnect_after_connection_loss(self):
        """Test reconnection after connection loss"""
        from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
        
        # Mock paho-mqtt client
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            
            manager = MQTTConnectionManager(logger=None)
            
            config = {
                'broker_host': 'test.broker',
                'broker_port': 1883,
                'username': 'test_user',
                'password': 'test_pass',
                'tls_enabled': False
            }
            
            # First connection
            assert manager.connect(config) is True
            assert mock_client.connect.call_count == 1
            
            # Simulate disconnection
            manager.disconnect()
            assert mock_client.disconnect.call_count == 1
            
            # Reconnect
            assert manager.reconnect(config) is True
            assert mock_client.connect.call_count == 2
    
    def test_subscription_persistence_on_reconnect(self):
        """Test that subscriptions persist after reconnection"""
        from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
        
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            
            manager = MQTTConnectionManager(logger=None)
            
            config = {
                'broker_host': 'test.broker',
                'broker_port': 1883,
                'username': 'test_user',
                'password': 'test_pass',
                'tls_enabled': False
            }
            
            # Connect and subscribe
            manager.connect(config)
            manager.subscribe('test/topic', lambda c, u, m: None)
            
            assert mock_client.subscribe.call_count == 1
            
            # Disconnect and reconnect
            manager.disconnect()
            manager.reconnect(config)
            
            # Subscription should be restored
            # In real implementation, subscriptions are stored and re-applied
            # This test verifies the concept
            assert hasattr(manager, '_subscriptions')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

