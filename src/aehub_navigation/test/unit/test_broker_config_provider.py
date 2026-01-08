#!/usr/bin/env python3

"""
Unit tests for BrokerConfigProvider

Tests fetching configuration from Config Service and change detection.
"""

import pytest
import time
import threading
import json
import requests
from unittest.mock import Mock, patch, MagicMock
from aehub_navigation.broker_config_provider import BrokerConfigProvider, BrokerConfig, ConfigState


class TestBrokerConfigProvider:
    """Test suite for BrokerConfigProvider"""
    
    @pytest.fixture
    def config_service_url(self):
        """Config Service URL for testing"""
        return "http://localhost:7900"
    
    @pytest.fixture
    def api_key(self):
        """API key for testing"""
        return "test_api_key_123"
    
    @pytest.fixture
    def provider(self, config_service_url, api_key):
        """Create a BrokerConfigProvider instance"""
        return BrokerConfigProvider(config_service_url, api_key, logger=None)
    
    @pytest.fixture
    def valid_config_response(self):
        """Valid config service response"""
        return {
            "broker": "mqtt.example.com",
            "broker_port": 1883,
            "mqtt_user": "testuser",
            "mqtt_password": "testpass",
            "mqtt_use_tls": False,
            "mqtt_tls_insecure": False
        }
    
    @pytest.fixture
    def tls_config_response(self):
        """Config service response with TLS enabled"""
        return {
            "broker": "mqtt-secure.example.com",
            "broker_port": 8883,
            "mqtt_user": "tlsuser",
            "mqtt_password": "tlspass",
            "mqtt_use_tls": True,
            "mqtt_tls_insecure": True
        }
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_success(self, mock_get, provider, valid_config_response):
        """Test successful config fetch"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = valid_config_response
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        assert config is not None
        assert isinstance(config, BrokerConfig)
        assert config.broker == "mqtt.example.com"
        assert config.broker_port == 1883
        assert config.mqtt_user == "testuser"
        assert config.mqtt_password == "testpass"
        assert config.mqtt_use_tls is False
        assert config.mqtt_tls_insecure is False
        assert provider.state == ConfigState.READY
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_with_tls(self, mock_get, provider, tls_config_response):
        """Test config fetch with TLS enabled"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = tls_config_response
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        assert config is not None
        assert config.mqtt_use_tls is True
        assert config.mqtt_tls_insecure is True
        assert config.broker_port == 8883
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_http_404(self, mock_get, provider):
        """Test config fetch with HTTP 404 error"""
        mock_response = MagicMock()
        mock_response.status_code = 404
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        assert config is None
        assert provider.state == ConfigState.ERROR
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_http_500(self, mock_get, provider):
        """Test config fetch with HTTP 500 error"""
        mock_response = MagicMock()
        mock_response.status_code = 500
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        assert config is None
        assert provider.state == ConfigState.ERROR
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_timeout(self, mock_get, provider):
        """Test config fetch timeout"""
        # Mock timeout exception
        mock_get.side_effect = requests.exceptions.Timeout("Request timeout")
        
        config = provider.fetch_config()
        
        assert config is None
        assert provider.state == ConfigState.ERROR
    
    def test_fetch_config_connection_error(self, provider):
        """Test config fetch with connection error"""
        # Use invalid URL to trigger connection error
        provider.config_service_url = "http://nonexistent-host-12345:7900"
        
        config = provider.fetch_config()
        
        assert config is None
        assert provider.state == ConfigState.ERROR
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_invalid_json(self, mock_get, provider):
        """Test config fetch with invalid JSON response"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.side_effect = json.JSONDecodeError("Expecting value", "invalid json {", 0)
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        # Should handle JSON decode error gracefully
        assert config is None
        assert provider.state == ConfigState.ERROR
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_fetch_config_missing_fields(self, mock_get, provider):
        """Test config fetch with missing required fields (should fail validation)"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {"broker": "test.com"}  # Missing mqtt_user (required)
        mock_get.return_value = mock_response
        
        config = provider.fetch_config()
        
        # Validation should fail because mqtt_user is required
        assert config is None
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_get_config_returns_cached(self, mock_get, provider, valid_config_response):
        """Test that get_config returns cached configuration"""
        # First fetch to populate cache
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = valid_config_response
        mock_get.return_value = mock_response
        provider.fetch_config()
        
        # Get cached config
        cached_config = provider.get_config()
        
        assert cached_config is not None
        assert cached_config.broker == "mqtt.example.com"
    
    def test_get_config_returns_none_when_no_cache(self, provider):
        """Test that get_config returns None when no config cached"""
        config = provider.get_config()
        assert config is None
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_config_change_detection(self, mock_get, provider, valid_config_response, tls_config_response):
        """Test that config changes are detected"""
        callback_called = []
        callback_config = []
        
        def change_callback(config):
            callback_called.append(True)
            callback_config.append(config)
        
        provider.watch(change_callback)
        
        # First fetch
        mock_response1 = MagicMock()
        mock_response1.status_code = 200
        mock_response1.json.return_value = valid_config_response
        mock_get.return_value = mock_response1
        provider.fetch_config()
        
        # Callback should not be called for initial fetch
        assert len(callback_called) == 0
        
        # Second fetch with different config
        mock_response2 = MagicMock()
        mock_response2.status_code = 200
        mock_response2.json.return_value = tls_config_response
        mock_get.return_value = mock_response2
        provider.fetch_config()
        
        # Callback should be called
        assert len(callback_called) == 1
        assert callback_config[0].mqtt_use_tls is True
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_config_no_change_no_callback(self, mock_get, provider, valid_config_response):
        """Test that callback is not called when config doesn't change"""
        callback_called = []
        
        def change_callback(config):
            callback_called.append(True)
        
        provider.watch(change_callback)
        
        # First fetch
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = valid_config_response
        mock_get.return_value = mock_response
        provider.fetch_config()
        
        # Second fetch with same config
        mock_get.return_value = mock_response
        provider.fetch_config()
        
        # Callback should not be called
        assert len(callback_called) == 0
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_multiple_callbacks(self, mock_get, provider, valid_config_response, tls_config_response):
        """Test that multiple callbacks are called on config change"""
        callback1_called = []
        callback2_called = []
        
        def callback1(config):
            callback1_called.append(True)
        
        def callback2(config):
            callback2_called.append(True)
        
        provider.watch(callback1)
        provider.watch(callback2)
        
        # Initial fetch
        mock_response1 = MagicMock()
        mock_response1.status_code = 200
        mock_response1.json.return_value = valid_config_response
        mock_get.return_value = mock_response1
        provider.fetch_config()
        
        # Change config
        mock_response2 = MagicMock()
        mock_response2.status_code = 200
        mock_response2.json.return_value = tls_config_response
        mock_get.return_value = mock_response2
        provider.fetch_config()
        
        # Both callbacks should be called
        assert len(callback1_called) == 1
        assert len(callback2_called) == 1
    
    def test_polling_starts_and_stops(self, provider):
        """Test that polling can be started and stopped"""
        assert provider._polling_active is False
        
        provider.start_polling(interval=0.1)
        
        # Give it a moment to start
        time.sleep(0.05)
        assert provider._polling_active is True
        
        provider.stop_polling()
        
        # Give it a moment to stop
        time.sleep(0.1)
        assert provider._polling_active is False
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_polling_interval(self, mock_get, provider, valid_config_response):
        """Test that polling respects interval"""
        fetch_times = []
        
        original_fetch = provider.fetch_config
        
        def tracked_fetch():
            fetch_times.append(time.time())
            return original_fetch()
        
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = valid_config_response
        mock_get.return_value = mock_response
        
        provider.fetch_config = tracked_fetch
        
        provider.start_polling(interval=0.2)
        time.sleep(0.5)  # Should trigger ~2-3 fetches
        provider.stop_polling()
        
        # Should have at least 2 fetches
        assert len(fetch_times) >= 2
        
        # Check interval (with some tolerance)
        if len(fetch_times) >= 2:
            interval = fetch_times[1] - fetch_times[0]
            assert 0.15 <= interval <= 0.3  # Allow some tolerance
    
    def test_broker_config_equality(self):
        """Test BrokerConfig equality comparison"""
        config1 = BrokerConfig(
            broker="test.com",
            broker_port=1883,
            mqtt_user="user",
            mqtt_password="pass",
            mqtt_use_tls=False,
            mqtt_tls_insecure=False
        )
        
        config2 = BrokerConfig(
            broker="test.com",
            broker_port=1883,
            mqtt_user="user",
            mqtt_password="pass",
            mqtt_use_tls=False,
            mqtt_tls_insecure=False
        )
        
        assert config1 == config2
    
    def test_broker_config_inequality(self):
        """Test BrokerConfig inequality"""
        config1 = BrokerConfig(
            broker="test.com",
            broker_port=1883,
            mqtt_user="user",
            mqtt_password="pass",
            mqtt_use_tls=False,
            mqtt_tls_insecure=False
        )
        
        config2 = BrokerConfig(
            broker="different.com",
            broker_port=1883,
            mqtt_user="user",
            mqtt_password="pass",
            mqtt_use_tls=False,
            mqtt_tls_insecure=False
        )
        
        assert config1 != config2
    
    @patch('aehub_navigation.broker_config_provider.requests.get')
    def test_api_key_in_headers(self, mock_get, provider, valid_config_response):
        """Test that API key is included in request headers"""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = valid_config_response
        mock_get.return_value = mock_response
        
        provider.fetch_config()
        
        # Verify API key is in headers
        assert mock_get.called
        call_kwargs = mock_get.call_args[1]  # Keyword arguments
        assert 'headers' in call_kwargs
        assert 'X-API-Key' in call_kwargs['headers']
        assert call_kwargs['headers']['X-API-Key'] == provider.api_key


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

