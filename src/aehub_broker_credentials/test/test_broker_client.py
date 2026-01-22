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
Unit tests for broker_client module.

Tests HTTP client functionality without ROS dependencies.
"""

from unittest.mock import Mock, patch

from aehub_broker_credentials.broker_client import BrokerConfigClient
import pytest
import requests


class TestBrokerConfigClient:
    """Unit tests for BrokerConfigClient."""

    def test_init(self):
        """Test client initialization."""
        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )

        assert client._url == 'https://example.com/api/v1/config/broker?include_password=true'
        assert client._headers['Accept'] == 'application/json'
        assert client._headers['X-API-Key'] == 'test-key'
        assert client._timeout == 2.0

    def test_init_url_normalization(self):
        """Test URL normalization (trailing slash removal)."""
        client = BrokerConfigClient(
            base_url='https://example.com/',
            api_key='test-key',
            timeout=2.0,
        )

        assert client._url == 'https://example.com/api/v1/config/broker?include_password=true'

    @patch('aehub_broker_credentials.broker_client.requests.get')
    def test_fetch_success(self, mock_get):
        """Test successful fetch."""
        # Setup mock response
        mock_response = Mock()
        mock_response.json.return_value = {
            'broker': 'test.broker.com',
            'broker_port': 8883,
            'mqtt_user': 'test_user',
            'mqtt_password': 'test_password',
            'mqtt_use_tls': True,
            'mqtt_tls_insecure': False,
        }
        mock_response.raise_for_status = Mock()
        mock_get.return_value = mock_response

        # Test fetch
        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )
        result = client.fetch()

        # Verify
        assert result['broker'] == 'test.broker.com'
        assert result['broker_port'] == 8883
        assert result['mqtt_user'] == 'test_user'
        assert result['mqtt_password'] == 'test_password'
        assert result['mqtt_use_tls'] is True
        assert result['mqtt_tls_insecure'] is False

        # Verify request was made correctly
        mock_get.assert_called_once()
        call_args = mock_get.call_args
        assert call_args[0][0] == 'https://example.com/api/v1/config/broker?include_password=true'
        assert call_args[1]['headers']['Accept'] == 'application/json'
        assert call_args[1]['headers']['X-API-Key'] == 'test-key'
        assert call_args[1]['timeout'] == 2.0
        mock_response.raise_for_status.assert_called_once()

    @patch('aehub_broker_credentials.broker_client.requests.get')
    def test_fetch_timeout(self, mock_get):
        """Test timeout handling."""
        mock_get.side_effect = requests.exceptions.Timeout('Request timeout')

        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )

        with pytest.raises(requests.exceptions.Timeout):
            client.fetch()

    @patch('aehub_broker_credentials.broker_client.requests.get')
    def test_fetch_http_error(self, mock_get):
        """Test HTTP error handling."""
        # Setup mock response with HTTP error
        mock_response = Mock()
        mock_response.raise_for_status.side_effect = requests.exceptions.HTTPError('404 Not Found')
        mock_get.return_value = mock_response

        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )

        with pytest.raises(requests.exceptions.HTTPError):
            client.fetch()

    @patch('aehub_broker_credentials.broker_client.requests.get')
    def test_fetch_connection_error(self, mock_get):
        """Test connection error handling."""
        mock_get.side_effect = requests.exceptions.ConnectionError('Connection failed')

        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )

        with pytest.raises(requests.exceptions.ConnectionError):
            client.fetch()

    @patch('aehub_broker_credentials.broker_client.requests.get')
    def test_fetch_includes_password_param(self, mock_get):
        """Test that URL includes include_password=true parameter."""
        mock_response = Mock()
        mock_response.json.return_value = {'broker': 'test.com', 'broker_port': 1883}
        mock_response.raise_for_status = Mock()
        mock_get.return_value = mock_response

        client = BrokerConfigClient(
            base_url='https://example.com',
            api_key='test-key',
            timeout=2.0,
        )
        client.fetch()

        # Verify URL contains include_password=true
        call_args = mock_get.call_args
        assert 'include_password=true' in call_args[0][0]
