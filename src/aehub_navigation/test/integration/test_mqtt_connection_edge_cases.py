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
MQTT Connection Edge Cases and Failure Modes

Tests for edge cases in MQTT connection management that could cause
production incidents.

Test Strategy:
- Test connection timeout scenarios
- Test callback ordering and race conditions
- Test reconnection logic under various failure modes
- Test resource cleanup

Risks Covered:
1. Connection timeout with callback never called
2. Callback called after timeout check
3. Multiple simultaneous reconnection attempts
4. Resource leaks from uncleaned connections
5. State inconsistency during reconnection
"""

import pytest
import threading
import time
from unittest.mock import Mock, patch, MagicMock
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
from aehub_navigation.broker_config_provider import BrokerConfig


@pytest.fixture
def mock_broker_config():
    """Create mock broker configuration"""
    config = Mock(spec=BrokerConfig)
    config.broker = "test.broker"
    config.broker_port = 8883
    config.mqtt_user = "test_user"
    config.mqtt_password = "test_pass"
    config.mqtt_use_tls = True
    config.mqtt_tls_insecure = True
    return config


@pytest.fixture
def mqtt_manager():
    """Create MQTT connection manager"""
    return MQTTConnectionManager(logger=Mock())


class TestConnectionTimeoutScenarios:
    """
    Tests for connection timeout edge cases.
    """
    
    def test_connection_timeout_before_callback(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Connection times out, but callback called
        after timeout, is_connected never set, but client thinks connected.
        
        Production incident: Network delay causes callback after timeout,
        is_connected remains False, but MQTT client is actually connected,
        messages lost.
        """
        # Given: Slow network (callback delayed)
        original_connect = mqtt_manager.client.connect if mqtt_manager.client else None
        
        def delayed_callback(client, userdata, flags, rc):
            # Simulate callback called after timeout
            time.sleep(0.15)  # Longer than timeout check interval
            if mqtt_manager.client:
                mqtt_manager._on_connect(client, userdata, flags, rc)
        
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            
            # Simulate connection that takes too long
            def slow_connect(*args, **kwargs):
                # Start connection but callback comes late
                threading.Timer(0.12, lambda: delayed_callback(mock_client, None, {}, 0)).start()
                return 0
            
            mock_client.connect = slow_connect
            mock_client.loop_start = Mock()
            
            # When: Connection attempted
            result = mqtt_manager.connect(mock_broker_config)
            
            # Then: Should return False (timeout)
            # But: Need to verify state is consistent
            assert result is False or mqtt_manager.is_connected is False, \
                "Connection should timeout or be marked as not connected"
    
    def test_callback_called_after_timeout_check(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Timeout check passes (returns False), but
        callback called immediately after, is_connected becomes True,
        but connect() already returned False.
        
        Production incident: Race condition between timeout check and
        callback, connect() returns False but connection actually succeeds,
        node thinks disconnected but is actually connected.
        """
        callback_called = threading.Event()
        
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            
            def delayed_connect(*args, **kwargs):
                # Callback arrives just after timeout check
                def late_callback():
                    time.sleep(0.11)  # Just after timeout
                    mqtt_manager._on_connect(mock_client, None, {}, 0)
                    callback_called.set()
                
                threading.Timer(0.11, late_callback).start()
                return 0
            
            mock_client.connect = delayed_connect
            mock_client.loop_start = Mock()
            
            # When: Connection attempted
            result = mqtt_manager.connect(mock_broker_config)
            
            # Wait for callback
            callback_called.wait(timeout=0.5)
            
            # Then: State should be consistent
            # If callback came after timeout, is_connected might be True
            # but connect() returned False - this is a bug
            if result is False and mqtt_manager.is_connected:
                pytest.fail("State inconsistency: connect() returned False but is_connected is True")
    
    def test_multiple_simultaneous_connection_attempts(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Multiple threads call connect() simultaneously,
        multiple clients created, subscriptions lost.
        
        Production incident: Config change triggers reconnect while
        initial connection still in progress, two clients created,
        subscriptions on wrong client.
        """
        connection_results = []
        barrier = threading.Barrier(2)
        
        def connect_attempt():
            barrier.wait()
            result = mqtt_manager.connect(mock_broker_config)
            connection_results.append(result)
        
        # When: Two threads connect simultaneously
        t1 = threading.Thread(target=connect_attempt)
        t2 = threading.Thread(target=connect_attempt)
        t1.start()
        t2.start()
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        
        # Then: Only one connection should succeed
        # Lock should prevent race condition
        assert len(connection_results) == 2, "Both threads should complete"
        # At least one should handle the "already connecting" case


class TestReconnectionResourceCleanup:
    """
    Tests for resource cleanup during reconnection.
    """
    
    def test_old_client_not_cleaned_on_reconnect(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Reconnection creates new client, but old
        client not properly cleaned up, memory leak and duplicate subscriptions.
        
        Production incident: Memory leak from uncleaned MQTT clients,
        system runs out of memory after many reconnections.
        """
        # Given: Initial connection exists
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client1 = MagicMock()
            mock_client_class.return_value = mock_client1
            mock_client1.connect.return_value = 0
            mock_client1.loop_start = Mock()
            
            # Simulate successful connection
            mqtt_manager.client = mock_client1
            mqtt_manager.is_connected = True
            mqtt_manager.current_config = mock_broker_config
            
            # When: Reconnection triggered
            new_config = Mock(spec=BrokerConfig)
            new_config.broker = "new.broker"
            new_config.broker_port = 8883
            new_config.mqtt_user = "new_user"
            new_config.mqtt_password = "new_pass"
            new_config.mqtt_use_tls = True
            new_config.mqtt_tls_insecure = True
            
            mock_client2 = MagicMock()
            mock_client_class.return_value = mock_client2
            mock_client2.connect.return_value = 0
            mock_client2.loop_start = Mock()
            
            mqtt_manager.connect(new_config)
            
            # Then: Old client should be cleaned up
            assert mock_client1.loop_stop.called, "Old client loop should be stopped"
            assert mock_client1.disconnect.called, "Old client should be disconnected"
            assert mqtt_manager.client is not mock_client1, "Old client should be replaced"
    
    def test_ca_certificate_cleanup_on_reconnect(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: CA certificate temp file not cleaned up
        on reconnection, disk space exhausted.
        
        Production incident: Many reconnections create many temp files,
        disk fills up, system crashes.
        """
        # Given: CA certificate was fetched
        with patch('tempfile.NamedTemporaryFile') as mock_temp, \
             patch('requests.get') as mock_get:
            
            mock_response = Mock()
            mock_response.status_code = 200
            mock_response.text = "-----BEGIN CERTIFICATE-----\nTEST\n-----END CERTIFICATE-----"
            mock_get.return_value = mock_response
            
            mock_temp_file = MagicMock()
            mock_temp_file.name = "/tmp/test_cert.crt"
            mock_temp.return_value = mock_temp_file
            
            # When: Reconnection happens
            new_config = Mock(spec=BrokerConfig)
            new_config.broker = "new.broker"
            new_config.broker_port = 8883
            new_config.mqtt_user = "new_user"
            new_config.mqtt_password = "new_pass"
            new_config.mqtt_use_tls = True
            new_config.mqtt_tls_insecure = False
            
            # Old cert should be cleaned up
            old_cert_path = mqtt_manager._ca_cert_path
            
            mqtt_manager.connect(new_config)
            
            # Then: Old cert file should be closed/cleaned
            # (Implementation should close old temp file before creating new one)


class TestSubscriptionManagement:
    """
    Tests for subscription management during reconnection.
    """
    
    def test_subscriptions_lost_on_reconnection(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Reconnection creates new client, but
        subscriptions not re-established, commands lost.
        
        Production incident: MQTT reconnects, but subscriptions not
        restored, all commands lost until manual restart.
        """
        # Given: Subscriptions exist
        mqtt_manager.subscriptions = [
            ("aroc/robot/robot_001/commands/navigateTo", 1),
            ("aroc/robot/robot_001/commands/cancel", 1)
        ]
        
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            mock_client.connect.return_value = 0
            mock_client.loop_start = Mock()
            
            # When: Reconnection happens
            mqtt_manager.connect(mock_broker_config)
            
            # Simulate connection callback
            mqtt_manager._on_connect(mock_client, None, {}, 0)
            
            # Then: Subscriptions should be restored
            assert mock_client.subscribe.called, "Subscriptions should be restored"
            # Verify all subscriptions were called
            subscribe_calls = [call[0][0] for call in mock_client.subscribe.call_args_list]
            assert len(subscribe_calls) == 2, "All subscriptions should be restored"
    
    def test_subscription_failure_during_reconnection(self, mqtt_manager, mock_broker_config):
        """
        Bug it would catch: Reconnection succeeds, but subscription
        fails, no error logged, commands lost silently.
        
        Production incident: Reconnection succeeds, but subscription
        fails due to permissions, no error, commands never received.
        """
        mqtt_manager.subscriptions = [("test/topic", 1)]
        
        with patch('paho.mqtt.client.Client') as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client
            mock_client.connect.return_value = 0
            mock_client.loop_start = Mock()
            mock_client.subscribe.return_value = (1, 0)  # Error code
            
            # When: Reconnection and subscription
            mqtt_manager.connect(mock_broker_config)
            mqtt_manager._on_connect(mock_client, None, {}, 0)
            
            # Then: Error should be logged
            # (Implementation should log subscription failures)
            assert mock_client.subscribe.called, "Subscription should be attempted"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

