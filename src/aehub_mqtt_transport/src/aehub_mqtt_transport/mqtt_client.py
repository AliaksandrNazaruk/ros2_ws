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
Thin wrapper over paho-mqtt client.

Isolates MQTT client creation and lifecycle from transport node.
"""

from typing import Optional, Callable, Dict
import paho.mqtt.client as mqtt

from aehub_msgs.msg import BrokerConfig


class MqttClient:
    """
    Thin wrapper over paho-mqtt Client.
    
    Encapsulates:
    - Client creation and configuration
    - Connection lifecycle
    - TLS setup
    """
    
    def __init__(self, client_id: str):
        """
        Initialize MQTT client wrapper.
        
        Args:
            client_id: MQTT client ID
        """
        self._client: Optional[mqtt.Client] = None
        self._client_id = client_id
        self._connected = False
        self._topic_callbacks: Dict[str, Callable] = {}
        self._original_on_message: Optional[Callable] = None
    
    def configure(self, config: BrokerConfig, ca_cert_path: Optional[str] = None):
        """
        Configure client with broker settings.
        
        Args:
            config: Broker configuration
            ca_cert_path: Optional path to CA cert for TLS verification
        """
        if self._client:
            self.disconnect()
        
        self._client = mqtt.Client(
            client_id=self._client_id,
            clean_session=True,
        )
        
        # Set credentials
        if config.mqtt_user:
            self._client.username_pw_set(
                config.mqtt_user,
                config.mqtt_password
            )
        
        # Configure TLS
        if config.mqtt_use_tls:
            if ca_cert_path:
                self._client.tls_set(ca_certs=ca_cert_path)
            else:
                self._client.tls_set()
            if config.mqtt_tls_insecure:
                self._client.tls_insecure_set(True)
    
    def set_callbacks(
        self,
        on_connect: Optional[Callable] = None,
        on_disconnect: Optional[Callable] = None,
        on_message: Optional[Callable] = None,
    ):
        """
        Set MQTT event callbacks.
        
        Args:
            on_connect: Callback for connection events
            on_disconnect: Callback for disconnection events
            on_message: Callback for incoming messages
        """
        if not self._client:
            raise RuntimeError("Client not configured")
        
        if on_connect:
            self._client.on_connect = on_connect
        if on_disconnect:
            self._client.on_disconnect = on_disconnect
        if on_message:
            self._client.on_message = on_message
    
    def connect(self, host: str, port: int, keepalive: int = 30):
        """
        Connect to MQTT broker.
        
        Args:
            host: Broker hostname
            port: Broker port
            keepalive: Keepalive interval in seconds
        """
        if not self._client:
            raise RuntimeError("Client not configured")
        
        self._client.connect(host, port, keepalive=keepalive)
        self._client.loop_start()
    
    def disconnect(self):
        """Disconnect from MQTT broker."""
        if self._client:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass
            self._connected = False
    
    def subscribe(self, topic: str, qos: int = 1, callback: Optional[Callable] = None):
        """
        Subscribe to MQTT topic.
        
        Args:
            topic: MQTT topic pattern
            qos: QoS level (0, 1, or 2)
            callback: Optional callback function(topic: str, payload: bytes)
                     If None, uses default on_message callback
        """
        if not self._client:
            raise RuntimeError("Client not connected")
        
        if callback:
            # Store callback for this specific topic
            # We'll route messages through on_message and call the right callback
            if not hasattr(self, '_topic_callbacks'):
                self._topic_callbacks = {}
            
            # Wrap callback to match paho signature (client, userdata, msg)
            def wrapped_callback(client, userdata, msg):
                if msg.topic == topic or self._topic_matches(topic, msg.topic):
                    callback(msg.topic, msg.payload)
            
            self._topic_callbacks[topic] = wrapped_callback
            
            # If this is the first topic-specific callback, set up message router
            if not hasattr(self, '_original_on_message') or self._original_on_message is None:
                self._original_on_message = self._client.on_message
                
                def message_router(client, userdata, msg):
                    # Try topic-specific callbacks first
                    matched = False
                    for cb_topic, cb in getattr(self, '_topic_callbacks', {}).items():
                        if self._topic_matches(cb_topic, msg.topic):
                            cb(client, userdata, msg)
                            matched = True
                    
                    # Fall back to original callback if no match
                    if not matched and self._original_on_message:
                        self._original_on_message(client, userdata, msg)
                
                self._client.on_message = message_router
        
        self._client.subscribe(topic, qos=qos)
    
    @staticmethod
    def _topic_matches(pattern: str, topic: str) -> bool:
        """
        Check if topic matches pattern (supports wildcards).
        
        Args:
            pattern: MQTT topic pattern (may contain # or +)
            topic: Actual topic name
            
        Returns:
            True if topic matches pattern
        """
        # Simple wildcard matching
        if pattern == topic:
            return True
        
        # Handle # wildcard (multi-level)
        if pattern.endswith('/#'):
            prefix = pattern[:-2]
            if topic.startswith(prefix):
                return True
        
        # Handle + wildcard (single-level)
        pattern_parts = pattern.split('/')
        topic_parts = topic.split('/')
        
        if len(pattern_parts) != len(topic_parts):
            return False
        
        for p, t in zip(pattern_parts, topic_parts):
            if p != '+' and p != t:
                return False
        
        return True
    
    def unsubscribe(self, topic: str):
        """
        Unsubscribe from MQTT topic.
        
        Args:
            topic: MQTT topic to unsubscribe from
        """
        if not self._client:
            raise RuntimeError("Client not connected")
        
        # Remove topic-specific callback if exists
        if hasattr(self, '_topic_callbacks') and topic in self._topic_callbacks:
            del self._topic_callbacks[topic]
        
        self._client.unsubscribe(topic)
    
    def publish(self, topic: str, payload: str, qos: int = 1, retain: bool = False):
        """
        Publish message to MQTT topic.
        
        Args:
            topic: MQTT topic
            payload: Message payload (string)
            qos: QoS level (0, 1, or 2)
            retain: Retain flag
        """
        if not self._client:
            raise RuntimeError("Client not connected")
        self._client.publish(topic, payload, qos=qos, retain=retain)
    
    def is_connected(self) -> bool:
        """
        Check if client is connected.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected and self._client is not None
    
    def set_connected(self, value: bool):
        """Set connected state (used by callbacks)."""
        self._connected = value
    
    def get_client(self) -> Optional[mqtt.Client]:
        """
        Get underlying paho client (for advanced usage).
        
        Returns:
            paho.mqtt.client.Client or None
        """
        return self._client
