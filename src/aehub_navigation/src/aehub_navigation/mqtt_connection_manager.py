#!/usr/bin/env python3

"""
MQTT Connection Manager

Manages MQTT connection lifecycle with dynamic reconnection support.
Handles TLS configuration, subscriptions, and safe disconnect/reconnect.

AE.HUB MVP requirement: Dynamic reconnection without ROS2 node restart.
"""

import paho.mqtt.client as mqtt
import ssl
import threading
import time
import requests
import tempfile
import os
import atexit
from typing import Optional, List, Callable, Tuple, Dict, Any
from aehub_navigation.broker_config_provider import BrokerConfig


class MQTTConnectionManager:
    """
    Manages MQTT connection with support for dynamic reconnection.
    
    Handles:
    - Initial connection
    - Reconnection on config change
    - TLS setup
    - Subscription management
    - Safe disconnect
    
    Thread Safety:
    - Uses a single internal lock (_lock) to protect connection state
    - Lock is acquired with timeout to prevent deadlocks in callbacks
    - Non-blocking lock attempts are used in callbacks to avoid blocking MQTT client thread
    """
    
    def __init__(self, logger=None, config_service_url: Optional[str] = None, api_key: Optional[str] = None,
                 *, connect_timeout_s: float = 10.0, ca_cache_ttl_s: float = 300.0,
                 client_id_prefix: str = "nav_integrated", fail_closed_tls: bool = True,
                 reconnect_backoff_enabled: bool = True, reconnect_backoff_initial_s: float = 1.0,
                 reconnect_backoff_max_s: float = 60.0, reconnect_backoff_multiplier: float = 2.0,
                 reconnect_max_attempts: int = 10):
        """
        Initialize connection manager.
        
        Args:
            logger: ROS2 logger (optional)
            config_service_url: URL of Config Service for fetching CA certificate (optional)
            api_key: API key for Config Service authentication (optional)
            connect_timeout_s: Max time to wait for connection (seconds)
            ca_cache_ttl_s: TTL for cached CA certificate file (seconds)
            client_id_prefix: Prefix for MQTT client IDs
            fail_closed_tls: When True, require valid CA for TLS (no silent insecure fallback)
            reconnect_backoff_enabled: Enable exponential backoff for reconnection attempts
            reconnect_backoff_initial_s: Initial backoff delay in seconds
            reconnect_backoff_max_s: Maximum backoff delay in seconds
            reconnect_backoff_multiplier: Backoff multiplier (e.g., 2.0 for exponential)
            reconnect_max_attempts: Maximum reconnection attempts (0 = unlimited)
        """
        self.logger = logger
        self.config_service_url = config_service_url.rstrip('/') if config_service_url else None
        self.api_key = api_key
        self.client: Optional[mqtt.Client] = None
        self.current_config: Optional[BrokerConfig] = None
        self.subscriptions: List[Tuple[str, int]] = []  # List of (topic, qos) tuples
        self.is_connected = False
        
        # Thread safety: Single lock protects all connection state
        # Lock timeout is used in callbacks to prevent deadlocks
        # Default timeout: 1.0s for callbacks, 0.1s for subscribe operations
        self._lock = threading.Lock()
        self._lock_timeout_callback = 1.0  # Timeout for lock acquisition in callbacks
        self._lock_timeout_subscribe = 0.1  # Timeout for lock acquisition in subscribe
        
        self._reconnecting = False
        self._connected_event = threading.Event()
        self._stop_event = threading.Event()
        self._state: str = "DISCONNECTED"
        self._intentional_disconnect: bool = False
        self._connect_timeout_s = float(connect_timeout_s)
        self._ca_cache_ttl_s = float(ca_cache_ttl_s)
        self._client_id_prefix = str(client_id_prefix)
        self._fail_closed_tls = bool(fail_closed_tls)
        self._last_rc: Optional[int] = None
        self._last_error: Optional[str] = None
        
        # Reconnect backoff configuration
        self._reconnect_backoff_enabled = bool(reconnect_backoff_enabled)
        self._reconnect_backoff_initial_s = float(reconnect_backoff_initial_s)
        self._reconnect_backoff_max_s = float(reconnect_backoff_max_s)
        self._reconnect_backoff_multiplier = float(reconnect_backoff_multiplier)
        self._reconnect_max_attempts = int(reconnect_max_attempts)
        self._reconnect_attempt_count = 0
        self._reconnect_current_delay_s = self._reconnect_backoff_initial_s
        
        # CA certificate cache
        self._ca_cert_path: Optional[str] = None
        self._ca_cert_temp_file: Optional[tempfile.NamedTemporaryFile] = None
        self._ca_cached_at: float = 0.0
        
        # Register cleanup function to run on exit
        atexit.register(self._cleanup_ca_cert)
        
        # Callbacks
        self.on_connect_callback: Optional[Callable] = None
        self.on_message_callback: Optional[Callable] = None
        self.on_disconnect_callback: Optional[Callable] = None
    
    def _cleanup_ca_cert(self):
        """Clean up CA certificate temp file (called on exit)"""
        if self._ca_cert_path and os.path.exists(self._ca_cert_path):
            try:
                os.unlink(self._ca_cert_path)
            except Exception:
                pass  # Ignore errors during cleanup
        if self._ca_cert_temp_file:
            try:
                self._ca_cert_temp_file.close()
            except Exception:
                pass
        self._ca_cert_temp_file = None
        self._ca_cert_path = None
        self._ca_cached_at = 0.0
    
    def _log(self, level: str, message: str) -> None:
        """
        Log message using ROS2 logger or print.
        
        Args:
            level: Log level ('info', 'warn', 'error', 'debug')
            message: Message to log
        """
        if self.logger:
            if level == "info":
                self.logger.info(message)
            elif level == "warn":
                self.logger.warn(message)
            elif level == "error":
                self.logger.error(message)
            elif level == "debug":
                self.logger.debug(message)
        else:
            print(f"[{level.upper()}] {message}")
    
    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: Dict[str, int], rc: int) -> None:
        """
        MQTT on_connect callback.
        
        Args:
            client: MQTT client instance
            userdata: User data (unused)
            flags: Connection flags dictionary
            rc: Return code (0 = success, non-zero = error)
        """
        # Log connection attempt details
        broker = self.current_config.broker if self.current_config else 'unknown'
        port = self.current_config.broker_port if self.current_config else 1883
        if self.current_config and self.current_config.mqtt_use_tls and port == 1883:
            port = 8883
        
        self._log("info", f"MQTT on_connect callback called: rc={rc}, broker={broker}:{port}, flags={flags}")
        # Some MQTT client implementations may pass flags as None or a non-dict.
        # Normalize flags to a dict to avoid attribute errors in tests and edge cases.
        flags_dict = flags if isinstance(flags, dict) else {}
        self._log("debug", f"Connection flags: session_present={flags_dict.get('session present', False)}, "
                           f"clean_session={not flags_dict.get('session present', False)}")
        self._log("info", f" Step 1: About to enter try block")
        
        try:
            self._log("info", f" Step 2: Inside try block, about to acquire lock")
            # Deadlock prevention: Use non-blocking lock first, then timeout-based lock
            # This prevents blocking the MQTT client callback thread indefinitely
            # Timeout ensures we don't wait forever if another thread holds the lock
            lock_acquired = self._lock.acquire(blocking=False)
            if not lock_acquired:
                self._log("warn", f"  Could not acquire lock immediately, trying with timeout ({self._lock_timeout_callback}s)...")
                lock_acquired = self._lock.acquire(blocking=True, timeout=self._lock_timeout_callback)
            
            if lock_acquired:
                try:
                    self._log("info", f" Step 2.1: Lock acquired, checking rc={rc}")
                    if rc == 0:
                        self._log("info", f" Step 2.2: rc==0, setting is_connected=True")
                        self.is_connected = True
                        self._log("info", f" Successfully connected to MQTT broker: {broker}:{port}")
                        self._log("debug", f"is_connected flag set to True")
                    else:
                        self.is_connected = False
                        # Map MQTT error codes to human-readable messages
                        error_messages = {
                            1: "Connection refused - incorrect protocol version",
                            2: "Connection refused - invalid client identifier",
                            3: "Connection refused - server unavailable",
                            4: "Connection refused - bad username or password",
                            5: "Connection refused - not authorized"
                        }
                        error_msg = error_messages.get(rc, f"Unknown error code: {rc}")
                        self._log("error", f" Failed to connect to MQTT broker: rc={rc} ({error_msg})")
                        self._log("error", f"   Broker: {broker}:{port}")
                        self._log("error", f"   TLS: {self.current_config.mqtt_use_tls if self.current_config else 'unknown'}")
                        self._log("error", f"   Username: {'set' if (self.current_config and self.current_config.mqtt_user) else 'not set'}")
                    self._log("info", f" Step 2.3: Exiting lock block")
                finally:
                    self._lock.release()
            else:
                self._log("error", f" Could not acquire lock after timeout ({self._lock_timeout_callback}s), setting is_connected without lock")
                # Fallback: Set without lock to avoid deadlock
                # This is a last resort - better to have slightly inconsistent state than deadlock
                # The state will be corrected on next successful lock acquisition
                self.is_connected = (rc == 0)
        except Exception as e:
            self._log("error", f" Exception in _on_connect lock block: {e}", exc_info=True)
            import traceback
            self._log("error", f" Traceback: {traceback.format_exc()}")
        
        # Callbacks and subscriptions outside lock to avoid deadlock
        self._log("info", f" Step 3: After try-except block, processing connection result: rc={rc}")
        if rc == 0:
            self._log("info", f" Processing successful connection (rc=0), subscriptions count: {len(self.subscriptions)}")
            # Re-subscribe to all topics
            self._log("debug", f"Re-subscribing to {len(self.subscriptions)} topics...")
            for topic, qos in self.subscriptions:
                try:
                    result = client.subscribe(topic, qos)
                    if result[0] == mqtt.MQTT_ERR_SUCCESS:
                        self._log("debug", f" Re-subscribed to: {topic} (QoS {qos})")
                    else:
                        self._log("error", f" Failed to re-subscribe to {topic}: error code {result[0]}")
                except Exception as e:
                    self._log("error", f" Exception re-subscribing to {topic}: {e}")
            
            self._log("info", f" Checking on_connect_callback: {'set' if self.on_connect_callback else 'NOT SET'}")
            if self.on_connect_callback:
                self._log("info", "📞 Calling on_connect_callback...")
                try:
                    self.on_connect_callback(client, userdata, flags, rc)
                    self._log("info", " on_connect_callback completed successfully")
                except Exception as e:
                    self._log("error", f" Error in on_connect callback: {e}", exc_info=True)
            else:
                self._log("warn", "  on_connect_callback is not set - subscriptions will not be created!")
        else:
            self._log("info", f" Connection failed (rc={rc}), skipping callback")
    
    def _on_disconnect(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        """
        MQTT on_disconnect callback.
        
        Args:
            client: MQTT client instance
            userdata: User data (unused)
            rc: Return code (0 = normal disconnect, non-zero = unexpected)
        """
        with self._lock:
            self.is_connected = False
            self._connected_event.clear()
            self._last_rc = rc
            self._state = "DISCONNECTED" if self._intentional_disconnect else "CONNECTING"
        
        if rc == 0:
            self._log("info", "Disconnected from MQTT broker")
        else:
            self._log("warn", f"Unexpected disconnect from MQTT broker: {rc}")
        
        if self.on_disconnect_callback:
            try:
                self.on_disconnect_callback(client, userdata, rc)
            except Exception as e:
                self._log("error", f"Error in on_disconnect callback: {e}")
    
    def _on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        """
        MQTT on_message callback.
        
        Args:
            client: MQTT client instance
            userdata: User data (unused)
            msg: MQTT message
        """
        if self.on_message_callback:
            try:
                self.on_message_callback(client, userdata, msg)
            except Exception as e:
                self._log("error", f"Error in on_message callback: {e}")
    
    def _fetch_ca_certificate(self) -> Optional[str]:
        """
        Fetch CA certificate from Config Service.
        
        Returns:
            Path to temporary file with CA certificate, or None if failed
        """
        if not self.config_service_url or not self.api_key:
            return None
        
        try:
            # Try both endpoint paths
            endpoints = [
                f"{self.config_service_url}/api/v1/config/certificates/ca",
                f"{self.config_service_url}/config/certificates/ca"
            ]
            
            for url in endpoints:
                try:
                    headers = {"X-API-Key": self.api_key} if self.api_key else {}
                    response = requests.get(url, headers=headers, timeout=5.0)
                    
                    if response.status_code == 200:
                        # Create temporary file for CA certificate
                        if self._ca_cert_temp_file:
                            self._ca_cert_temp_file.close()
                        
                        self._ca_cert_temp_file = tempfile.NamedTemporaryFile(
                            mode='w', suffix='.crt', delete=False
                        )
                        self._ca_cert_temp_file.write(response.text)
                        self._ca_cert_temp_file.flush()
                        self._ca_cert_path = self._ca_cert_temp_file.name
                        self._log("info", f"CA certificate fetched from Config Service: {self._ca_cert_path}")
                        self._ca_cached_at = time.monotonic()
                        return self._ca_cert_path
                    elif response.status_code == 404:
                        # Try next endpoint
                        continue
                    else:
                        self._log("warn", f"Failed to fetch CA certificate: HTTP {response.status_code}")
                        return None
                except requests.exceptions.RequestException as e:
                    self._log("debug", f"Error fetching CA certificate from {url}: {e}")
                    continue
            
            return None
        except Exception as e:
            self._log("error", f"Unexpected error fetching CA certificate: {e}")
            return None
    
    def connect(self, config: BrokerConfig, *, wait: bool = True) -> bool:
        """
        Connect to MQTT broker using provided configuration.
        
        Args:
            config: Broker configuration
            wait: Wait for connection event (default True)
        Returns:
            True if connection initiated successfully, False otherwise
        """
        with self._lock:
            # If already connected to same broker, skip
            if self.is_connected and self.current_config == config:
                self._log("debug", "Already connected to same broker, skipping")
                return True
            self._state = "CONNECTING"
            self._connected_event.clear()
            self._last_error = None
            self._last_rc = None
            # Disconnect existing connection if any
            old_client = self.client
            self.client = None
            self.is_connected = False
            self.current_config = config
        if old_client is not None:
            try:
                old_client.loop_stop()
            except Exception:
                pass
            try:
                old_client.disconnect()
            except Exception:
                pass
        # Create new client
        client_id = f"{self._client_id_prefix}_{int(time.time())}"
        self.client = mqtt.Client(client_id=client_id)
        # Credentials
        if config.mqtt_user:
            self.client.username_pw_set(config.mqtt_user, config.mqtt_password)
        # TLS
        if config.mqtt_use_tls:
            if config.mqtt_tls_insecure:
                self.client.tls_set(cert_reqs=ssl.CERT_NONE)
                self.client.tls_insecure_set(True)
                self._log("warn", "Using insecure TLS (for demo/testing only)")
            else:
                # Use cached CA when valid
                if not self._ca_cert_path or (time.monotonic() - self._ca_cached_at) > self._ca_cache_ttl_s or not os.path.exists(self._ca_cert_path):
                    self._fetch_ca_certificate()
                if not self._ca_cert_path or not os.path.exists(self._ca_cert_path):
                    msg = "TLS required but CA certificate not available"
                    if self._fail_closed_tls:
                        self._log("error", msg)
                        return False
                    self._log("warn", f"{msg}; falling back to insecure TLS")
                    self.client.tls_set(cert_reqs=ssl.CERT_NONE)
                    self.client.tls_insecure_set(True)
                else:
                    try:
                        self.client.tls_set(ca_certs=self._ca_cert_path, cert_reqs=ssl.CERT_REQUIRED)
                        self.client.tls_insecure_set(False)
                        self._log("info", f"Using secure TLS with CA certificate: {self._ca_cert_path}")
                    except Exception as e:
                        if self._fail_closed_tls:
                            self._log("error", f"Failed to configure TLS with CA certificate: {e}")
                            return False
                        self._log("warn", "TLS configure failed, falling back to insecure TLS")
                        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
                        self.client.tls_insecure_set(True)
        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        # Connect asynchronously
        try:
            port = config.broker_port
            if config.mqtt_use_tls and port == 1883:
                port = 8883
            self._log("info", f"Connecting to MQTT broker: {config.broker}:{port} (TLS: {config.mqtt_use_tls})")
            self.client.connect_async(config.broker, port, 60)
            self.client.loop_start()
        except Exception as e:
            self._log("error", f" Exception during connect_async/loop_start: {e}")
            return False
        if not wait:
            return True
        ok = self._connected_event.wait(timeout=self._connect_timeout_s)
        if ok:
            self._log("info", " Connection established")
            return True
        self._log("error", f" Connection timeout after {self._connect_timeout_s:.1f}s")
        return False
    
    def disconnect(self):
        """Disconnect from MQTT broker."""
        with self._lock:
            self._intentional_disconnect = True
            client = self.client
        if client is not None:
            try:
                self._log("info", "Disconnecting from MQTT broker...")
                client.disconnect()
            except Exception as e:
                self._log("warn", f"Error disconnecting: {e}")
            try:
                client.loop_stop()
            except Exception:
                pass
        with self._lock:
            self.client = None
            self.is_connected = False
            self.current_config = None
            self._state = "DISCONNECTED"
            self._intentional_disconnect = False
            # Clean up CA certificate temp file
            self._cleanup_ca_cert()
    
    def subscribe(self, topic: str, qos: int = 1) -> None:
        """
        Subscribe to topic.
        
        Args:
            topic: MQTT topic
            qos: Quality of Service level (0, 1, or 2)
        """
        # Store subscription for reconnection
        if (topic, qos) not in self.subscriptions:
            self.subscriptions.append((topic, qos))
        
        # Subscribe if connected (use timeout-based lock to avoid deadlock)
        # Deadlock prevention: Short timeout prevents blocking if lock is held by callback
        lock_acquired = False
        try:
            lock_acquired = self._lock.acquire(blocking=False)
            if not lock_acquired:
                # Try with short timeout (subscribe operations should be fast)
                lock_acquired = self._lock.acquire(blocking=True, timeout=self._lock_timeout_subscribe)
            
            if lock_acquired:
                is_connected = self.is_connected
                client = self.client
            else:
                # Fallback: read without lock to avoid deadlock
                # This is acceptable for read-only operations during subscribe
                self._log("warn", f"  Could not acquire lock in subscribe() after {self._lock_timeout_subscribe}s timeout, using fallback read")
                is_connected = self.is_connected
                client = self.client
        finally:
            if lock_acquired:
                self._lock.release()
        
        if client and is_connected:
            try:
                result = client.subscribe(topic, qos)
                self._log("info", f" Subscribed to: {topic} (QoS {qos}), result: {result}")
            except Exception as e:
                self._log("error", f" Error subscribing to {topic}: {e}")
                import traceback
                self._log("error", f" Traceback: {traceback.format_exc()}")
        else:
            self._log("warn", f"  Cannot subscribe to {topic}: client={client is not None}, is_connected={is_connected}")
    
    def publish(self, topic: str, payload: str, qos: int = 1) -> bool:
        """
        Publish message to topic.
        
        Args:
            topic: MQTT topic
            payload: Message payload
            qos: Quality of Service level
            
        Returns:
            True if published successfully, False otherwise
        """
        with self._lock:
            client = self.client
            is_conn = self.is_connected
        if not is_conn or client is None:
            self._log("warn", f"Cannot publish: not connected to broker")
            return False
        try:
            info = client.publish(topic, payload, qos)
            # For QoS>0 wait briefly for completion
            try:
                if qos > 0 and hasattr(info, "wait_for_publish"):
                    info.wait_for_publish(timeout=2.0)
            except Exception:
                pass
            if getattr(info, "rc", mqtt.MQTT_ERR_SUCCESS) == mqtt.MQTT_ERR_SUCCESS:
                return True
            self._log("error", f"Failed to publish: {getattr(info,'rc',None)}")
            return False
        except Exception as e:
            self._log("error", f"Error publishing: {e}")
            return False
    
    def reconnect(self, config: BrokerConfig) -> bool:
        """
        Reconnect to new broker configuration with exponential backoff.
        
        Implements exponential backoff to avoid overwhelming the broker
        during network issues or broker unavailability.
        
        Args:
            config: New broker configuration
            
        Returns:
            True if reconnected successfully, False otherwise
        """
        self._log("info", "Reconnecting to new broker configuration...")
        
        # Reset backoff on successful config change (new broker)
        if self.current_config != config:
            self._reconnect_attempt_count = 0
            self._reconnect_current_delay_s = self._reconnect_backoff_initial_s
            self._log("debug", f"New broker config detected, resetting backoff: {config.broker}:{config.broker_port}")
        
        attempt = 0
        while True:
            attempt += 1
            self._reconnect_attempt_count = attempt
            
            # Check max attempts limit
            if self._reconnect_max_attempts > 0 and attempt > self._reconnect_max_attempts:
                self._log("error", f" Reconnection failed after {attempt-1} attempts (max: {self._reconnect_max_attempts})")
                return False
            
            # Attempt connection
            self._log("info", f"Reconnection attempt {attempt}/{self._reconnect_max_attempts if self._reconnect_max_attempts > 0 else '∞'}")
            success = self.connect(config)
            
            if success:
                # Reset backoff on success
                self._reconnect_attempt_count = 0
                self._reconnect_current_delay_s = self._reconnect_backoff_initial_s
                self._log("info", f" Successfully reconnected after {attempt} attempt(s)")
                return True
            
            # If backoff disabled, return immediately on failure
            if not self._reconnect_backoff_enabled:
                self._log("warn", f"Reconnection failed (backoff disabled)")
                return False
            
            # Calculate next backoff delay
            delay = min(self._reconnect_current_delay_s, self._reconnect_backoff_max_s)
            self._log("warn", f"Reconnection attempt {attempt} failed, waiting {delay:.1f}s before retry...")
            
            # Wait with backoff (check stop event to allow graceful shutdown)
            if self._stop_event.wait(timeout=delay):
                self._log("info", "Reconnection cancelled (stop event)")
                return False
            
            # Increase delay for next attempt
            self._reconnect_current_delay_s = min(
                self._reconnect_current_delay_s * self._reconnect_backoff_multiplier,
                self._reconnect_backoff_max_s
            )
    
    def get_connection_status(self) -> dict:
        """
        Get detailed connection status for diagnostics.
        
        Returns:
            Dictionary with connection status information
        """
        with self._lock:
            status = {
                "is_connected": self.is_connected,
                "has_client": self.client is not None,
                "broker": self.current_config.broker if self.current_config else None,
                "port": self.current_config.broker_port if self.current_config else None,
                "tls_enabled": self.current_config.mqtt_use_tls if self.current_config else None,
                "subscriptions_count": len(self.subscriptions),
                "subscriptions": [{"topic": t, "qos": q} for t, q in self.subscriptions]
            }
            
            if self.client is not None:
                try:
                    status["client_state"] = self.client._state if hasattr(self.client, '_state') else 'unknown'
                    status["client_id"] = self.client._client_id if hasattr(self.client, '_client_id') else 'unknown'
                except:
                    status["client_state"] = "error_getting_state"
            
            return status

