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
Broker Config Provider

Fetches MQTT broker configuration from centralized Broker Config Service.
Provides dynamic configuration updates without ROS2 node restart.

AE.HUB MVP requirement: No local MQTT configuration storage.
All configuration must come from Config Service (http://localhost:7900).
"""

import requests
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum
import threading
import time
from .circuit_breaker import CircuitBreaker, CircuitBreakerOpenError


class ConfigState(Enum):
    """Configuration state"""
    UNKNOWN = "unknown"
    FETCHING = "fetching"
    READY = "ready"
    ERROR = "error"


@dataclass
class BrokerConfig:
    """MQTT broker configuration"""
    broker: str
    broker_port: int
    mqtt_user: str
    mqtt_password: str
    mqtt_use_tls: bool
    mqtt_tls_insecure: bool
    
    def __eq__(self, other):
        """Compare configurations for changes"""
        if not isinstance(other, BrokerConfig):
            return False
        return (
            self.broker == other.broker and
            self.broker_port == other.broker_port and
            self.mqtt_user == other.mqtt_user and
            self.mqtt_password == other.mqtt_password and
            self.mqtt_use_tls == other.mqtt_use_tls and
            self.mqtt_tls_insecure == other.mqtt_tls_insecure
        )
    
    def __ne__(self, other):
        return not self.__eq__(other)


class BrokerConfigProvider:
    """
    Provider for MQTT broker configuration from centralized Config Service.
    
    Fetches configuration from HTTP API and supports polling for changes.
    """
    
    def __init__(self, config_service_url: str, api_key: str, logger=None):
        """
        Initialize provider.
        
        Args:
            config_service_url: URL of Config Service (e.g., "http://localhost:7900")
            api_key: API key for authentication
            logger: ROS2 logger (optional)
        """
        self.config_service_url = config_service_url.rstrip('/')
        self.api_key = api_key
        self.logger = logger
        
        self.current_config: Optional[BrokerConfig] = None
        self.state = ConfigState.UNKNOWN
        
        self._change_callbacks: list[Callable[[BrokerConfig], None]] = []
        self._polling_thread: Optional[threading.Thread] = None
        self._polling_interval = 5.0  # seconds
        self._polling_active = False
        self._lock = threading.Lock()
        
        # Circuit breaker for Config Service
        self._circuit_breaker = CircuitBreaker(
            failure_threshold=5,  # Open after 5 consecutive failures
            timeout=60.0,  # Try to close after 60 seconds
            expected_exception=(requests.exceptions.RequestException, Exception)
        )
    
    def _log(self, level: str, message: str):
        """Log message using ROS2 logger or print"""
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
    
    def _validate_and_normalize_config(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Validate and normalize configuration data from Config Service.
        
        Converts None to empty strings, validates types, and checks required fields.
        
        Args:
            data: Raw configuration data from Config Service
            
        Returns:
            Validated and normalized config dict, or None if validation fails
        """
        # Normalize broker (convert None to empty string, strip whitespace)
        broker = data.get("broker")
        if broker is None:
            broker = ""
        else:
            broker = str(broker).strip()
        
        # Validate broker (required, non-empty)
        if not broker:
            self._log("error", "Broker field is missing or empty")
            return None
        
        # Validate broker length (prevent DoS)
        if len(broker) > 255:
            self._log("error", f"Broker field too long: {len(broker)} characters (max 255)")
            return None
        
        # Normalize and validate port
        broker_port = data.get("broker_port", 1883)
        if broker_port is None:
            self._log("error", "Broker port is None")
            return None
        
        # Convert to int if string
        try:
            broker_port = int(broker_port)
        except (ValueError, TypeError):
            self._log("error", f"Invalid broker port type: {type(broker_port).__name__}")
            return None
        
        # Validate port range
        if broker_port <= 0 or broker_port > 65535:
            self._log("error", f"Invalid broker port: {broker_port} (must be 1-65535)")
            return None
        
        # Normalize mqtt_user (convert None to empty string)
        mqtt_user = data.get("mqtt_user")
        if mqtt_user is None:
            mqtt_user = ""
        else:
            mqtt_user = str(mqtt_user).strip()
        
        # Validate mqtt_user (required, non-empty)
        if not mqtt_user:
            self._log("error", "MQTT user field is missing or empty")
            return None
        
        # Validate mqtt_user length
        if len(mqtt_user) > 255:
            self._log("error", f"MQTT user field too long: {len(mqtt_user)} characters (max 255)")
            return None
        
        # Normalize mqtt_password (convert None to empty string)
        mqtt_password = data.get("mqtt_password")
        if mqtt_password is None:
            mqtt_password = ""
        else:
            mqtt_password = str(mqtt_password)
        
        # Validate mqtt_password (required, non-empty)
        if not mqtt_password:
            self._log("error", "MQTT password field is missing or empty")
            return None
        
        # Validate mqtt_password length
        if len(mqtt_password) > 1024:
            self._log("error", f"MQTT password field too long: {len(mqtt_password)} characters (max 1024)")
            return None
        
        # Normalize boolean fields
        mqtt_use_tls = bool(data.get("mqtt_use_tls", False))
        mqtt_tls_insecure = bool(data.get("mqtt_tls_insecure", False))
        
        return {
            "broker": broker,
            "broker_port": broker_port,
            "mqtt_user": mqtt_user,
            "mqtt_password": mqtt_password,
            "mqtt_use_tls": mqtt_use_tls,
            "mqtt_tls_insecure": mqtt_tls_insecure
        }
    
    def fetch_config(self) -> Optional[BrokerConfig]:
        """
        Fetch current configuration from Config Service with retry logic.
        
        Retries on temporary errors (timeout, connection error) with exponential backoff.
        Does not retry on permanent errors (400, 401, 403).
        
        Returns:
            BrokerConfig if successful, None otherwise
        """
        # Request password explicitly (required for MQTT connection)
        url = f"{self.config_service_url}/api/v1/config/broker?include_password=true"
        headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }
        
        # Check circuit breaker
        if self._circuit_breaker.is_open():
            self._log("warn", "Circuit breaker is OPEN - using cached config if available")
            cached_config = self.get_config()
            if cached_config:
                return cached_config
            else:
                self._log("error", "Circuit breaker OPEN and no cached config available")
                self.state = ConfigState.ERROR
                return None
        
        max_retries = 3
        retry_delays = [1.0, 2.0, 4.0]  # Exponential backoff: 1s, 2s, 4s
        
        for attempt in range(max_retries):
            try:
                self.state = ConfigState.FETCHING
                # Use circuit breaker to protect request
                response = self._circuit_breaker.call(
                    requests.get,
                    url,
                    headers=headers,
                    timeout=5.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    
                    # Validate and normalize data
                    validated_data = self._validate_and_normalize_config(data)
                    if validated_data is None:
                        # Validation failed - don't retry (permanent error)
                        self._log("error", "Config validation failed - invalid data from Config Service")
                        self.state = ConfigState.ERROR
                        return None
                    
                    # Parse response with validated data
                    config = BrokerConfig(
                        broker=validated_data["broker"],
                        broker_port=validated_data["broker_port"],
                        mqtt_user=validated_data["mqtt_user"],
                        mqtt_password=validated_data["mqtt_password"],
                        mqtt_use_tls=validated_data["mqtt_use_tls"],
                        mqtt_tls_insecure=validated_data["mqtt_tls_insecure"]
                    )
                    
                    # Check for changes
                    with self._lock:
                        old_config = self.current_config
                        self.current_config = config
                        self.state = ConfigState.READY
                        
                        # Notify callbacks if configuration changed
                        if old_config is not None and old_config != config:
                            self._log("info", f"Configuration changed: {config.broker}:{config.broker_port}")
                            for callback in self._change_callbacks:
                                try:
                                    callback(config)
                                except Exception as e:
                                    self._log("error", f"Error in config change callback: {e}")
                        elif old_config is None:
                            self._log("info", f"Initial configuration loaded: {config.broker}:{config.broker_port}")
                    
                    return config
                else:
                    # Check if error is permanent (don't retry)
                    if response.status_code in [400, 401, 403]:
                        self._log("error", f"Permanent error fetching config: HTTP {response.status_code}")
                        self.state = ConfigState.ERROR
                        return None
                    else:
                        # Temporary error - retry
                        if attempt < max_retries - 1:
                            delay = retry_delays[attempt]
                            self._log(
                                "warn",
                                f"Temporary error fetching config: HTTP {response.status_code}. "
                                f"Retrying in {delay}s (attempt {attempt + 1}/{max_retries})",
                            )
                            time.sleep(delay)
                            continue
                        else:
                            self._log("error", f"Failed to fetch config after {max_retries} attempts: HTTP {response.status_code}")
                            self.state = ConfigState.ERROR
                            return None
                    
            except requests.exceptions.Timeout:
                # Temporary error - retry
                if attempt < max_retries - 1:
                    delay = retry_delays[attempt]
                    self._log(
                        "warn",
                        f"Timeout fetching config from {url}. "
                        f"Retrying in {delay}s (attempt {attempt + 1}/{max_retries})",
                    )
                    time.sleep(delay)
                    continue
                else:
                    self._log("error", f"Timeout fetching config after {max_retries} attempts")
                    self.state = ConfigState.ERROR
                    return None
                    
            except CircuitBreakerOpenError:
                # Circuit breaker is open - use cached config
                self._log("warn", "Circuit breaker OPEN - using cached config")
                cached_config = self.get_config()
                if cached_config:
                    return cached_config
                else:
                    self._log("error", "Circuit breaker OPEN and no cached config")
                    self.state = ConfigState.ERROR
                    return None
                    
            except requests.exceptions.ConnectionError:
                # Temporary error - retry (circuit breaker will track failures)
                if attempt < max_retries - 1:
                    delay = retry_delays[attempt]
                    self._log(
                        "warn",
                        f"Cannot connect to Config Service at {url}. "
                        f"Retrying in {delay}s (attempt {attempt + 1}/{max_retries})",
                    )
                    time.sleep(delay)
                    continue
                else:
                    self._log("error", f"Cannot connect to Config Service after {max_retries} attempts")
                    self.state = ConfigState.ERROR
                    return None
                    
            except Exception as e:
                # Unexpected error - retry once, then fail (circuit breaker will track failures)
                if attempt < max_retries - 1:
                    delay = retry_delays[attempt]
                    self._log(
                        "warn",
                        f"Unexpected error fetching config: {e}. "
                        f"Retrying in {delay}s (attempt {attempt + 1}/{max_retries})",
                    )
                    time.sleep(delay)
                    continue
                else:
                    self._log("error", f"Error fetching config after {max_retries} attempts: {e}")
                    self.state = ConfigState.ERROR
                    return None
        
        # Should not reach here, but just in case
        self.state = ConfigState.ERROR
        return None
    
    def get_config(self) -> Optional[BrokerConfig]:
        """
        Get current cached configuration.
        
        Returns:
            BrokerConfig if available, None otherwise
        """
        with self._lock:
            return self.current_config
    
    def watch(self, callback: Callable[[BrokerConfig], None]):
        """
        Register callback for configuration changes.
        
        Args:
            callback: Function to call when configuration changes
        """
        with self._lock:
            if callback not in self._change_callbacks:
                self._change_callbacks.append(callback)

    # ------------------------------------------------------------------
    # Backward-compatible API (used by navigation_integrated_node)
    # ------------------------------------------------------------------

    def set_polling_interval(self, interval: float) -> None:
        """Set polling interval used by start_polling()."""
        try:
            self._polling_interval = float(interval)
        except Exception:
            # Keep previous value on bad input.
            self._polling_interval = self._polling_interval

    def add_change_callback(self, callback: Callable[[BrokerConfig], None]) -> None:
        """Alias for watch()."""
        self.watch(callback)

    def remove_change_callback(self, callback: Callable[[BrokerConfig], None]) -> None:
        """Alias for unwatch()."""
        self.unwatch(callback)
    
    def unwatch(self, callback: Callable[[BrokerConfig], None]):
        """
        Unregister callback for configuration changes.
        
        Args:
            callback: Function to remove from callbacks
        """
        with self._lock:
            if callback in self._change_callbacks:
                self._change_callbacks.remove(callback)
    
    def start_polling(self, interval: Optional[float] = None):
        """
        Start polling for configuration changes.
        
        Args:
            interval: Polling interval in seconds. If None, uses the previously
                configured value (default is 5.0).
        """
        if self._polling_active:
            self._log("warn", "Polling already active")
            return

        if interval is not None:
            self._polling_interval = float(interval)
        self._polling_active = True
        
        def polling_loop():
            while self._polling_active:
                self.fetch_config()
                time.sleep(self._polling_interval)
        
        self._polling_thread = threading.Thread(target=polling_loop, daemon=True)
        self._polling_thread.start()
        self._log("info", f"Started polling for config changes (interval: {self._polling_interval}s)")
    
    def stop_polling(self):
        """Stop polling for configuration changes."""
        if not self._polling_active:
            return
        
        self._polling_active = False
        if self._polling_thread:
            self._polling_thread.join(timeout=2.0)
        self._log("info", "Stopped polling for config changes")
    
    def is_ready(self) -> bool:
        """Check if configuration is ready."""
        return self.state == ConfigState.READY and self.current_config is not None

