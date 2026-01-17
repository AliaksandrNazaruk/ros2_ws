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

"""aehub_navigation.mqtt_connection_manager

A pragmatic MQTT connection manager for the robot-side command service.

Key goals (AE.HUB MVP):
- One shared MQTT client for status + command events + command subscriptions.
- Safe reconnect on broker config change without restarting the ROS2 node.
- TLS support (with optional CA fetch from Config Service).
- Minimal work inside paho callbacks (no long blocking operations).

This file intentionally keeps the public surface area used by
NavigationIntegratedNode:
- connect(config) -> bool
- reconnect(config) -> bool
- disconnect() -> None
- subscribe(topic, qos) -> bool
- publish(topic, payload, qos, retain) -> bool
- get_connection_status() -> dict

The previous version contained extensive debug scaffolding and a logger wrapper
that incorrectly passed `exc_info` into rclpy loggers (which raises TypeError).
This implementation avoids `exc_info` and logs tracebacks as strings.
"""

from __future__ import annotations

import atexit
import os
import ssl
import tempfile
import threading
import time
import traceback
from dataclasses import asdict
from typing import Optional, Callable, List, Tuple, Dict, Any

import paho.mqtt.client as mqtt
import requests

from aehub_navigation.broker_config_provider import BrokerConfig


class MQTTConnectionManager:
    def __init__(
        self,
        logger=None,
        config_service_url: Optional[str] = None,
        api_key: Optional[str] = None,
        *,
        connect_timeout_s: float = 10.0,
        ca_cache_ttl_s: float = 300.0,
        client_id_prefix: str = "nav_integrated",
        fail_closed_tls: bool = True,
        reconnect_backoff_enabled: bool = True,
        reconnect_backoff_initial_s: float = 1.0,
        reconnect_backoff_max_s: float = 60.0,
        reconnect_backoff_multiplier: float = 2.0,
        reconnect_max_attempts: int = 10,
    ):
        self.logger = logger
        self.config_service_url = config_service_url.rstrip("/") if config_service_url else None
        self.api_key = api_key

        self.connect_timeout_s = float(connect_timeout_s)
        self.ca_cache_ttl_s = float(ca_cache_ttl_s)
        self.client_id_prefix = str(client_id_prefix)
        self.fail_closed_tls = bool(fail_closed_tls)

        self.reconnect_backoff_enabled = bool(reconnect_backoff_enabled)
        self.reconnect_backoff_initial_s = float(reconnect_backoff_initial_s)
        self.reconnect_backoff_max_s = float(reconnect_backoff_max_s)
        self.reconnect_backoff_multiplier = float(reconnect_backoff_multiplier)
        self.reconnect_max_attempts = int(reconnect_max_attempts)

        # Public callbacks (assigned by NavigationIntegratedNode)
        self.on_connect_callback: Optional[Callable[[mqtt.Client, Any, Dict[str, Any], int], None]] = None
        self.on_message_callback: Optional[Callable[[mqtt.Client, Any, mqtt.MQTTMessage], None]] = None

        # Internal state
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()

        self.client: Optional[mqtt.Client] = None
        self.is_connected: bool = False
        self.current_config: Optional[BrokerConfig] = None
        self.subscriptions: List[Tuple[str, int]] = []

        # CA cache
        self._ca_file_path: Optional[str] = None
        self._ca_file_expires_at: float = 0.0

        atexit.register(self._atexit_cleanup)

    # ------------------------- logging helpers -------------------------

    def _log(self, level: str, msg: str) -> None:
        if self.logger is None:
            return
        try:
            if level == "debug":
                self.logger.debug(msg)
            elif level == "info":
                self.logger.info(msg)
            elif level == "warn":
                self.logger.warn(msg)
            else:
                self.logger.error(msg)
        except Exception:
            # Never throw from logging
            pass

    # ------------------------- CA certificate -------------------------

    def _get_ca_file_path(self) -> Optional[str]:
        """Fetch or reuse cached CA certificate file."""
        now = time.time()
        if self._ca_file_path and now < self._ca_file_expires_at and os.path.exists(self._ca_file_path):
            return self._ca_file_path

        if not self.config_service_url or not self.api_key:
            return None

        headers = {"X-API-Key": self.api_key}
        candidates = [
            f"{self.config_service_url}/api/v1/config/certificates/ca",
            f"{self.config_service_url}/config/certificates/ca",
        ]

        last_exc = None
        for url in candidates:
            try:
                r = requests.get(url, headers=headers, timeout=5.0)
                r.raise_for_status()
                pem = r.text
                if not pem or "BEGIN CERTIFICATE" not in pem:
                    raise ValueError("CA response does not look like PEM")

                fd, path = tempfile.mkstemp(prefix="aehub_ca_", suffix=".crt")
                with os.fdopen(fd, "w") as f:
                    f.write(pem)

                # Replace old file
                if self._ca_file_path and self._ca_file_path != path:
                    try:
                        os.unlink(self._ca_file_path)
                    except Exception:
                        pass

                self._ca_file_path = path
                self._ca_file_expires_at = now + self.ca_cache_ttl_s
                self._log("info", f"Fetched CA certificate from Config Service: {url}")
                return path
            except Exception as e:
                last_exc = e

        self._log("warn", f"Failed to fetch CA certificate from Config Service: {last_exc}")
        return None

    def _atexit_cleanup(self) -> None:
        try:
            self.disconnect()
        except Exception:
            pass
        try:
            if self._ca_file_path and os.path.exists(self._ca_file_path):
                os.unlink(self._ca_file_path)
        except Exception:
            pass

    # ------------------------- MQTT callbacks -------------------------

    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: Dict[str, Any], rc: int) -> None:
        with self._lock:
            self.is_connected = (rc == 0)
        self._connected_event.set()

        if rc == 0:
            self._log("info", "MQTT connected")
            # Resubscribe
            try:
                for topic, qos in list(self.subscriptions):
                    client.subscribe(topic, qos=qos)
            except Exception:
                self._log("warn", f"Resubscribe error:\n{traceback.format_exc()}")
        else:
            self._log("error", f"MQTT connect failed: rc={rc}")

        # Delegate
        if self.on_connect_callback:
            try:
                self.on_connect_callback(client, userdata, flags, rc)
            except Exception:
                self._log("warn", f"User on_connect callback raised:\n{traceback.format_exc()}")

    def _on_disconnect(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        with self._lock:
            self.is_connected = False
        if self._stop_event.is_set():
            self._log("info", "MQTT disconnected (shutdown)")
        else:
            self._log("warn", f"MQTT disconnected: rc={rc}")

    def _on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        if self.on_message_callback:
            try:
                self.on_message_callback(client, userdata, msg)
            except Exception:
                self._log("warn", f"User on_message callback raised:\n{traceback.format_exc()}")

    # ------------------------- public API -------------------------

    def connect(self, config: BrokerConfig) -> bool:
        """Create client and connect. Idempotent: reconnects if config changed."""
        with self._lock:
            self.current_config = config

        self._stop_event.clear()
        self._connected_event.clear()

        # Tear down any old client
        self.disconnect()

        # Build new client
        client_id = f"{self.client_id_prefix}-{int(time.time())}"  # good enough
        client = mqtt.Client(client_id=client_id, clean_session=True)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message

        client.username_pw_set(config.mqtt_user, config.mqtt_password)

        # TLS
        if config.mqtt_use_tls:
            ca_path = self._get_ca_file_path()
            if not ca_path and self.fail_closed_tls and not config.mqtt_tls_insecure:
                self._log("error", "TLS is enabled but CA is unavailable (fail-closed)")
                return False

            try:
                if ca_path:
                    client.tls_set(ca_certs=ca_path, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS)
                else:
                    # Insecure mode without CA (testing only)
                    client.tls_set(cert_reqs=ssl.CERT_NONE, tls_version=ssl.PROTOCOL_TLS)

                client.tls_insecure_set(bool(config.mqtt_tls_insecure or (not ca_path)))
            except Exception:
                self._log("error", f"Failed to configure TLS:\n{traceback.format_exc()}")
                return False

        try:
            client.connect_async(config.broker, int(config.broker_port), keepalive=30)
            client.loop_start()
        except Exception:
            self._log("error", f"MQTT connect_async failed:\n{traceback.format_exc()}")
            return False

        with self._lock:
            self.client = client

        # Wait for connection outcome
        if not self._connected_event.wait(timeout=self.connect_timeout_s):
            self._log("error", f"MQTT connect timeout after {self.connect_timeout_s}s")
            return False

        with self._lock:
            return bool(self.is_connected)

    def disconnect(self) -> None:
        with self._lock:
            client = self.client
            self.client = None
            self.is_connected = False
        if client is None:
            return
        try:
            self._stop_event.set()
            try:
                client.loop_stop()
            except Exception:
                pass
            try:
                client.disconnect()
            except Exception:
                pass
        finally:
            self._stop_event.clear()

    def reconnect(self, new_config: BrokerConfig) -> bool:
        """Reconnect with backoff. Synchronous; returns True if connected."""
        attempts = 0
        delay = self.reconnect_backoff_initial_s
        while True:
            attempts += 1
            ok = self.connect(new_config)
            if ok:
                return True

            if self.reconnect_max_attempts > 0 and attempts >= self.reconnect_max_attempts:
                self._log("error", f"Reconnect failed after {attempts} attempts")
                return False

            if not self.reconnect_backoff_enabled:
                time.sleep(0.5)
                continue

            time.sleep(min(delay, self.reconnect_backoff_max_s))
            delay = min(delay * self.reconnect_backoff_multiplier, self.reconnect_backoff_max_s)

    def subscribe(self, topic: str, qos: int = 0) -> bool:
        if not topic:
            return False
        with self._lock:
            if (topic, qos) not in self.subscriptions:
                self.subscriptions.append((topic, qos))
            client = self.client
            connected = self.is_connected

        if client and connected:
            try:
                client.subscribe(topic, qos=qos)
            except Exception:
                self._log("warn", f"Subscribe failed for {topic}:\n{traceback.format_exc()}")
                return False
        return True

    def publish(self, topic: str, payload: str, qos: int = 0, retain: bool = False) -> bool:
        with self._lock:
            client = self.client
            connected = self.is_connected
        if not client or not connected:
            return False
        try:
            info = client.publish(topic, payload=payload, qos=int(qos), retain=bool(retain))
            return bool(info.rc == mqtt.MQTT_ERR_SUCCESS)
        except Exception:
            self._log("warn", f"Publish failed for {topic}:\n{traceback.format_exc()}")
            return False

    def get_connection_status(self) -> dict:
        with self._lock:
            cfg = self.current_config
            status = {
                "is_connected": self.is_connected,
                "has_client": self.client is not None,
                "broker": cfg.broker if cfg else None,
                "port": cfg.broker_port if cfg else None,
                "tls_enabled": cfg.mqtt_use_tls if cfg else None,
                "subscriptions_count": len(self.subscriptions),
                "subscriptions": [{"topic": t, "qos": q} for (t, q) in self.subscriptions],
            }
            if cfg:
                try:
                    status["config"] = asdict(cfg)
                except Exception:
                    status["config"] = {
                        "broker": cfg.broker,
                        "broker_port": cfg.broker_port,
                        "mqtt_use_tls": cfg.mqtt_use_tls,
                        "mqtt_tls_insecure": cfg.mqtt_tls_insecure,
                    }
            return status
