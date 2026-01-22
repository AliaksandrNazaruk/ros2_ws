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
Broker Credentials node.

ROS2 lifecycle node that fetches MQTT broker configuration from Config Service
and publishes it to ROS topic /aehub/mqtt/broker_config.

Responsibilities:
- Read /api/v1/config/broker
- Validate response
- Detect changes
- Publish configuration to ROS

Does NOT:
- MQTT connection
- TLS setup
- Reconnect logic
- Retries MQTT
- Business logic
"""

import hashlib
import json

from aehub_msgs.msg import BrokerConfig
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import requests

try:
    from .broker_client import BrokerConfigClient
except ImportError:
    # Fallback for when running as installed script
    from aehub_broker_credentials.broker_client import BrokerConfigClient


class BrokerCredentialsNode(LifecycleNode):
    """
    Lifecycle node for fetching and publishing broker configuration.

    Publishes BrokerConfig messages to /aehub/mqtt/broker_config topic
    with transient_local durability for late-joining subscribers.
    """

    def __init__(self):
        super().__init__('broker_credentials_node')

        # Declare parameters
        self.declare_parameter('config_service_url', '')
        self.declare_parameter('api_key', '')
        self.declare_parameter('poll_interval_sec', 5.0)
        self.declare_parameter('request_timeout_sec', 2.0)
        self.declare_parameter('fail_on_startup', False)
        # Topic parameters (allow namespacing via launch)
        # Default is relative (no leading '/'): will live under namespace robot/<id>/...
        self.declare_parameter('broker_config_topic', 'infra/mqtt/broker_config')

        # Create publisher with transient_local QoS for late-joining subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        broker_config_topic = str(self.get_parameter('broker_config_topic').value)
        self._pub = self.create_publisher(
            BrokerConfig,
            broker_config_topic,
            qos,
        )

        # Internal state
        self._client = None
        self._timer = None
        self._last_hash = None
        self._startup_failed = False

    def on_configure(self, state: State):
        """
        Configure lifecycle node.

        Creates HTTP client and validates parameters.
        """
        url = self.get_parameter('config_service_url').value
        api_key = self.get_parameter('api_key').value
        timeout = self.get_parameter('request_timeout_sec').value

        if not url or not api_key:
            self.get_logger().error('config_service_url or api_key missing')
            self.get_logger().error(f"  config_service_url: '{url}' (empty={not url})")
            self.get_logger().error(
                (
                    f"  api_key: '{api_key[:10]}...' (empty={not api_key})"
                    if api_key
                    else "  api_key: '' (empty=True)"
                )
            )
            return rclpy.lifecycle.TransitionCallbackReturn.FAILURE

        try:
            self._client = BrokerConfigClient(url, api_key, timeout)
            self.get_logger().info('BrokerCredentialsNode configured')
            self.get_logger().info(f'  URL: {url}, timeout: {timeout}s')
            self.get_logger().info(f'  API key: {api_key[:10]}... (length: {len(api_key)})')
            return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to configure client: {e}')
            return rclpy.lifecycle.TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State):
        """
        Activate lifecycle node.

        Starts polling timer and performs initial fetch.
        """
        interval = self.get_parameter('poll_interval_sec').value
        fail_on_startup = bool(self.get_parameter('fail_on_startup').value)

        # Perform initial fetch (errors are handled inside _poll)
        self._poll()

        # Check if we should fail on startup (if no config was published)
        if fail_on_startup and self._last_hash is None:
            self.get_logger().error('Initial fetch failed and fail_on_startup=true')
            return rclpy.lifecycle.TransitionCallbackReturn.FAILURE

        # Create timer for periodic polling
        self._timer = self.create_timer(interval, self._poll)
        self.get_logger().info(f'Activated: polling every {interval}s')
        return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """
        Deactivate lifecycle node.

        Stops polling timer.
        """
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info('BrokerCredentialsNode deactivated')
        return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        """
        Cleanup lifecycle node.

        Releases resources.
        """
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self._client = None
        self._last_hash = None
        self.get_logger().info('BrokerCredentialsNode cleaned up')
        return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State):
        """
        Handle error state.

        Logs error and attempts recovery.
        """
        self.get_logger().error('BrokerCredentialsNode entered error state')
        return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

    def _poll(self):
        """
        Poll Config Service for broker configuration.

        Fetches configuration, validates it, detects changes,
        and publishes to ROS topic if changed.
        """
        if self._client is None:
            self.get_logger().error('Client not initialized')
            return

        try:
            # Fetch configuration
            data = self._client.fetch()

            # Validate required fields explicitly
            required_fields = [
                'broker',
                'broker_port',
                'mqtt_user',
                'mqtt_use_tls',
                'mqtt_tls_insecure',
            ]
            missing_fields = [k for k in required_fields if k not in data]
            if missing_fields:
                raise ValueError(f'Missing required fields in broker config: {missing_fields}')

            # Create hash of payload to detect changes
            payload = json.dumps(data, sort_keys=True).encode()
            h = hashlib.sha256(payload).hexdigest()

            # Skip if unchanged
            if h == self._last_hash:
                return

            # Validate and extract fields
            broker = str(data.get('broker', '')).strip()
            if not broker:
                raise ValueError('Broker field is missing or empty')

            broker_port = int(data.get('broker_port', 1883))
            if broker_port <= 0 or broker_port > 65535:
                raise ValueError(f'Invalid broker_port: {broker_port} (must be 1-65535)')

            mqtt_user = str(data.get('mqtt_user', '')).strip()
            if not mqtt_user:
                raise ValueError('mqtt_user field is missing or empty')

            mqtt_password = str(data.get('mqtt_password', ''))
            # Validate that password is not redacted.
            # API should return actual password when include_password=true is used.
            if mqtt_password in ['***REDACTED***', '***', '']:
                raise ValueError(
                    'mqtt_password is redacted - API did not return actual password '
                    '(check include_password=true parameter)'
                )

            mqtt_use_tls = bool(data.get('mqtt_use_tls', False))
            mqtt_tls_insecure = bool(data.get('mqtt_tls_insecure', False))

            # Create and publish message
            msg = BrokerConfig(
                broker=broker,
                broker_port=broker_port,
                mqtt_user=mqtt_user,
                mqtt_password=mqtt_password,
                mqtt_use_tls=mqtt_use_tls,
                mqtt_tls_insecure=mqtt_tls_insecure,
                fetched_at=self.get_clock().now().to_msg(),
            )

            self._pub.publish(msg)
            self._last_hash = h
            self.get_logger().info(
                f'Broker config updated: {broker}:{broker_port} '
                f'(TLS: {mqtt_use_tls}, user: {mqtt_user})'
            )

        except requests.exceptions.Timeout:
            # Temporary network error - use WARN level
            self.get_logger().warn('Broker config fetch timeout (temporary network issue)')
        except requests.exceptions.ConnectionError:
            # Temporary network error - use WARN level
            self.get_logger().warn('Broker config fetch connection error (temporary)')
        except requests.exceptions.HTTPError as e:
            # HTTP errors (4xx, 5xx) - use ERROR for client errors, WARN for server errors
            if e.response and 400 <= e.response.status_code < 500:
                self.get_logger().error(
                    f'Broker config fetch failed (client error): HTTP {e.response.status_code}'
                )
            else:
                self.get_logger().warn(
                    'Broker config fetch failed (server error): HTTP '
                    f"{e.response.status_code if e.response else 'unknown'}"
                )
        except requests.exceptions.RequestException as e:
            # Other request exceptions - use WARN for temporary issues
            self.get_logger().warn(f'Broker config fetch failed (HTTP): {e}')
        except (KeyError, ValueError, TypeError) as e:
            # Validation errors - always ERROR (data corruption or API contract violation)
            self.get_logger().error(f'Broker config validation failed: {e}')
        except Exception as e:
            # Unexpected errors - always ERROR
            self.get_logger().error(f'Broker config fetch failed (unexpected): {e}')


def main(args=None):
    """Run `broker_credentials_node`."""
    rclpy.init(args=args)

    node = BrokerCredentialsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # rclpy may already be shutdown by launch system
            pass


if __name__ == '__main__':
    main()
