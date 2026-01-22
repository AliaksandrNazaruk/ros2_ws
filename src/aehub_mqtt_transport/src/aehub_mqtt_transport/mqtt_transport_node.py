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
MQTT Transport Node

Pure transport layer between ROS2 and MQTT broker.
- Consumes BrokerConfig from ROS topic (parametrized)
- Manages MQTT connection lifecycle
- Provides ROS ↔ MQTT bridge using a typed envelope (`aehub_msgs/MqttEnvelope`)

Does NOT:
- Parse JSON
- Know about Nav2
- Store credentials
- Handle business logic
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from aehub_msgs.msg import BrokerConfig
from aehub_msgs.msg import MqttEnvelope

try:
    from .backoff import ExponentialBackoff
    from .mqtt_client import MqttClient
except ImportError:
    # Fallback for installed console_script execution (no package context)
    from aehub_mqtt_transport.backoff import ExponentialBackoff
    from aehub_mqtt_transport.mqtt_client import MqttClient


class MqttTransportNode(LifecycleNode):
    """
    MQTT transport layer.
    
    - Consumes BrokerConfig from ROS
    - Manages MQTT connection lifecycle
    - Provides ROS ↔ MQTT bridge
    """

    def __init__(self):
        super().__init__('mqtt_transport_node')

        # Parameters
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mqtt_connect_timeout_sec', 5.0)
        self.declare_parameter('mqtt_reconnect_initial_sec', 1.0)
        self.declare_parameter('mqtt_reconnect_max_sec', 30.0)
        # TLS CA certificate path (optional)
        self.declare_parameter('mqtt_ca_cert_path', '')
        # Topic parameters (relative by default; allow namespacing)
        self.declare_parameter('broker_config_topic', 'infra/mqtt/broker_config')
        self.declare_parameter('ros_in_topic', 'infra/mqtt/in')
        self.declare_parameter('ros_out_topic', 'infra/mqtt/out')
        self.declare_parameter('mqtt_subscribe_topic', 'aroc/robot/{robot_id}/commands/#')

        # Internal state
        self._broker_config: Optional[BrokerConfig] = None
        self._mqtt_client: Optional[MqttClient] = None
        self._mqtt_lock = threading.Lock()
        self._backoff: Optional[ExponentialBackoff] = None
        self._stop_event = threading.Event()
        self._reconnect_thread: Optional[threading.Thread] = None

        self._ros_in_pub = None
        self._ros_out_sub = None

        # rclpy LifecycleNode API differs across distros; keep explicit active flag.
        self._active = False

        # QoS for config (latched, transient_local)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._cfg_sub = self.create_subscription(
            BrokerConfig,
            str(self.get_parameter('broker_config_topic').value),
            self._on_broker_config,
            qos,
        )

        self.get_logger().info('MqttTransportNode constructed')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_configure(self, state: State):
        """Configure lifecycle node."""
        self.get_logger().info('Configuring MQTT transport')
        self._stop_event.clear()
        
        # Initialize backoff policy
        initial = self.get_parameter('mqtt_reconnect_initial_sec').value
        maximum = self.get_parameter('mqtt_reconnect_max_sec').value
        self._backoff = ExponentialBackoff(initial=initial, maximum=maximum)

        # ROS in/out (typed envelope only)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
        )
        ros_in_topic = str(self.get_parameter('ros_in_topic').value)
        ros_out_topic = str(self.get_parameter('ros_out_topic').value)
        self._ros_in_pub = self.create_publisher(MqttEnvelope, ros_in_topic, qos_reliable)
        self._ros_out_sub = self.create_subscription(MqttEnvelope, ros_out_topic, self._on_ros_out, qos_reliable)
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        """Activate lifecycle node."""
        self.get_logger().info('Activating MQTT transport')
        self._active = True

        if self._broker_config is None:
            self.get_logger().warn(
                'No BrokerConfig received yet, waiting before MQTT connect'
            )
            return TransitionCallbackReturn.SUCCESS

        self._connect_async()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """Deactivate lifecycle node."""
        self.get_logger().info('Deactivating MQTT transport')
        self._active = False
        self._stop_event.set()
        
        self._disconnect()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        """Cleanup lifecycle node."""
        self.get_logger().info('Cleaning up MQTT transport')
        self._active = False
        self._disconnect()

        if self._ros_out_sub is not None:
            try:
                self.destroy_subscription(self._ros_out_sub)
            except Exception:
                pass
        self._ros_out_sub = None

        if self._ros_in_pub is not None:
            try:
                self.destroy_publisher(self._ros_in_pub)
            except Exception:
                pass
        self._ros_in_pub = None

        self._backoff = None
        self._broker_config = None
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State):
        """Handle error state."""
        self.get_logger().error('MqttTransportNode entered error state')
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # BrokerConfig handling
    # ------------------------------------------------------------------

    def _on_broker_config(self, msg: BrokerConfig):
        """Handle BrokerConfig update."""
        self.get_logger().info(
            f'Received BrokerConfig: {msg.broker}:{msg.broker_port} '
            f'(TLS={msg.mqtt_use_tls}, user={msg.mqtt_user})'
        )

        reconnect_needed = (
            self._broker_config is None or
            self._config_changed(self._broker_config, msg)
        )

        self._broker_config = msg

        # If active and config changed, reconnect
        if self._is_active() and reconnect_needed:
            self.get_logger().info('Broker config changed → reconnecting MQTT')
            self._disconnect()
            self._connect_async()

    def _on_ros_out(self, msg: MqttEnvelope) -> None:
        """ROS → MQTT: publish raw payload to mqtt_topic from envelope."""
        if not self._is_active():
            return
        mqtt = self._mqtt_client
        if mqtt is None or not mqtt.is_connected():
            return

        topic = str(msg.mqtt_topic or "")
        if not topic:
            self.get_logger().warn("Dropping outbound MqttEnvelope: empty mqtt_topic")
            return
        payload = str(msg.payload_raw or "")
        try:
            mqtt.publish(topic, payload, qos=1, retain=False)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish MQTT message: {e}")

    @staticmethod
    def _config_changed(a: BrokerConfig, b: BrokerConfig) -> bool:
        """Check if broker configuration changed."""
        return (
            a.broker != b.broker or
            a.broker_port != b.broker_port or
            a.mqtt_user != b.mqtt_user or
            a.mqtt_password != b.mqtt_password or
            a.mqtt_use_tls != b.mqtt_use_tls or
            a.mqtt_tls_insecure != b.mqtt_tls_insecure
        )

    def _is_active(self) -> bool:
        """Check if node is in active state."""
        return bool(self._active)

    # ------------------------------------------------------------------
    # MQTT management
    # ------------------------------------------------------------------

    def _connect_async(self):
        """Start async connection in background thread."""
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            return
        
        self._reconnect_thread = threading.Thread(
            target=self._connect_loop,
            daemon=True
        )
        self._reconnect_thread.start()

    def _connect_loop(self):
        """Connection loop with exponential backoff."""
        if not self._backoff:
            self.get_logger().error('Backoff not initialized')
            return
        
        self._backoff.reset()

        while not self._stop_event.is_set():
            try:
                self._connect_once()
                return
            except Exception as e:
                self.get_logger().error(f'MQTT connect failed: {e}')
                delay = self._backoff.next_delay()
                if self._stop_event.wait(timeout=delay):
                    return

    def _connect_once(self):
        """Perform single MQTT connection attempt."""
        if self._broker_config is None:
            raise RuntimeError('No BrokerConfig available')

        with self._mqtt_lock:
            # Clean up old client if exists
            if self._mqtt_client:
                self._mqtt_client.disconnect()

            robot_id = self.get_parameter('robot_id').value or 'unknown'
            client_id = f'aehub-{robot_id}-{self.get_name()}'
            
            # Create and configure MQTT client
            self._mqtt_client = MqttClient(client_id=client_id)
            ca_cert_path = str(self.get_parameter('mqtt_ca_cert_path').value or "")
            self._mqtt_client.configure(self._broker_config, ca_cert_path=ca_cert_path)
            self._mqtt_client.set_callbacks(
                on_connect=self._on_mqtt_connect,
                on_disconnect=self._on_mqtt_disconnect,
                on_message=None,
            )

            self.get_logger().info(
                f'Connecting to MQTT '
                f'{self._broker_config.broker}:{self._broker_config.broker_port}'
            )

            self._mqtt_client.connect(
                self._broker_config.broker,
                self._broker_config.broker_port,
                keepalive=30,
            )

    def _disconnect(self):
        """Gracefully disconnect MQTT client."""
        with self._mqtt_lock:
            if self._mqtt_client:
                try:
                    self._mqtt_client.disconnect()
                except Exception as e:
                    self.get_logger().warn(f'Error during MQTT disconnect: {e}')
                self._mqtt_client = None

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection event."""
        if rc == 0:
            with self._mqtt_lock:
                if self._mqtt_client:
                    self._mqtt_client.set_connected(True)
            self.get_logger().info('MQTT connected')

            # Subscribe to inbound MQTT topic pattern and forward as MqttEnvelope.
            mqtt = self._mqtt_client
            if mqtt is not None:
                robot_id = str(self.get_parameter('robot_id').value or 'unknown')
                pattern = str(self.get_parameter('mqtt_subscribe_topic').value or "")
                pattern = pattern.format(robot_id=robot_id)
                mqtt.subscribe(pattern, qos=1, callback=self._on_mqtt_message)
                self.get_logger().info(f"MQTT subscribed: {pattern}")
        else:
            self.get_logger().error(f'MQTT connection failed rc={rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """Handle MQTT disconnection event."""
        with self._mqtt_lock:
            if self._mqtt_client:
                self._mqtt_client.set_connected(False)
        
        if not self._stop_event.is_set() and self._is_active():
            self.get_logger().warn('MQTT disconnected unexpectedly, reconnecting')
            self._connect_async()

    def _on_mqtt_message(self, topic: str, payload: bytes) -> None:
        """MQTT → ROS: publish typed envelope."""
        if not self._is_active() or self._ros_in_pub is None:
            return
        env = MqttEnvelope()
        env.mqtt_topic = str(topic)
        try:
            env.payload_raw = payload.decode("utf-8")
        except Exception:
            # Keep bytes best-effort as latin-1 roundtrip to avoid crashes.
            env.payload_raw = payload.decode("latin-1", errors="replace")
        env.received_at = self.get_clock().now().to_msg()
        self._ros_in_pub.publish(env)


def main(args=None):
    """Main entry point for mqtt_transport_node."""
    rclpy.init(args=args)
    
    node = MqttTransportNode()
    
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
            pass


if __name__ == '__main__':
    main()
