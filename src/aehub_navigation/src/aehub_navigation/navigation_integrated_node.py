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
Integrated Navigation Node

Combines all navigation components in a single node:
- MQTT command handling
- Nav2 Action Client
- State management
- Status publishing

AE.HUB MVP: Single integrated node for navigation control.

MQTT Configuration:
- NO local MQTT configuration storage
- Configuration fetched from centralized Broker Config Service
- Dynamic reconnection without ROS2 node restart
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState
from aehub_navigation.position_registry import PositionRegistry
from aehub_navigation.mqtt_status_publisher import MQTTStatusPublisher
from aehub_navigation.mqtt_command_event_publisher import MQTTCommandEventPublisher
from aehub_navigation.broker_config_provider import BrokerConfig, BrokerConfigProvider
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
from aehub_navigation.command_validator import CommandValidator
from aehub_navigation.command_rate_limiter import CommandRateLimiter
from aehub_navigation.navigation_action_client import NavigationActionClient
from aehub_navigation.error_handler import NavigationErrorHandler
import json
import os
import math
import threading
import time
import requests
import socket
import hashlib
import fcntl
import sys
from urllib.parse import urlparse
from typing import Optional, Dict, Any


class NavigationIntegratedNode(Node):
    def __init__(self):
        # Check for duplicate instances BEFORE creating the node - CRITICAL: prevent duplicate nodes
        lock_file_path = '/tmp/navigation_integrated_node.lock'
        lock_file = None
        try:
            # Try to acquire exclusive lock on lock file
            lock_file = open(lock_file_path, 'w')
            try:
                # Try to acquire non-blocking exclusive lock
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                # Lock acquired successfully - write PID to file
                lock_file.write(str(os.getpid()) + '\n')
                lock_file.flush()
                # Keep file open to maintain lock
                self._lock_file = lock_file
            except BlockingIOError:
                # Lock is held by another process - duplicate detected!
                lock_file.close()
                error_msg = (
                    'CRITICAL ERROR: navigation_integrated_node is already running!\n'
                    'Another instance is holding the lock file. '
                    'This will cause MQTT topic conflicts and unpredictable behavior.\n'
                    'Only one instance should be running.\n'
                    'Please stop the existing instance before starting a new one.\n'
                    f'Lock file: {lock_file_path}'
                )
                print(error_msg, file=sys.stderr)
                sys.exit(1)
        except Exception as e:
            if lock_file:
                lock_file.close()
            # If lock file operations fail, try ROS2 graph check as fallback
            print(f'WARNING: Could not use file lock for duplicate check: {e}', file=sys.stderr)
            print('Attempting ROS2 graph check as fallback...', file=sys.stderr)
        
        # Now create the ROS2 node
        super().__init__('navigation_integrated_node')
        
        # Additional check via ROS2 graph (fallback/secondary check)
        try:
            # Wait a bit for ROS2 graph to stabilize
            time.sleep(0.5)
            
            existing_nodes = self.get_node_names()
            duplicate_count = sum(1 for name in existing_nodes if name == 'navigation_integrated_node')
            
            if duplicate_count > 1:
                error_msg = (
                    f'CRITICAL ERROR: Multiple navigation_integrated_node instances detected in ROS2 graph ({duplicate_count}). '
                    f'This will cause MQTT topic conflicts and unpredictable behavior. '
                    f'Only one instance should be running. '
                    f'Please stop all instances and restart with a single launch file.'
                )
                self.get_logger().error(error_msg)
                # Release lock file before exiting
                if hasattr(self, '_lock_file'):
                    try:
                        fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
                        self._lock_file.close()
                    except Exception:
                        pass
                # Exit with error to prevent duplicate from running
                raise RuntimeError(error_msg)
        except RuntimeError:
            # Re-raise our intentional error
            raise
        except Exception as e:
            self.get_logger().warn(f'Could not check for duplicate instances via ROS2 graph: {e}. Continuing anyway...')
        
        # Parameters
        # NOTE: mqtt_broker and mqtt_port are DEPRECATED and NOT USED
        # All MQTT configuration comes from Broker Config Service
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('auto_unique_robot_id', False)  # Auto-generate unique robot_id for dev
        self.declare_parameter('config_service_url', 'http://localhost:7900')
        self.declare_parameter('config_service_api_key', '')
        self.declare_parameter('config_poll_interval', 5.0)  # seconds
        self.declare_parameter('positions_file', 'config/positions.yaml')
        self.declare_parameter('rate_limit_interval', 0.1)  # seconds

        # Symovo readiness check (reject navigation if robot cannot drive)
        self.declare_parameter('symovo_endpoint', 'https://192.168.1.100')
        self.declare_parameter('amr_id', 15)
        self.declare_parameter('tls_verify', False)
        self.declare_parameter('symovo_ready_check_enabled', False)  # Default: disabled for testing
        self.declare_parameter('symovo_ready_check_timeout_s', 3.0)
        self.declare_parameter('symovo_ready_check_fail_open', False)
        self.declare_parameter('symovo_auto_enable_drive_mode', False)
        self.declare_parameter('symovo_client_cert_file', '')
        self.declare_parameter('symovo_client_key_file', '')
        
        robot_id_param = self.get_parameter('robot_id').value
        auto_unique = self.get_parameter('auto_unique_robot_id').value
        
        # Auto-generate unique robot_id for dev environments
        if auto_unique or os.getenv('AEHUB_AUTO_UNIQUE_ROBOT_ID', '').lower() in ('1', 'true', 'yes'):
            self.robot_id = self._generate_unique_robot_id(robot_id_param)
            self.get_logger().info(f'Auto-generated unique robot_id for dev environment: {self.robot_id}')
        else:
            self.robot_id = robot_id_param
        config_service_url = self.get_parameter('config_service_url').value
        config_service_api_key = self.get_parameter('config_service_api_key').value
        config_poll_interval = self.get_parameter('config_poll_interval').value
        positions_file = self.get_parameter('positions_file').value
        rate_limit_interval = float(self.get_parameter('rate_limit_interval').value)

        # Symovo readiness settings
        self.symovo_endpoint = str(self.get_parameter('symovo_endpoint').value).rstrip('/')
        self.symovo_amr_id = int(self.get_parameter('amr_id').value)
        self.symovo_tls_verify = bool(self.get_parameter('tls_verify').value)
        self.symovo_ready_check_enabled = bool(self.get_parameter('symovo_ready_check_enabled').value)
        self.symovo_ready_check_timeout_s = float(self.get_parameter('symovo_ready_check_timeout_s').value)
        self.symovo_ready_check_fail_open = bool(self.get_parameter('symovo_ready_check_fail_open').value)
        self.symovo_auto_enable_drive_mode = bool(self.get_parameter('symovo_auto_enable_drive_mode').value)
        self.symovo_client_cert_file = str(self.get_parameter('symovo_client_cert_file').value)
        self.symovo_client_key_file = str(self.get_parameter('symovo_client_key_file').value)
        
        self.get_logger().info(f' Initializing NavigationIntegratedNode: robot_id={self.robot_id}')
        if self.symovo_ready_check_enabled:
            self.get_logger().info(
                f' Symovo readiness check enabled: endpoint={self.symovo_endpoint}, '
                f'amr_id={self.symovo_amr_id}, tls_verify={self.symovo_tls_verify}, '
                f'fail_open={self.symovo_ready_check_fail_open}, auto_enable_drive_mode={self.symovo_auto_enable_drive_mode}'
            )
        
        # NOTE: Keep runtime code free of IDE-specific debug logging (e.g. writing into .cursor/).
        
        # Validate API key
        if not config_service_api_key:
            self.get_logger().error(' config_service_api_key is required but not provided')
            raise RuntimeError('config_service_api_key parameter is required')

        # Components
        self.state_manager = NavigationStateManager(logger=self.get_logger())
        self.position_registry = PositionRegistry()
        
        # Load positions
        positions_path = positions_file
        if not os.path.isabs(positions_path):
            from ament_index_python.packages import get_package_share_directory
            try:
                pkg_dir = get_package_share_directory('aehub_navigation')
                positions_path = os.path.join(pkg_dir, 'config', 'positions.yaml')
            except Exception:
                positions_path = positions_file
        
        if not self.position_registry.loadFromYAML(positions_path):
            self.get_logger().error(f'Failed to load positions from {positions_path}')
            raise RuntimeError('Failed to load positions')
        
        # New refactored components
        self.command_validator = CommandValidator(max_processed_ids=1000, ttl_seconds=3600)
        self.rate_limiter = CommandRateLimiter(min_interval_seconds=rate_limit_interval)
        self.error_handler = NavigationErrorHandler(
            logger=self.get_logger(),
            status_publisher=None  # Will be set after status_publisher is created
        )
        
        # Nav2 Action Client (wrapped)
        self.nav_action_client = NavigationActionClient(self, action_name='navigate_to_pose')
        # Keep direct reference for backward compatibility during transition
        self.nav2_action_client = self.nav_action_client._action_client
        
        # Broker Config Provider (centralized configuration)
        self.config_provider = BrokerConfigProvider(
            config_service_url=config_service_url,
            api_key=config_service_api_key,
            logger=self.get_logger()
        )
        
        # MQTT Connection Manager (dynamic reconnection support)
        self.mqtt_manager = MQTTConnectionManager(
            logger=self.get_logger(),
            config_service_url=config_service_url,
            api_key=config_service_api_key
        )
        self.mqtt_manager.on_connect_callback = self.on_mqtt_connect
        self.mqtt_manager.on_message_callback = self.on_mqtt_message
        
        # MQTT Status Publisher will be initialized AFTER current_position is created
        
        # Current goal tracking (thread-safe)
        # 
        # CRITICAL: Lock ordering for deadlock prevention
        # ================================================
        # When acquiring multiple locks, ALWAYS acquire them in this strict order:
        # 1. _mqtt_ready_lock (outermost - protects MQTT connection state)
        # 2. _rate_limit_lock (if exists - protects rate limiting state)
        # 3. _goal_state_lock (protects current goal, target_id, command_id)
        # 4. _position_lock (protects current_position)
        # 5. _progress_lock (innermost - protects progress tracking)
        #
        # This ordering prevents deadlocks by ensuring all threads acquire locks
        # in the same sequence. Violating this order can cause deadlocks.
        #
        # Example of CORRECT lock ordering:
        #   with self._goal_state_lock:
        #       with self._position_lock:
        #           # Access both goal state and position
        #
        # Example of INCORRECT lock ordering (DO NOT DO THIS):
        #   with self._position_lock:
        #       with self._goal_state_lock:  # WRONG - violates lock order!
        #           # This can cause deadlock!
        #
        self._goal_state_lock = threading.Lock()
        self.current_goal_handle = None
        self.current_target_id = None
        self.current_command_id = None  # Track command_id for status correlation
        
        # MQTT ready flag (thread-safe)
        # Protected by _mqtt_ready_lock - indicates if MQTT is ready to accept commands
        # Set to False during reconnection to prevent command processing
        self._mqtt_ready_lock = threading.Lock()
        self._mqtt_ready = False  # Flag to prevent commands during reconnect
        
        # Current position tracking (from AMCL or odom) - thread-safe
        # Protected by _position_lock - updated from AMCL (priority) or odom (fallback)
        self._position_lock = threading.Lock()
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.current_position_valid = False
        
        # MQTT Status Publisher (will use mqtt_manager)
        # Initialize AFTER current_position is created
        self.status_publisher = MQTTStatusPublisher()
        self.status_publisher.set_mqtt_manager(self.mqtt_manager)
        self.status_publisher.set_robot_id(self.robot_id)
        # Command events publisher
        self.command_event_publisher = MQTTCommandEventPublisher()
        self.command_event_publisher.set_mqtt_manager(self.mqtt_manager)
        self.command_event_publisher.set_robot_id(self.robot_id)
        
        # Update error handler with status publisher
        self.error_handler._status_publisher = self.status_publisher
        
        # Initialize status publisher with IDLE state
        self.status_publisher.updateStatus(
            state=NavigationState.IDLE,
            target_id=None,
            command_id=None,
            current_position=self.current_position
        )
        
        # Navigation progress tracking (thread-safe)
        # Protected by _progress_lock - tracks navigation progress, distance, ETA
        # Lock order: _progress_lock is innermost (acquired last, released first)
        self._progress_lock = threading.Lock()
        self.initial_distance = None  # Initial distance to goal (set from first feedback)
        self.last_distance_remaining = None  # Last distance remaining from feedback
        self.last_eta_seconds = None  # Last ETA in seconds from feedback
        self.last_feedback_time = None  # Timestamp of last feedback for velocity calculation
        self.average_velocity = None  # Average velocity for ETA fallback calculation (m/s)
        
        # Subscribe to AMCL pose (priority) and odom (fallback)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Link state manager to status publisher
        self.state_manager.setStateChangeCallback(self.on_state_change)
        
        # Register callback for config changes
        self.config_provider.watch(self.on_config_changed)
        
        # Fetch initial configuration and connect
        self.get_logger().info(' Fetching MQTT configuration from Config Service...')
        initial_config = self.config_provider.fetch_config()
        
        if initial_config is None:
            self.get_logger().error(' Failed to fetch MQTT configuration from Config Service')
            self.get_logger().error('   Cannot proceed without MQTT configuration')
            raise RuntimeError('Failed to fetch MQTT configuration from Config Service')
        
        self.get_logger().info(f' MQTT configuration fetched: broker={initial_config.broker}, port={initial_config.broker_port}, TLS={initial_config.mqtt_use_tls}')
        
        # Connect to MQTT using fetched configuration
        # Note: Connection failure is not fatal - node will continue and retry
        # NOTE: Do NOT set _mqtt_ready here - it will be set in on_mqtt_connect callback
        # Setting it here causes race condition because connect() is async
        self.get_logger().info('🔌 Connecting to MQTT broker...')
        connect_result = self.mqtt_manager.connect(initial_config)
        if connect_result:
            # Connection initiated, but wait for callback to set _mqtt_ready
            self.get_logger().info(' MQTT connection initiated - waiting for callback to set ready flag')
        else:
            # Check if connection is actually established (callback may have set it)
            conn_status = self.mqtt_manager.get_connection_status()
            self.get_logger().warn(f'  MQTT connect() returned False, but checking actual connection status: {conn_status}')
            
            # If actually connected (callback set it), don't reset the flag
            if conn_status.get('is_connected', False):
                self.get_logger().info(' MQTT is actually connected (callback set it), keeping _mqtt_ready flag')
            else:
                self.get_logger().warn('  Failed to connect to MQTT broker - will retry on config polling')
                self.get_logger().warn('   Node will continue but commands will be blocked until connection is established')
                with self._mqtt_ready_lock:
                    # Only reset if not already set by callback
                    if not self._mqtt_ready:
                        self._mqtt_ready = False
            # Don't raise exception - allow node to start and retry connection
        
        # Initialize status publisher with robot_id (needed for schema_version)
        self.status_publisher.set_robot_id(self.robot_id)
        
        # Start polling for configuration changes
        self.config_provider.start_polling(interval=config_poll_interval)
        
        # Setup Nav2 action client callbacks
        self.nav_action_client.set_goal_response_callback(self.goal_response_callback)
        self.nav_action_client.set_feedback_callback(self.feedback_callback)
        self.nav_action_client.set_result_callback(self.result_callback)
        
        # Wait for Nav2 action server
        self.get_logger().info(' Waiting for Nav2 action server (timeout: 10s)...')
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('  Nav2 action server not available, will retry on goal send')
            self.get_logger().warn('   Run: python3 scripts/diagnose_nav2_lifecycle.py to check lifecycle nodes')
            self.get_logger().warn('   Or: POST /api/process/nav2/activate_lifecycle to manually activate')
            # Note: We don't set error state here, as server might become available later
        else:
            self.get_logger().info(' Nav2 action server is available')
        
        self.get_logger().info(' NavigationIntegratedNode initialized successfully')
    
    def _generate_unique_robot_id(self, base_robot_id: str) -> str:
        """
        Generate unique robot_id for dev environments.
        
        Format: {hostname}-{base_robot_id}-{hash_suffix}
        Example: raspberrypi-robot_001-a3f2b1
        
        Args:
            base_robot_id: Base robot_id from parameter (used as fallback/prefix)
            
        Returns:
            Unique robot_id string
        """
        try:
            # Get hostname
            hostname = socket.gethostname()
            # Sanitize hostname (remove invalid chars for MQTT topics)
            hostname_clean = ''.join(c if c.isalnum() or c in ('-', '_') else '-' for c in hostname).lower()
            # Limit hostname length
            hostname_clean = hostname_clean[:20]
            
            # Generate short hash suffix from hostname + timestamp + PID
            hash_input = f"{hostname}{time.time()}{os.getpid()}"
            hash_suffix = hashlib.md5(hash_input.encode()).hexdigest()[:6]
            
            # Combine: hostname-base-suffix
            unique_id = f"{hostname_clean}-{base_robot_id}-{hash_suffix}"
            
            # Ensure it's valid for MQTT topics (alphanumeric, -, _)
            unique_id = ''.join(c if c.isalnum() or c in ('-', '_') else '-' for c in unique_id)
            
            return unique_id
        except Exception as e:
            self.get_logger().warn(f'  Failed to generate unique robot_id: {e}, using base: {base_robot_id}')
            # Fallback: add timestamp to base
            return f"{base_robot_id}-{int(time.time())}"
    
    def on_mqtt_connect(self, client: Any, userdata: Any, flags: Any, rc: int) -> None:
        """
        MQTT connection callback.
        
        Called when MQTT connection is established or fails. Handles subscription
        to command topics and sets MQTT ready flag.
        
        Args:
            client: MQTT client instance
            userdata: User data (unused)
            flags: Connection flags
            rc: Return code (0 = success, non-zero = error)
        """
        # NOTE: IDE debug logging removed (no writes to .cursor/).
        
        self.get_logger().info(f'🔌 on_mqtt_connect callback called: rc={rc}, flags={flags}')
        
        if rc == 0:
            self.get_logger().info(' Connected to MQTT broker successfully')
            
            try:
                # Subscribe to navigateTo command topic
                navigate_topic = f'aroc/robot/{self.robot_id}/commands/navigateTo'
                self.get_logger().info(f' Subscribing to navigateTo topic: {navigate_topic}')
                result = self.mqtt_manager.subscribe(navigate_topic, qos=1)
                self.get_logger().info(f' Subscribed to: {navigate_topic}, result: {result}')
            except Exception as e:
                self.get_logger().error(f' Failed to subscribe to navigateTo topic: {e}')
                import traceback
                self.get_logger().error(f' Traceback: {traceback.format_exc()}')
            
            try:
                # Subscribe to cancel command topic
                cancel_topic = f'aroc/robot/{self.robot_id}/commands/cancel'
                self.get_logger().info(f' Subscribing to cancel topic: {cancel_topic}')
                result = self.mqtt_manager.subscribe(cancel_topic, qos=1)
                self.get_logger().info(f' Subscribed to: {cancel_topic}, result: {result}')
            except Exception as e:
                self.get_logger().error(f' Failed to subscribe to cancel topic: {e}')
                import traceback
                self.get_logger().error(f' Traceback: {traceback.format_exc()}')
            
            with self._mqtt_ready_lock:
                self._mqtt_ready = True
                self.get_logger().info(' MQTT ready flag set to True (in callback)')
            # Log after lock release for clarity
            self.get_logger().info(' MQTT is now ready to accept commands')
            
            # Publish current status after connection (one-shot)
            self.get_logger().debug('Publishing initial status after connection...')
            try:
                # Get current state and update status publisher before publishing
                current_state = self.state_manager.getState()
                with self._goal_state_lock:
                    current_command_id = self.current_command_id
                self.status_publisher.updateStatus(
                    state=current_state,
                    target_id=None,
                    command_id=current_command_id,
                    current_position=self.current_position
                )
                self.status_publisher.publishStatus()
                self.get_logger().info(' Initial status published after MQTT connection')
            except Exception as e:
                self.get_logger().error(f' Failed to publish initial status: {e}')
        else:
            # Map MQTT error codes
            error_messages = {
                1: "Connection refused - incorrect protocol version",
                2: "Connection refused - invalid client identifier",
                3: "Connection refused - server unavailable",
                4: "Connection refused - bad username or password",
                5: "Connection refused - not authorized"
            }
            error_msg = error_messages.get(rc, f"Unknown error code: {rc}")
            self.get_logger().error(f' Failed to connect to MQTT broker: rc={rc} ({error_msg})')
            with self._mqtt_ready_lock:
                self._mqtt_ready = False
            self.get_logger().debug('MQTT ready flag set to False due to connection failure')
    
    def on_mqtt_message(self, client: Any, userdata: Any, msg: Any) -> None:
        """
        MQTT message callback with validation and rate limiting.
        
        Validates incoming MQTT messages and routes them to appropriate handlers.
        Implements payload size limits and basic injection protection.
        
        Args:
            client: MQTT client instance
            userdata: User data (unused)
            msg: MQTT message with topic and payload
        """
        # NOTE: IDE debug logging removed (no writes to .cursor/).
        
        self.get_logger().info(f' Received MQTT message: topic={msg.topic}, payload_size={len(msg.payload)} bytes')
        
        try:
            # Validate payload size (prevent DoS)
            max_payload_size = 10 * 1024  # 10 KB limit
            if len(msg.payload) > max_payload_size:
                self.get_logger().error(
                    f' MQTT payload too large: {len(msg.payload)} bytes (max: {max_payload_size}). '
                    f'Topic: {msg.topic}'
                )
                return
            
            # Validate topic format (basic injection protection)
            if not isinstance(msg.topic, str):
                self.get_logger().error(f' Invalid topic type: {type(msg.topic)}')
                return
            
            if len(msg.topic) > 512:  # MQTT topic length limit
                self.get_logger().error(f' Topic too long: {len(msg.topic)} characters')
                return
            
            self.get_logger().debug(f'Validating MQTT message: topic={msg.topic}, payload_size={len(msg.payload)}')
            
            # Decode payload with size check
            try:
                payload_str = msg.payload.decode('utf-8')
                self.get_logger().debug(f'Payload decoded successfully, length: {len(payload_str)} chars')
            except UnicodeDecodeError as e:
                self.get_logger().error(f' Failed to decode MQTT payload as UTF-8: {e}')
                return
            
            # Parse JSON with size validation
            try:
                payload = json.loads(payload_str)
                self.get_logger().debug(f'JSON parsed successfully, type: {type(payload).__name__}')
            except json.JSONDecodeError as e:
                self.get_logger().error(
                    f' Failed to parse MQTT message JSON: {e}. '
                    f'Topic: {msg.topic}, Payload preview: {payload_str[:100]}'
                )
                return
            
            # Validate payload is a dictionary
            if not isinstance(payload, dict):
                self.get_logger().error(
                    f' MQTT payload must be a JSON object, got {type(payload).__name__}. '
                    f'Topic: {msg.topic}'
                )
                return
            
            self.get_logger().info(f' MQTT message validated: topic={msg.topic}, payload_keys={list(payload.keys())}')
            
            # Route message based on topic
            if msg.topic.endswith('/commands/navigateTo'):
                # NOTE: IDE debug logging removed (no writes to .cursor/).
                
                self.get_logger().info(f'🔄 Routing to handleNavigationCommand: {msg.topic}')
                # Publish ACK received as early as possible
                try:
                    cmd_id = payload.get('command_id', 'unknown') if isinstance(payload, dict) else 'unknown'
                    if isinstance(payload, dict):
                        tgt = payload.get('target_id')
                        if not tgt and ('x' in payload and 'y' in payload):
                            tgt = '__pose__'
                        if not tgt:
                            tgt = 'unknown'
                    else:
                        tgt = 'unknown'
                    self.command_event_publisher.publish_ack(cmd_id, tgt, ack_type='received')
                except Exception:
                    pass
                self.handleNavigationCommand(payload)
            elif msg.topic.endswith('/commands/cancel'):
                self.get_logger().info(f'🔄 Routing to handleCancelCommand: {msg.topic}')
                # Publish ACK received as early as possible (SPECIFICATION.md ordering guarantee)
                try:
                    cmd_id = payload.get('command_id', 'unknown') if isinstance(payload, dict) else 'unknown'
                    self.command_event_publisher.publish_ack(cmd_id, None, ack_type='received')
                except Exception:
                    pass
                self.handleCancelCommand(payload)
            else:
                self.get_logger().warn(f'  Unknown topic: {msg.topic}')
        
        except Exception as e:
            self.get_logger().error(f' Error handling MQTT message: {e}')
    
    def validateCommand(self, command: dict) -> tuple[bool, Optional[str]]:
        """
        Validate navigation command using CommandValidator.
        
        Args:
            command: Command dictionary to validate
            
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if command is valid, False otherwise
            - error_message: Error message if validation failed, None if valid
        """
        return self.command_validator.validate_command(command, self.position_registry, expected_robot_id=self.robot_id)

    def _fetch_symovo_agv_item(self) -> tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        """
        Fetch Symovo /v0/agv list and return the item for the configured AMR.

        Returns:
            (item, error_details)
        """
        url = f"{self.symovo_endpoint}/v0/agv"
        try:
            res = requests.get(url, verify=self.symovo_tls_verify, timeout=self.symovo_ready_check_timeout_s)
            res.raise_for_status()
            data = res.json()
            if not isinstance(data, list):
                return None, {"url": url, "reason": "symovo_invalid_response", "type": type(data).__name__}

            for item in data:
                if item.get("id") == self.symovo_amr_id:
                    return item, None

            return None, {"url": url, "reason": "amr_id_not_found", "amr_id": self.symovo_amr_id}

        except Exception as e:
            return None, {"url": url, "reason": "symovo_unreachable", "error": str(e)}

    def _check_symovo_drive_ready(self) -> tuple[bool, str, Dict[str, Any]]:
        """
        Check Symovo state_flags to determine if robot is ready to drive.

        Returns:
            (ok, reason, details)
        """
        item, err = self._fetch_symovo_agv_item()
        if err is not None:
            return False, err.get("reason", "symovo_unreachable"), err
        assert item is not None

        flags = item.get("state_flags") or {}
        state = item.get("state")
        drive_ready = bool(flags.get("drive_ready", False))
        drive_manual = bool(flags.get("drive_manual", False))
        charging_connector = bool(flags.get("charging_connector", False))
        safety_cleared = bool(flags.get("safety_cleared", True))
        robot_paused = bool(flags.get("robot_paused", False))
        api_port = item.get("api_port")

        details = {
            "state": state,
            "drive_ready": drive_ready,
            "drive_manual": drive_manual,
            "charging_connector": charging_connector,
            "safety_cleared": safety_cleared,
            "robot_paused": robot_paused,
            "api_port": api_port,
        }

        if not safety_cleared:
            return False, "safety_not_cleared", details
        if robot_paused:
            return False, "robot_paused", details
        if drive_manual:
            return False, "manual_mode", details
        if charging_connector:
            return False, "charging_connector", details
        if not drive_ready:
            return False, "drive_not_ready", details

        return True, "ok", details

    def _try_enable_symovo_drive_mode(self) -> tuple[bool, str, Dict[str, Any]]:
        """
        Try to enable drive_mode / drive_ready via Symovo API.

        Notes:
        - Symovo privileged API may require a client certificate (mTLS), often exposed on api_port (e.g. 8003).
        - If client cert is not available, we will not be able to enable drive_mode programmatically.

        Returns:
            (ok, reason, details)
        """
        item, err = self._fetch_symovo_agv_item()
        if err is not None:
            return False, err.get("reason", "symovo_unreachable"), err
        assert item is not None

        api_port = item.get("api_port")
        parsed = urlparse(self.symovo_endpoint)
        host = parsed.hostname or parsed.path  # path fallback for endpoints without scheme
        scheme = parsed.scheme or "https"

        attempts: list[Dict[str, Any]] = []

        # 1) Try the proxy endpoint (some deployments expose it; ours returned 500 previously)
        try:
            url1 = f"{self.symovo_endpoint}/v0/agv/{self.symovo_amr_id}/move/drive_mode"
            r1 = requests.put(
                url1,
                json={"enable": True},
                verify=self.symovo_tls_verify,
                timeout=self.symovo_ready_check_timeout_s,
            )
            attempts.append({"url": url1, "status": r1.status_code, "body": (r1.text or "")[:200]})
        except Exception as e:
            attempts.append({"url": f"{self.symovo_endpoint}/v0/agv/{self.symovo_amr_id}/move/drive_mode", "exc": str(e)})

        # 2) Try the documented endpoint on api_port with optional client certificate
        cert_tuple = None
        if self.symovo_client_cert_file and self.symovo_client_key_file:
            cert_tuple = (self.symovo_client_cert_file, self.symovo_client_key_file)

        if api_port:
            url2 = f"{scheme}://{host}:{int(api_port)}/v0/amr/{self.symovo_amr_id}/move/drive_mode"
            try:
                r2 = requests.put(url2, verify=self.symovo_tls_verify, timeout=self.symovo_ready_check_timeout_s, cert=cert_tuple)
                attempts.append({"url": url2, "status": r2.status_code, "body": (r2.text or "")[:200]})
            except Exception as e:
                attempts.append({"url": url2, "exc": str(e)})

        # Re-check after attempts
        ok, reason, details = self._check_symovo_drive_ready()
        details = {**details, "drive_mode_attempts": attempts}
        if ok:
            return True, "ok", details

        # Special-case: if we saw 401, highlight mTLS requirement
        for a in attempts:
            if a.get("status") == 401:
                return False, "mtls_required", details
        return False, reason, details
    
    def on_config_changed(self, new_config: 'BrokerConfig') -> None:
        """
        Handle configuration change from Config Service (Section 6.2).
        
        Reconnects to new MQTT broker without restarting ROS2 node.
        
        Args:
            new_config: New broker configuration from Config Service
        """
        self.get_logger().info('MQTT configuration changed, reconnecting...')
        with self._mqtt_ready_lock:
            self._mqtt_ready = False  # Prevent commands during reconnect
        
        # MUST: Disconnect MQTT (handled by reconnect method)
        # MUST: Re-init client and connect to new broker (handled by reconnect method)
        # MUST: Re-subscribe topics (handled in on_mqtt_connect callback)
        # MUST: Publish current navigation status (one-shot)
        # MUST NOT: Restart ROS2 node (we don't)
        # MUST NOT: Cancel active Nav2 goal (we don't)
        if self.mqtt_manager.reconnect(new_config):
            self.get_logger().info('Successfully reconnected to new MQTT broker')
            with self._mqtt_ready_lock:
                self._mqtt_ready = True
            # Publish current status after reconnection (Section 6.2 requirement)
            # Get current state and update status publisher before publishing
            current_state = self.state_manager.getState()
            with self._goal_state_lock:
                current_command_id = self.current_command_id
            self.status_publisher.updateStatus(
                state=current_state,
                target_id=None,
                command_id=current_command_id,
                current_position=self.current_position
            )
            self.status_publisher.publishStatus()
        else:
            self.get_logger().error('Failed to reconnect to new MQTT broker')
            # Will retry on next poll
    
    def handleNavigationCommand(self, command: dict) -> None:
        """
        Handle navigation command from MQTT with improved error handling.
        
        Processes navigation commands with validation, rate limiting, and error handling.
        Publishes command events (ack/rejected/result) and updates navigation status.
        
        Args:
            command: Command dictionary with 'command_id', 'target_id', 'timestamp', 'priority'
        """
        command_id = command.get('command_id', 'unknown')
        target_id = command.get('target_id')
        if not target_id and ('x' in command and 'y' in command):
            target_id = '__pose__'
        if not target_id:
            target_id = 'unknown'
        
        # NOTE: IDE debug logging removed (no writes to .cursor/).
        
        self.get_logger().info(f' Starting navigation command processing: command_id={command_id}, target_id={target_id}')
        
        try:
            # Check if MQTT is ready (not reconnecting)
            with self._mqtt_ready_lock:
                mqtt_ready = self._mqtt_ready
            if not mqtt_ready:
                self.get_logger().warn(f'  MQTT not ready (reconnecting), ignoring command: command_id={command_id}')
                # Notify user that command was rejected using error_handler (includes command_id)
                try:
                    # Event: rejected
                    try:
                        self.command_event_publisher.publish_ack(command_id, target_id, ack_type='rejected', reason='mqtt_not_ready')
                    except Exception:
                        pass
                    self.error_handler.handle_error(
                        'NAV_MQTT_ERROR',
                        {'target_id': target_id, 'reason': 'MQTT reconnecting, command rejected. Please retry.'},
                        command_id
                    )
                    self._schedule_idle_reset(0.5)
                    self.get_logger().info(f' Published rejection status for command: {command_id}')
                except Exception as e:
                    self.get_logger().error(f' Failed to publish rejection status: {e}')
                return
            
            self.get_logger().info(' MQTT is ready, proceeding with command processing')
            
            # Rate limiting check using CommandRateLimiter
            # Check by global command counter to prevent rapid commands from any source
            # (command_id is always unique UUID, so we use a global key for rate limiting)
            # Use wall-clock time for rate limiting to avoid ROS clock discrepancies
            current_time = time.time()
            
            self.get_logger().info(f' [RATE LIMIT] Checking rate limit for command: command_id={command_id}, target_id={target_id}, current_time={current_time:.3f}')
            
            # Use global key for rate limiting (prevents rapid commands from any source)
            rate_limit_key = "__global__"  # Rate limit globally, not per target_id
            is_allowed, rate_limit_error = self.rate_limiter.check_rate_limit(rate_limit_key, current_time)
            
            self.get_logger().info(f' [RATE LIMIT] Result: is_allowed={is_allowed}, error={rate_limit_error}')
            
            if not is_allowed:
                self.get_logger().warn(f'  [RATE LIMIT] {rate_limit_error}')
                try:
                    self.command_event_publisher.publish_ack(command_id, target_id, ack_type='rejected', reason='rate_limited')
                except Exception:
                    pass
                self.error_handler.handle_error(
                    'NAV_RATE_LIMIT_EXCEEDED',
                    {'command_id': command_id, 'target_id': target_id},
                    command_id
                )
                self._schedule_idle_reset(0.5)
                return
            
            self.get_logger().info(' [RATE LIMIT] Command allowed, proceeding to validation')
            
            # Cleanup old entries periodically
            self.rate_limiter.cleanup_old_entries(max_age_seconds=3600, current_time=current_time)
            self.command_validator.cleanup_expired()
            
            # NOTE: IDE debug logging removed (no writes to .cursor/).
            
            # Validate command
            self.get_logger().info(f' [VALIDATION] Validating command: command_id={command_id}, target_id={target_id}')
            is_valid, error_msg = self.validateCommand(command)
            
            self.get_logger().info(f' [VALIDATION] Result: is_valid={is_valid}, error_msg={error_msg}')
            
            # NOTE: IDE debug logging removed (no writes to .cursor/).
            
            if not is_valid:
                self.get_logger().warn(f'  [VALIDATION] Command validation failed: {error_msg}')
                try:
                    self.command_event_publisher.publish_ack(command_id, target_id, ack_type='rejected', reason='validation_failed')
                except Exception:
                    pass
                self.error_handler.handle_error(
                    'NAV_INVALID_COMMAND',
                    {'command_id': command_id, 'target_id': target_id, 'error': error_msg},
                    command_id
                )
                self._schedule_idle_reset(0.5)
                return
            
            self.get_logger().info(' [VALIDATION] Command validation passed')
            
            self.get_logger().info(f' Command validated successfully: command_id={command_id}, target_id={target_id}')

            # Hardware readiness (Symovo): reject navigation when robot cannot drive (manual / not ready / charging)
            if self.symovo_ready_check_enabled:
                ok, reason, details = self._check_symovo_drive_ready()
                if not ok:
                    # Optional auto-enable drive mode (only if NOT in manual and NOT charging and safety ok)
                    if self.symovo_auto_enable_drive_mode and reason == "drive_not_ready":
                        self.get_logger().warn(f' Robot not drive-ready; attempting to enable drive mode: {details}')
                        en_ok, en_reason, en_details = self._try_enable_symovo_drive_mode()
                        if en_ok:
                            self.get_logger().info(' Symovo drive mode enabled; proceeding with navigation')
                        else:
                            self.get_logger().warn(f' Failed to enable drive mode: reason={en_reason}, details={en_details}')
                            reason = en_reason
                            details = en_details

                    # Fail-open: if enabled, ignore readiness check failures and proceed
                    if self.symovo_ready_check_fail_open:
                        self.get_logger().warn(f' Symovo readiness check failed open (ignoring): reason={reason}, details={details}')
                        # Continue with navigation despite readiness check failure
                    else:
                        self.get_logger().warn(f' Robot not ready to drive: reason={reason}, details={details}')
                        try:
                            self.command_event_publisher.publish_ack(
                                command_id, target_id, ack_type='rejected', reason=f'robot_not_ready:{reason}'
                            )
                        except Exception:
                            pass
                        self.error_handler.handle_error(
                            'NAV_ROBOT_NOT_READY',
                            {'reason': reason, **details},
                            command_id
                        )
                        self._schedule_idle_reset(0.5)
                        return
            
            # Mark command as processed AFTER successful validation
            # This ensures only valid commands are deduplicated
            self.command_validator.mark_as_processed(command_id)
            
            # Store command_id for status correlation (thread-safe)
            
            with self._goal_state_lock:
                self.current_command_id = command_id
            
            # Resolve goal → pose with error handling
            # Prefer direct x/y/theta if present (supports clients that don't use PositionRegistry)
            self.get_logger().debug(f' Resolving goal to pose: target_id={target_id}')
            try:
                pose = None
                if 'x' in command and 'y' in command:
                    pose = self._pose_from_direct_command(command)
                else:
                    pose = self.position_registry.getPosition(target_id)
                    if pose is None:
                        raise ValueError(f'Position registry returned None for target_id: {target_id}')
                
                self.get_logger().info(
                    f' Position resolved: target_id={target_id}, '
                    f'pose=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, '
                    f'theta={pose.pose.orientation.z:.2f})'
                )
            except Exception as e:
                self.error_handler.handle_error(
                    'NAV_INVALID_TARGET',
                    {'target_id': target_id, 'error': str(e)},
                    command_id
                )
                self._schedule_idle_reset(0.5)
                return
            
            # Cancel active goal if navigating (thread-safe)
            try:
                with self._goal_state_lock:
                    if self.state_manager.getState() == NavigationState.NAVIGATING:
                        self.get_logger().info('Cancelling current goal before sending new one')
                        self.cancelCurrentGoal()
            except Exception as e:
                # Log with context for debugging
                self.error_handler.log_exception(
                    e,
                    {'target_id': target_id, 'command_id': command_id, 'operation': 'cancel_current_goal'},
                    command_id
                )
                self.get_logger().warn(
                    f'  Error cancelling current goal, continuing with new goal: {e}'
                )
                # Continue anyway - try to send new goal (graceful degradation)
            
            # Send new Nav2 goal with error handling
            try:
                self.sendNav2Goal(pose, target_id, command_id)
            except Exception as e:
                self.error_handler.log_exception(
                    e,
                    {'target_id': target_id, 'command_id': command_id},
                    command_id
                )
                self.error_handler.handle_error(
                    'NAV_SERVER_UNAVAILABLE',
                    {'target_id': target_id, 'error': str(e)},
                    command_id
                )
                self._schedule_idle_reset(0.5)
        
        except Exception as e:
            # Catch-all for any unexpected errors
            command_id = command.get('command_id', 'unknown') if isinstance(command, dict) else 'unknown'
            target_id = command.get('target_id', 'unknown') if isinstance(command, dict) else 'unknown'
            self.error_handler.log_exception(
                e,
                {'target_id': target_id, 'command': str(command)[:200]},
                command_id
            )
            self.error_handler.handle_error(
                'NAV_UNKNOWN_ERROR',
                {'target_id': target_id, 'error': str(e)},
                command_id
            )
            self._schedule_idle_reset(0.5)

    def _pose_from_direct_command(self, command: Dict[str, Any]) -> PoseStamped:
        """
        Build a PoseStamped directly from MQTT command fields:
          - x, y required (meters)
          - theta or yaw optional (radians)
          - frame_id optional (defaults to 'map')
        """
        x = float(command.get('x'))
        y = float(command.get('y'))
        theta = command.get('theta', command.get('yaw', 0.0))
        theta = float(theta) if theta is not None else 0.0
        frame_id = command.get('frame_id', 'map')
        if not isinstance(frame_id, str) or not frame_id:
            frame_id = 'map'

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose
    
    def handleCancelCommand(self, command: dict) -> None:
        """
        Handle cancel command from MQTT with improved error handling.
        
        Cancels active navigation goal if navigating, or resets to IDLE state.
        
        Args:
            command: Cancel command dictionary with 'command_id', 'timestamp', optional 'reason'
        """
        try:
            # Check if MQTT is ready (not reconnecting)
            with self._mqtt_ready_lock:
                mqtt_ready = self._mqtt_ready
            if not mqtt_ready:
                self.get_logger().warn('MQTT not ready (reconnecting), ignoring cancel command')
                return
            
            # Validate cancel command
            if not isinstance(command, dict):
                self.get_logger().error(f'Invalid cancel command: expected dict, got {type(command)}')
                return
            
            required_fields = ['schema_version', 'robot_id', 'command_id', 'timestamp']
            for field in required_fields:
                if field not in command:
                    self.get_logger().error(
                        f'Invalid cancel command: Missing required field: {field}. '
                        f'Command: {command}'
                    )
                    try:
                        cmd_id = command.get('command_id', 'unknown')
                        self.command_event_publisher.publish_ack(cmd_id, None, ack_type='rejected', reason='validation_failed')
                    except Exception:
                        pass
                    self.error_handler.handle_error(
                        'NAV_INVALID_COMMAND',
                        {'target_id': None, 'error': f'Missing required field: {field}'},
                        command.get('command_id')
                    )
                    return
            
            command_id = command['command_id']
            reason = command.get('reason', 'user')

            # Strict schema checks
            if command.get('schema_version') != '1.0' or command.get('robot_id') != self.robot_id:
                try:
                    self.command_event_publisher.publish_ack(command_id, None, ack_type='rejected', reason='validation_failed')
                except Exception:
                    pass
                self.error_handler.handle_error(
                    'NAV_INVALID_COMMAND',
                    {
                        'target_id': None,
                        'error': f'schema_version/robot_id mismatch (schema_version={command.get("schema_version")}, robot_id={command.get("robot_id")})',
                    },
                    command_id
                )
                return
            
            # Validate reason (optional field)
            if reason not in ['user', 'system', 'emergency']:
                self.get_logger().warn(
                    f'Unknown cancel reason: {reason}, using default: user. '
                    f'Valid reasons: user, system, emergency'
                )
                reason = 'user'
            
            self.get_logger().info(
                f'Received cancel command: command_id={command_id}, reason={reason}'
            )
            # Cancel command accepted (even if idempotent)
            try:
                self.command_event_publisher.publish_ack(command_id, None, ack_type='accepted')
            except Exception:
                pass
            
            # Cancel active goal if navigating (idempotent)
            try:
                current_state = self.state_manager.getState()
                if current_state == NavigationState.NAVIGATING:
                    self.get_logger().info('Cancelling current navigation goal')
                    try:
                        self.state_manager.onCancelRequested(target_id=self.current_target_id)
                    except Exception:
                        pass
                    self.cancelCurrentGoal()
                    # Terminal result for the CANCEL command itself (command outcome), not the navigation goal.
                    try:
                        self.command_event_publisher.publish_result(command_id, None, result_type='succeeded')
                    except Exception:
                        pass
                elif current_state == NavigationState.IDLE:
                    self.get_logger().debug('Already in IDLE state, cancel is idempotent')
                    try:
                        self.command_event_publisher.publish_result(command_id, None, result_type='succeeded')
                    except Exception:
                        pass
                else:
                    # For any other state (SUCCEEDED, ABORTED, ERROR, etc), transition to IDLE
                    self.get_logger().info(
                        f'Transitioning from {current_state.value} to IDLE due to cancel command'
                    )
                    self.state_manager.resetToIdle()
                    try:
                        self.command_event_publisher.publish_result(command_id, None, result_type='succeeded')
                    except Exception:
                        pass
            except Exception as e:
                # Log with context for debugging
                self.error_handler.log_exception(
                    e,
                    {'command_id': command_id, 'reason': reason, 'operation': 'cancel_goal'},
                    command_id
                )
                self.get_logger().error(
                    f'Error during cancel operation: {e}'
                )
                # Try to reset to idle anyway (graceful degradation)
                try:
                    self.state_manager.resetToIdle()
                except Exception as reset_error:
                    self.error_handler.log_exception(
                        reset_error,
                        {'command_id': command_id, 'operation': 'reset_to_idle_after_cancel_error'},
                        command_id
                    )
                    self.get_logger().error(
                        f'Failed to reset to idle: {reset_error}',
                        exc_info=True
                    )
        
        except Exception as e:
            # Catch-all for any unexpected errors in cancel command handling
            command_id = command.get('command_id', 'unknown') if isinstance(command, dict) else 'unknown'
            self.error_handler.log_exception(
                e,
                {'command': str(command)[:200], 'operation': 'handleCancelCommand'},
                command_id
            )
            self.get_logger().error(
                f'Unexpected error in handleCancelCommand: {e}',
                exc_info=True
            )
    
    def sendNav2Goal(self, pose: PoseStamped, target_id: str, command_id: Optional[str] = None) -> None:
        """
        Send goal to Nav2 using NavigationActionClient.
        
        Args:
            pose: Target pose (PoseStamped)
            target_id: Target position ID
            command_id: Command ID for status correlation (optional)
        """
        self.get_logger().info(f' Preparing to send Nav2 goal: target_id={target_id}, command_id={command_id}')
        
        # Check if Nav2 server is available
        if not self.nav_action_client.server_is_ready():
            self.error_handler.handle_error(
                'NAV_SERVER_UNAVAILABLE',
                {'target_id': target_id},
                command_id
            )
            self._schedule_idle_reset(0.5)
            return
        
        # Send goal using NavigationActionClient
        success = self.nav_action_client.send_goal(pose, target_id, command_id)
        if success:
            self.state_manager.onGoalSent(target_id)
            self.get_logger().info(f' Goal sent to Nav2, state updated: target_id={target_id}')
        else:
            self.error_handler.handle_error(
                'NAV_SERVER_UNAVAILABLE',
                {'target_id': target_id, 'error': 'Failed to send goal'},
                command_id
            )
            self._schedule_idle_reset(0.5)
    
    def cancelCurrentGoal(self) -> None:
        """
        Cancel current Nav2 goal using NavigationActionClient.
        
        Idempotent - safe to call even if no goal is active.
        """
        success = self.nav_action_client.cancel_goal()
        if success:
            self.get_logger().info(' Cancel request sent to Nav2')
        else:
            self.get_logger().debug('No active goal to cancel (idempotent)')
    
    def _schedule_idle_reset(self, delay_s: float) -> None:
        """Schedule a one-shot transition back to IDLE."""
        timer_holder = {}

        def _reset_once():
            try:
                self.state_manager.resetToIdle()
            except Exception as e:
                self.get_logger().warn(f' Failed to reset to IDLE: {e}')
            finally:
                timer = timer_holder.get('timer')
                if timer:
                    timer.cancel()

        timer_holder['timer'] = self.create_timer(delay_s, _reset_once)

    def goal_response_callback(self, future, target_id=None, command_id=None):
        """Handle goal response from NavigationActionClient
        
        Args:
            future: Future from send_goal_async
            target_id: Target ID passed from closure (optional, falls back to state_manager)
            command_id: Command ID passed from closure (optional, falls back to current_command_id)
        """
        self.get_logger().info(f' Goal response callback triggered: target_id={target_id}, command_id={command_id}')
        
        try:
            goal_handle = future.result()
            self.get_logger().debug(f'Goal handle received: accepted={goal_handle.accepted if goal_handle else "None"}')
        except Exception as e:
            self.error_handler.log_exception(e, {'target_id': target_id}, command_id)
            return
        
        if not goal_handle.accepted:
            self.get_logger().warn(f' Goal rejected by Nav2: target_id={target_id}')
            rejected_target_id = target_id if target_id is not None else self.state_manager.getTargetId()
            
            # Event: ack rejected by server
            try:
                self.command_event_publisher.publish_ack(command_id or 'unknown', rejected_target_id, ack_type='rejected', reason='nav2_goal_rejected')
            except Exception:
                pass
            self.error_handler.handle_error(
                'NAV_GOAL_REJECTED',
                {'target_id': rejected_target_id},
                command_id
            )
            
            # Clear state
            self.nav_action_client.clear_current_goal()
            with self._goal_state_lock:
                self.current_goal_handle = None
                self.current_target_id = None
                self.current_command_id = None
            
            # Reset progress tracking
            with self._progress_lock:
                self.initial_distance = None
                self.last_distance_remaining = None
                self.last_eta_seconds = None
                self.last_feedback_time = None
                self.average_velocity = None
            return
        
        self.get_logger().info(f' Goal accepted by Nav2: target_id={target_id}')
        # Event: ack accepted
        try:
            self.command_event_publisher.publish_ack(command_id or 'unknown', target_id, ack_type='accepted')
        except Exception:
            pass
        
        # Get target_id and command_id from parameters or state
        if target_id is None:
            target_id = self.state_manager.getTargetId()
            self.get_logger().debug(f'Using target_id from state_manager: {target_id}')
        
        if command_id is None:
            with self._goal_state_lock:
                command_id = self.current_command_id
        
        # Store goal handle in NavigationActionClient and update local state
        self.nav_action_client.set_current_goal(goal_handle, target_id, command_id)
        
        with self._goal_state_lock:
            self.current_goal_handle = goal_handle
            self.current_target_id = target_id
            self.current_command_id = command_id
        
        # Reset progress tracking for new goal
        with self._progress_lock:
            self.initial_distance = None
            self.last_distance_remaining = None
            self.last_eta_seconds = None
    
    def feedback_callback(self, feedback_msg: Any) -> None:
        """
        Handle feedback from Nav2 with improved progress and ETA calculation.
        
        Processes navigation feedback, calculates progress percentage and ETA,
        and publishes status updates.
        
        Args:
            feedback_msg: Nav2 feedback message containing distance_remaining and estimated_time_remaining
        """
        # Guard: Ignore feedback if goal was cancelled (race condition protection)
        with self._goal_state_lock:
            goal_handle = self.current_goal_handle
            if goal_handle is None:
                return
        
        try:
            feedback = feedback_msg.feedback
            
            # Get current time for velocity calculation
            from rclpy.clock import Clock
            current_time = Clock().now()
            current_time_sec = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] / 1e9
            
            # Get distance remaining with validation
            try:
                # Jazzy: NavigateToPose.Feedback.distance_remaining is a float (not std_msgs/Float32)
                dr = feedback.distance_remaining
                distance_remaining = float(dr.data) if hasattr(dr, "data") else float(dr)
            except AttributeError as e:
                self.get_logger().error(f'Feedback missing distance_remaining: {e}')
                return
            
            # Validate distance_remaining
            if distance_remaining < 0:
                self.get_logger().warn(f'Invalid distance_remaining: {distance_remaining}, setting to 0')
                distance_remaining = 0
            
            # Set initial distance from first feedback if not set
            with self._progress_lock:
                if self.initial_distance is None:
                    self.initial_distance = distance_remaining
                    self.last_feedback_time = current_time_sec
                    self.get_logger().info(f'Initial distance to goal: {self.initial_distance:.2f}m')
                    # Initialize average velocity with a reasonable default (0.5 m/s)
                    self.average_velocity = 0.5
                else:
                    # Calculate velocity from distance change
                    if self.last_feedback_time is not None and self.last_distance_remaining is not None:
                        time_delta = current_time_sec - self.last_feedback_time
                        if time_delta > 0:
                            distance_delta = self.last_distance_remaining - distance_remaining
                            instant_velocity = distance_delta / time_delta
                            
                            # Update average velocity (exponential moving average)
                            if self.average_velocity is None:
                                self.average_velocity = max(0.1, instant_velocity)  # Minimum 0.1 m/s
                            else:
                                # EMA with alpha=0.3 (30% weight to new value)
                                self.average_velocity = 0.7 * self.average_velocity + 0.3 * max(0.1, instant_velocity)
                    
                    self.last_feedback_time = current_time_sec
                
                # Get estimated time remaining from Nav2 if available
                eta_seconds = 0
                try:
                    if hasattr(feedback, 'estimated_time_remaining') and feedback.estimated_time_remaining:
                        # Convert Duration to seconds
                        eta_seconds = feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec / 1e9
                except Exception as e:
                    self.get_logger().debug(f'Error reading estimated_time_remaining: {e}')
                
                # Fallback: Calculate ETA from distance and average velocity
                if eta_seconds <= 0 and self.average_velocity is not None and self.average_velocity > 0.01:
                    try:
                        eta_seconds = distance_remaining / self.average_velocity
                    except ZeroDivisionError:
                        self.get_logger().warn('Average velocity is zero, cannot calculate ETA')
                        eta_seconds = 0
                
                # Calculate progress percentage with improved validation
                progress_percent = 0
                try:
                    if self.initial_distance is not None and self.initial_distance > 0.01:  # Minimum 1cm to avoid division issues
                        progress_ratio = 1.0 - (distance_remaining / self.initial_distance)
                        progress_percent = max(0, min(100, int(progress_ratio * 100)))
                    elif distance_remaining < 0.1:  # Very close to goal (< 10cm)
                        progress_percent = 100
                except (ZeroDivisionError, TypeError) as e:
                    self.get_logger().warn(f'Error calculating progress: {e}, using 0%')
                    progress_percent = 0
                
                # Validate and clamp ETA
                try:
                    eta_seconds = max(0, int(eta_seconds))  # Ensure non-negative integer
                except (ValueError, TypeError):
                    self.get_logger().warn(f'Invalid eta_seconds value: {eta_seconds}, setting to 0')
                    eta_seconds = 0
                
                # Store values for status updates
                self.last_distance_remaining = distance_remaining
                self.last_eta_seconds = eta_seconds
                avg_velocity = self.average_velocity
            
            self.get_logger().debug(
                f' Nav2 feedback: distance_remaining={distance_remaining:.2f}m, '
                f'progress={progress_percent}%, eta={eta_seconds}s, '
                f'avg_velocity={avg_velocity:.2f}m/s' if avg_velocity else 'avg_velocity=N/A'
            )
            
            # Update status publisher with current progress
            # Lock order: _goal_state_lock -> _position_lock (correct order)
            try:
                with self._goal_state_lock:
                    target_id = self.current_target_id
                    command_id = self.current_command_id
                with self._position_lock:
                    current_pos = self.current_position.copy()
                
                self.get_logger().debug(f'Updating status: target_id={target_id}, command_id={command_id}, progress={progress_percent}%, eta={eta_seconds}s')
                self.status_publisher.updateStatus(
                    state=NavigationState.NAVIGATING,
                    target_id=target_id,
                    command_id=command_id,
                    progress_percent=progress_percent,
                    eta_seconds=eta_seconds,
                    current_position=current_pos
                )
                
                # Publish status update
                self.status_publisher.publishStatus()
                self.get_logger().debug(f' Status published: navigating, progress={progress_percent}%')
            except Exception as e:
                # Log with context but don't fail - status publishing failure shouldn't break navigation
                with self._goal_state_lock:
                    cmd_id = self.current_command_id
                    tgt_id = self.current_target_id
                self.error_handler.log_exception(
                    e,
                    {'target_id': tgt_id, 'command_id': cmd_id, 'operation': 'update_status_in_feedback'},
                    cmd_id
                )
                self.get_logger().warn(
                    f'  Error updating/publishing status in feedback callback: {e}'
                )
        
        except Exception as e:
            # Catch-all for any unexpected errors in feedback processing
            # Log with context but don't fail - feedback processing failure shouldn't break navigation
            with self._goal_state_lock:
                cmd_id = self.current_command_id
                tgt_id = self.current_target_id
            self.error_handler.log_exception(
                e,
                {'target_id': tgt_id, 'command_id': cmd_id, 'operation': 'feedback_callback'},
                cmd_id
            )
            self.get_logger().error(
                f'Unexpected error in feedback_callback: {e}',
                exc_info=True
            )
    
    def result_callback(self, future: Any) -> None:
        """
        Handle result from Nav2 with improved error handling.
        
        Processes navigation goal completion (succeeded/aborted/canceled) and
        publishes terminal result events. Maps Nav2 error codes to standardized
        error codes.
        
        Args:
            future: Future containing Nav2 result with status and error information
        """
        self.get_logger().info(' Result callback triggered')
        
        # Get target_id once at the beginning (thread-safe)
        with self._goal_state_lock:
            target_id = self.current_target_id
        
        self.get_logger().debug(f'Processing result for target_id: {target_id}')
        
        try:
            # Get result with error handling
            try:
                future_result = future.result()
                result = future_result.result
                status = future_result.status
                self.get_logger().info(f' Nav2 result received: status={status}, target_id={target_id}')
            except Exception as e:
                self.get_logger().error(
                    f' Failed to get result from future: {e}'
                )
                # Try to update state to error
                try:
                    self.state_manager.onGoalAborted(
                        target_id or 'unknown',
                        'NAV_GOAL_ABORTED',
                        f'Failed to get result: {str(e)}'
                    )
                    self.status_publisher.publishStatus()
                    self.get_logger().info(' Published error status: Failed to get result')
                except Exception as pub_error:
                    self.get_logger().error(f' Failed to publish error status: {pub_error}')
                return
            
            # Handle different result statuses
            try:
                if status == 4:  # SUCCEEDED
                    self.get_logger().info(
                        f' Navigation goal succeeded for target_id: {target_id}'
                    )
                    try:
                        self.command_event_publisher.publish_result(self.current_command_id or 'unknown', target_id, result_type='succeeded')
                    except Exception:
                        pass
                    self.state_manager.onGoalSucceeded(target_id)
                    self.status_publisher.publishStatus()
                    self.get_logger().info(f' Published success status: target_id={target_id}')
                    # State machine: SUCCEEDED → IDLE (after short delay for status publication)
                    try:
                        self._schedule_idle_reset(0.5)
                        self.get_logger().debug('Timer created for state reset to IDLE')
                    except Exception as timer_error:
                        self.get_logger().error(f' Failed to create timer: {timer_error}')
                        # Fallback: reset immediately
                        self.state_manager.resetToIdle()
                elif status == 2:  # CANCELED
                    self.get_logger().info(
                        f'  Navigation goal canceled for target_id: {target_id}'
                    )
                    try:
                        self.command_event_publisher.publish_result(self.current_command_id or 'unknown', target_id, result_type='canceled')
                    except Exception:
                        pass
                    # Cancellation is terminal via RESULT event; status returns to IDLE.
                    self.state_manager.onGoalCanceled(target_id)
                    self.status_publisher.publishStatus()
                    self.get_logger().info(f' Published cancel status (idle): target_id={target_id}')
                else:  # ABORTED or other error status
                    self.get_logger().warn(
                        f' Navigation goal aborted (status={status}) for target_id: {target_id}'
                    )
                    
                    # Extract error_code and error_message from Nav2 result if available
                    nav2_error_code = None
                    nav2_error_message = None
                    if hasattr(result, 'error_code'):
                        nav2_error_code = result.error_code
                    if hasattr(result, 'error_message'):
                        nav2_error_message = result.error_message
                    
                    # Map Nav2 status codes and error codes to normalized error codes
                    error_code = 'NAV_GOAL_ABORTED'
                    error_message = f'Nav2 goal aborted with status {status}'
                    
                    if status == 5:  # REJECTED
                        error_code = 'NAV_GOAL_REJECTED'
                        error_message = 'Nav2 rejected the goal'
                    
                    # Map Nav2 error codes to our error codes
                    if nav2_error_code is not None:
                        # Nav2 error codes (from nav2_msgs/action/NavigateToPose.action)
                        if nav2_error_code == 1:  # UNKNOWN
                            error_code = 'NAV_UNKNOWN_ERROR'
                            error_message = nav2_error_message or 'Unknown navigation error'
                        elif nav2_error_code == 2:  # NO_VALID_CONTROL
                            error_code = 'NAV_NO_VALID_CONTROL'
                            error_message = nav2_error_message or 'No valid control available (base may be blocked or emergency stop active)'
                        elif nav2_error_code == 3:  # FAILED_TO_MAKE_PROGRESS
                            error_code = 'NAV_FAILED_TO_MAKE_PROGRESS'
                            error_message = nav2_error_message or 'Failed to make progress towards goal'
                        elif nav2_error_code == 4:  # PATIENCE_EXCEEDED
                            error_code = 'NAV_PATIENCE_EXCEEDED'
                            error_message = nav2_error_message or 'Patience exceeded while waiting for progress'
                        elif nav2_error_code == 5:  # INVALID_PATH
                            error_code = 'NAV_INVALID_PATH'
                            error_message = nav2_error_message or 'Invalid path to goal'
                        elif nav2_error_code == 6:  # CONTROLLER_TIMED_OUT
                            error_code = 'NAV_CONTROLLER_TIMED_OUT'
                            error_message = nav2_error_message or 'Controller timed out'
                        elif nav2_error_code == 7:  # TF_ERROR
                            error_code = 'NAV_TF_ERROR'
                            error_message = nav2_error_message or 'Transform error'
                        elif nav2_error_code == 8:  # INVALID_CONTROLLER
                            error_code = 'NAV_INVALID_CONTROLLER'
                            error_message = nav2_error_message or 'Invalid controller'
                        else:
                            # Use Nav2 error message if available
                            if nav2_error_message:
                                error_message = nav2_error_message
                    
                    self.get_logger().warn(
                        f' Navigation error details: error_code={error_code}, '
                        f'nav2_error_code={nav2_error_code}, message={error_message}'
                    )
                    
                    self.state_manager.onGoalAborted(
                        target_id,
                        error_code,
                        error_message
                    )
                    try:
                        self.command_event_publisher.publish_result(
                            self.current_command_id or 'unknown', 
                            target_id, 
                            result_type='aborted', 
                            error_code=error_code, 
                            error_message=error_message
                        )
                    except Exception as pub_error:
                        # Log but don't fail - event publishing failure shouldn't break navigation
                        self.get_logger().warn(
                            f'  Failed to publish result event: {pub_error} '
                            f'(command_id: {self.current_command_id}, target_id: {target_id})'
                        )
                    self.status_publisher.publishStatus()
                    self.get_logger().info(f' Published abort status: target_id={target_id}, error_code={error_code}, message={error_message}')
                    # State machine: ABORTED → IDLE (short delay so UI can see aborted)
                    try:
                        self._schedule_idle_reset(1.0)
                    except Exception:
                        self.state_manager.resetToIdle()
            except Exception as state_error:
                # Log with full context for debugging
                self.error_handler.log_exception(
                    state_error,
                    {'target_id': target_id, 'command_id': self.current_command_id, 'operation': 'update_state_in_result'},
                    self.current_command_id
                )
                self.get_logger().error(
                    f' Error updating state in result_callback: {state_error}'
                )
        
        except Exception as e:
            # Catch-all for any unexpected errors
            # Log with full context including traceback
            self.error_handler.log_exception(
                e,
                {'target_id': target_id, 'operation': 'result_callback', 'location': 'outer_catch_all'},
                self.current_command_id if hasattr(self, 'current_command_id') else None
            )
            self.get_logger().error(
                f'Unexpected error in result_callback: {e}',
                exc_info=True
            )
            # Try to update state to error as last resort (graceful degradation)
            try:
                self.state_manager.onGoalAborted(
                    target_id or 'unknown',
                    'NAV_GOAL_ABORTED',
                    f'Unexpected error: {str(e)}'
                )
            except Exception as state_error:
                # Even state update failed - log but don't crash
                self.error_handler.log_exception(
                    state_error,
                    {'target_id': target_id, 'operation': 'update_state_after_error'},
                    None
                )
                self.get_logger().error(
                    f'Failed to update state in result_callback: {state_error}',
                    exc_info=True
                )
            
            # State machine: ERROR → IDLE (after short delay for status publication)
            # Graceful degradation: If timer creation fails, reset immediately
            try:
                self._schedule_idle_reset(1.0)
            except Exception as timer_error:
                # Timer creation failed (e.g., node shutting down) - log and fallback
                self.error_handler.log_exception(
                    timer_error,
                    {'target_id': target_id, 'operation': 'create_timer_for_idle_reset'},
                    None
                )
                self.get_logger().warn(
                    f'  Failed to create timer for idle reset, resetting immediately: {timer_error}'
                )
                # Try to reset to idle immediately if timer creation fails (graceful degradation)
                try:
                    self.state_manager.resetToIdle()
                except Exception as reset_error:
                    # Even immediate reset failed - log but don't crash
                    self.error_handler.log_exception(
                        reset_error,
                        {'target_id': target_id, 'operation': 'immediate_idle_reset'},
                        None
                    )
                    self.get_logger().error(
                        f'Failed to reset to idle after timer error: {reset_error}',
                        exc_info=True
                    )
        
        # Clear goal state
        self.nav_action_client.clear_current_goal()
        with self._goal_state_lock:
            self.current_goal_handle = None
            self.current_target_id = None
            self.current_command_id = None
        
        # Reset progress tracking
        with self._progress_lock:
            self.initial_distance = None
            self.last_distance_remaining = None
            self.last_eta_seconds = None
    
    def cancel_done_callback(self, future):
        """Handle cancel completion with improved error handling"""
        try:
            cancel_response = future.result()
            # Get target_id before clearing (for state update)
            with self._goal_state_lock:
                target_id = self.current_target_id
            
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Goal cancel request accepted')
                # Transition to IDLE after successful cancel
                try:
                    self.state_manager.onGoalCanceled(target_id)
                except Exception as e:
                    self.get_logger().error(f'Error updating state after cancel: {e}')
            else:
                self.get_logger().warn('Goal cancel request rejected')
            
            # Always clear state after cancel (regardless of acceptance)
            # This ensures state consistency
            self.nav_action_client.clear_current_goal()
            with self._goal_state_lock:
                self.current_goal_handle = None
                self.current_target_id = None
                self.current_command_id = None
            # Reset progress tracking
            with self._progress_lock:
                self.initial_distance = None
                self.last_distance_remaining = None
                self.last_eta_seconds = None
                self.last_feedback_time = None
                self.average_velocity = None
        except Exception as e:
            self.get_logger().error(
                f'Error in cancel_done_callback: {e}'
            )
            # Ensure state is cleared even on error
            self.current_goal_handle = None
            self.current_target_id = None
            self.current_command_id = None
            # Try to reset state anyway
            try:
                target_id = getattr(self, '_last_cancel_target_id', None)
                if target_id:
                    self.state_manager.onGoalCanceled(target_id)
            except Exception:
                pass
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for AMCL pose (priority source for current position)"""
        try:
            pose = msg.pose.pose
            self.current_position['x'] = pose.position.x
            self.current_position['y'] = pose.position.y
            # Convert quaternion to theta
            qz = pose.orientation.z
            qw = pose.orientation.w
            self.current_position['theta'] = 2 * math.atan2(qz, qw)
            self.current_position_valid = True
        except Exception as e:
            self.get_logger().warn(f'Error processing AMCL pose: {e}')
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry (fallback source for current position)"""
        # Only use odom if AMCL is not available
        if not self.current_position_valid:
            try:
                pose = msg.pose.pose
                self.current_position['x'] = pose.position.x
                self.current_position['y'] = pose.position.y
                # Convert quaternion to theta
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.current_position['theta'] = 2 * math.atan2(qz, qw)
            except Exception as e:
                self.get_logger().warn(f'Error processing odom: {e}')
    
    def publish_status_periodic(self):
        """Periodic status publishing (called by timer)"""
        # Only publish if status is set (avoid publishing N/A status)
        # Don't overwrite error status - errors are published immediately by error_handler
        if self.status_publisher.current_status:
            # Skip periodic publishing if we're in ERROR state (errors are published immediately)
            if self.status_publisher.current_status != NavigationState.ERROR.value:
                self.status_publisher.publishStatus()
    
    def on_state_change(self, state, target_id, error_code, error_message):
        """Handle state changes and update status publisher"""
        self.get_logger().info(
            f'🔄 Navigation state changed: state={state.value}, target_id={target_id}, '
            f'error_code={error_code}, error_message={error_message}'
        )
        
        # Use stored progress/ETA values if available, otherwise 0
        progress = self.status_publisher.progress_percent if state == NavigationState.NAVIGATING else 0
        eta = self.status_publisher.eta_seconds if state == NavigationState.NAVIGATING else 0
        
        # For non-navigating states, reset progress tracking
        if state != NavigationState.NAVIGATING:
            progress = 0
            eta = 0
        
        # Get command_id from current state (thread-safe)
        with self._goal_state_lock:
            command_id = self.current_command_id
        
        self.get_logger().debug(f'Updating status publisher: progress={progress}%, eta={eta}s, command_id={command_id}')
        self.status_publisher.updateStatus(
            state=state,
            target_id=target_id,
            command_id=command_id,
            progress_percent=progress,
            eta_seconds=eta,
            error_code=error_code,
            error_message=error_message,
            current_position=self.current_position
        )
        
        # Publish immediately on state change
        try:
            self.status_publisher.publishStatus()
            self.get_logger().debug(f' Status published after state change: {state.value}')
        except Exception as e:
            # Log with context - status publishing failure is non-critical but should be logged
            self.error_handler.log_exception(
                e,
                {'state': state.value, 'target_id': target_id, 'command_id': command_id, 'operation': 'publish_status_on_state_change'},
                command_id
            )
            self.get_logger().warn(
                f'  Error publishing status in on_state_change: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationIntegratedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Stop polling
        node.config_provider.stop_polling()
        
        # Disconnect MQTT
        node.mqtt_manager.disconnect()
        
        # Release lock file
        if hasattr(node, '_lock_file'):
            try:
                fcntl.flock(node._lock_file.fileno(), fcntl.LOCK_UN)
                node._lock_file.close()
            except Exception as e:
                node.get_logger().warn(f'Error releasing lock file: {e}')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

