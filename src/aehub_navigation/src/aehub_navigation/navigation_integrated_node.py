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

"""aehub_navigation.navigation_integrated_node

Robot-side integrated navigation node.

Responsibilities
---------------
- Subscribe to MQTT command topics:
    - aroc/robot/{robot_id}/commands/navigateTo
    - aroc/robot/{robot_id}/commands/cancel   (used as STOP)
- Validate commands
- Send Nav2 NavigateToPose goals
- On cancel: stop the robot in-place (cmd_vel zero burst) and cancel Nav2 goal
- Publish:
    - Status: aroc/robot/{robot_id}/status/navigation
    - Command events: aroc/robot/{robot_id}/commands/events (ack/result)

Key architectural fixes vs previous version
------------------------------------------
- MQTT callbacks are not allowed to call ROS APIs directly.
  All MQTT messages are enqueued and processed from a ROS timer.
- Rate limiting is global (not keyed by command_id).
- Command validation matches Canonical Specification v2.0 (`SPECIFICATION.md`).
- Duplicate command_id handling is idempotent (replays last known ack/result).
- Cancel is treated as STOP:
    - Immediately publishes a stop burst to cmd_vel
    - Cancels the active Nav2 goal (if any)

NOTE: Only the main modules are provided per request; scripts/tests/docs are omitted.
"""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlparse

import requests
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from aehub_navigation.broker_config_provider import BrokerConfigProvider, BrokerConfig
from aehub_navigation.command_rate_limiter import CommandRateLimiter
from aehub_navigation.command_validator import CommandValidator
from aehub_navigation.error_handler import NavigationErrorHandler
from aehub_navigation.mqtt_command_event_publisher import MQTTCommandEventPublisher
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager
from aehub_navigation.mqtt_status_publisher import MQTTStatusPublisher
from aehub_navigation.navigation_action_client import NavigationActionClient
from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState
from aehub_navigation.position_registry import PositionRegistry


@dataclass(frozen=True)
class _QueuedCommand:
    topic: str
    payload_raw: str
    received_ts_s: float


class NavigationIntegratedNode(Node):
    def __init__(self):
        super().__init__('navigation_integrated_node')

        # --- Parameters (keep backward compatibility with existing launch file) ---
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('config_service_url', 'http://localhost:7900')
        self.declare_parameter('config_service_api_key', '')
        self.declare_parameter('config_poll_interval', 5.0)
        self.declare_parameter('positions_file', '')

        # Optional tunables
        self.declare_parameter('status_publish_hz', 2.0)
        self.declare_parameter('command_process_hz', 20.0)
        self.declare_parameter('command_rate_limit_s', 0.10)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('stop_burst_duration_s', 0.6)
        self.declare_parameter('stop_burst_rate_hz', 20.0)
        self.declare_parameter('duplicate_cache_ttl_s', 600.0)

        # Pose source (for real vs sim): amcl_pose (default) or odom
        self.declare_parameter('pose_source', 'amcl')  # 'amcl' | 'odom'
        self.declare_parameter('pose_topic', 'amcl_pose')  # topic name without namespace

        # Symovo readiness check (existing params preserved)
        self.declare_parameter('symovo_endpoint', 'https://192.168.1.100')
        self.declare_parameter('amr_id', 0)
        self.declare_parameter('tls_verify', False)
        self.declare_parameter('symovo_ready_check_enabled', False)
        self.declare_parameter('symovo_ready_check_timeout_s', 3.0)
        self.declare_parameter('symovo_ready_check_fail_open', False)
        self.declare_parameter('symovo_auto_enable_drive_mode', False)
        self.declare_parameter('symovo_client_cert_file', '')
        self.declare_parameter('symovo_client_key_file', '')

        # --- Resolved params ---
        self.robot_id = str(self.get_parameter('robot_id').value)
        config_service_url = str(self.get_parameter('config_service_url').value)
        config_service_api_key = str(self.get_parameter('config_service_api_key').value)
        config_poll_interval = float(self.get_parameter('config_poll_interval').value)
        positions_file = str(self.get_parameter('positions_file').value)

        self.status_publish_hz = float(self.get_parameter('status_publish_hz').value)
        self.command_process_hz = float(self.get_parameter('command_process_hz').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.stop_burst_duration_s = float(self.get_parameter('stop_burst_duration_s').value)
        self.stop_burst_rate_hz = float(self.get_parameter('stop_burst_rate_hz').value)
        duplicate_cache_ttl_s = float(self.get_parameter('duplicate_cache_ttl_s').value)
        self.pose_source = str(self.get_parameter('pose_source').value).strip().lower()
        self.pose_topic = str(self.get_parameter('pose_topic').value).strip()

        # Symovo
        self.symovo_endpoint = str(self.get_parameter('symovo_endpoint').value).rstrip('/')
        self.symovo_amr_id = int(self.get_parameter('amr_id').value)
        self.symovo_tls_verify = bool(self.get_parameter('tls_verify').value)
        self.symovo_ready_check_enabled = bool(self.get_parameter('symovo_ready_check_enabled').value)
        self.symovo_ready_check_timeout_s = float(self.get_parameter('symovo_ready_check_timeout_s').value)
        self.symovo_ready_check_fail_open = bool(self.get_parameter('symovo_ready_check_fail_open').value)
        self.symovo_auto_enable_drive_mode = bool(self.get_parameter('symovo_auto_enable_drive_mode').value)
        self.symovo_client_cert_file = str(self.get_parameter('symovo_client_cert_file').value)
        self.symovo_client_key_file = str(self.get_parameter('symovo_client_key_file').value)

        # Optional env overrides for MQTT connectivity (escape hatch)
        # Useful when Config Service points to an unreachable port (e.g. 8883 blocked) but
        # another listener (e.g. 1884) is available.
        self._mqtt_broker_override = str(os.getenv('AEHUB_MQTT_BROKER_OVERRIDE', '')).strip()
        self._mqtt_port_override = int(os.getenv('AEHUB_MQTT_PORT_OVERRIDE', '0') or '0')
        self._mqtt_tls_override = str(os.getenv('AEHUB_MQTT_TLS_OVERRIDE', '')).strip().lower()  # 'true'|'false'|''
        self._mqtt_tls_insecure_override = str(os.getenv('AEHUB_MQTT_TLS_INSECURE_OVERRIDE', '')).strip().lower()

        if not config_service_api_key:
            self.get_logger().warn('config_service_api_key is empty. If Config Service requires auth, broker fetch will fail.')

        self.get_logger().info(
            f'AE.HUB NavigationIntegratedNode starting: robot_id={self.robot_id}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, status_hz={self.status_publish_hz}, process_hz={self.command_process_hz}'
        )

        # --- Single-instance lock (prevents two nodes controlling the same base) ---
        self._lock_fd = None
        self._acquire_single_instance_lock()

        # --- Core components ---
        self.position_registry = PositionRegistry()
        if positions_file:
            self.position_registry.loadFromYAML(positions_file)
        else:
            self.position_registry.loadFromYAML('')  # triggers default path resolution

        self.command_validator = CommandValidator(ttl_seconds=int(duplicate_cache_ttl_s))
        self.rate_limiter = CommandRateLimiter(min_interval_seconds=float(self.get_parameter('command_rate_limit_s').value))

        self.state_manager = NavigationStateManager(logger=self.get_logger())

        self.mqtt_manager = MQTTConnectionManager(
            logger=self.get_logger(),
            config_service_url=config_service_url,
            api_key=config_service_api_key,
        )
        self.mqtt_manager.on_connect_callback = self._on_mqtt_connect
        self.mqtt_manager.on_message_callback = self._on_mqtt_message

        self.status_publisher = MQTTStatusPublisher()
        self.status_publisher.set_mqtt_manager(self.mqtt_manager)
        self.status_publisher.set_robot_id(self.robot_id)

        self.event_publisher = MQTTCommandEventPublisher()
        self.event_publisher.set_mqtt_manager(self.mqtt_manager)
        self.event_publisher.set_robot_id(self.robot_id)

        self.error_handler = NavigationErrorHandler(logger=self.get_logger(), status_publisher=self.status_publisher)

        self.nav_action_client = NavigationActionClient(self, action_name='navigate_to_pose')
        self.nav_action_client.set_goal_response_callback(self._on_nav_goal_response)
        self.nav_action_client.set_feedback_callback(self._on_nav_feedback)
        self.nav_action_client.set_result_callback(self._on_nav_result)

        # --- Robot position snapshot ---
        self._pose_lock = threading.Lock()
        self._current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # --- Progress snapshot ---
        self._progress_lock = threading.Lock()
        self._goal_initial_distance_m: Optional[float] = None
        self._distance_remaining_m: Optional[float] = None
        self._eta_seconds: int = 0

        # --- Command / goal tracking ---
        self._active_goal_lock = threading.Lock()
        self._active_target_id: Optional[str] = None
        self._active_command_id: Optional[str] = None

        # For idempotency and duplicate replay
        self._outcome_lock = threading.Lock()
        self._outcomes: Dict[str, Dict[str, Any]] = {}

        # --- STOP burst ---
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._stop_timer = None
        self._stop_remaining_msgs = 0

        # --- Subscriptions (pose) ---
        if self.pose_source == 'odom':
            self.create_subscription(Odometry, self.pose_topic or 'odom', self._on_odom_pose, 10)
        else:
            # Default: AMCL pose in map frame
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic or 'amcl_pose', self._on_amcl_pose, 10)

        # --- MQTT command queue ---
        self._cmd_queue: queue.Queue[_QueuedCommand] = queue.Queue(maxsize=200)

        # --- Config provider (polling thread) ---
        self.config_provider = BrokerConfigProvider(
            config_service_url=config_service_url,
            api_key=config_service_api_key,
            logger=self.get_logger(),
        )
        self.config_provider.set_polling_interval(config_poll_interval)
        self.config_provider.add_change_callback(self._on_broker_config_changed)

        # Initial config fetch & connect
        initial_cfg = self.config_provider.fetch_config()
        if initial_cfg is None:
            self.get_logger().error('Failed to fetch initial broker config. MQTT will stay disconnected until config becomes available.')
        else:
            self._connect_mqtt(initial_cfg)

        self.config_provider.start_polling()

        # --- Timers ---
        self._process_timer = self.create_timer(1.0 / max(self.command_process_hz, 1.0), self._process_command_queue)
        self._status_timer = self.create_timer(1.0 / max(self.status_publish_hz, 0.2), self._publish_periodic_status)

        # Start in idle
        self.state_manager.setState(NavigationState.IDLE)

    def _apply_mqtt_overrides(self, cfg: BrokerConfig) -> BrokerConfig:
        broker = cfg.broker
        port = cfg.broker_port
        use_tls = cfg.mqtt_use_tls
        tls_insecure = cfg.mqtt_tls_insecure

        changed = False
        if self._mqtt_broker_override:
            broker = self._mqtt_broker_override
            changed = True
        if self._mqtt_port_override > 0:
            port = int(self._mqtt_port_override)
            changed = True
        if self._mqtt_tls_override in {'true', 'false'}:
            use_tls = (self._mqtt_tls_override == 'true')
            changed = True
        if self._mqtt_tls_insecure_override in {'true', 'false'}:
            tls_insecure = (self._mqtt_tls_insecure_override == 'true')
            changed = True

        if changed:
            self.get_logger().warn(
                f'Applying MQTT overrides: broker={broker}:{port} tls={use_tls} insecure={tls_insecure}'
            )

        return BrokerConfig(
            broker=broker,
            broker_port=port,
            mqtt_user=cfg.mqtt_user,
            mqtt_password=cfg.mqtt_password,
            mqtt_use_tls=use_tls,
            mqtt_tls_insecure=tls_insecure,
        )

    # ---------------------------------------------------------------------
    # Single-instance guard
    # ---------------------------------------------------------------------

    def _acquire_single_instance_lock(self) -> None:
        lock_path = f'/tmp/aehub_navigation_{self.robot_id}.lock'
        try:
            self._lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
            try:
                import fcntl
                fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except Exception:
                self.get_logger().error(f'Another navigation_integrated_node instance appears to be running (lock: {lock_path}).')
                raise SystemExit(2)
        except Exception:
            self.get_logger().error(f'Failed to acquire instance lock: {lock_path}\n{traceback.format_exc()}')
            raise

    # ---------------------------------------------------------------------
    # ROS callbacks
    # ---------------------------------------------------------------------

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        with self._pose_lock:
            self._current_pose = {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'theta': float(yaw),
            }

    def _on_odom_pose(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        with self._pose_lock:
            self._current_pose = {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'theta': float(yaw),
            }

    # ---------------------------------------------------------------------
    # MQTT connection & callbacks
    # ---------------------------------------------------------------------

    def _connect_mqtt(self, cfg: BrokerConfig) -> None:
        ok = self.mqtt_manager.connect(self._apply_mqtt_overrides(cfg))
        if not ok:
            self.get_logger().error('MQTT connect failed')
            return

        base = f'aroc/robot/{self.robot_id}/commands'
        self.mqtt_manager.subscribe(f'{base}/navigateTo', qos=1)
        self.mqtt_manager.subscribe(f'{base}/cancel', qos=1)

    def _on_broker_config_changed(self, new_cfg: BrokerConfig) -> None:
        # Called from polling thread.
        try:
            self.get_logger().info('Broker config changed, reconnecting MQTT...')
            self.mqtt_manager.reconnect(self._apply_mqtt_overrides(new_cfg))
            base = f'aroc/robot/{self.robot_id}/commands'
            self.mqtt_manager.subscribe(f'{base}/navigateTo', qos=1)
            self.mqtt_manager.subscribe(f'{base}/cancel', qos=1)
        except Exception:
            self.get_logger().error(f'Failed to reconnect MQTT on config change:\n{traceback.format_exc()}')

    def _on_mqtt_connect(self, _client, _userdata, _flags, rc: int) -> None:
        if rc == 0:
            self.get_logger().info('MQTT connected')
        else:
            self.get_logger().warn(f'MQTT connection failed rc={rc}')

    def _on_mqtt_message(self, _client, _userdata, msg) -> None:
        # Runs on paho network thread. Must be fast and must not touch ROS.
        try:
            payload_raw = msg.payload.decode('utf-8', errors='replace')
            item = _QueuedCommand(topic=str(msg.topic), payload_raw=payload_raw, received_ts_s=time.time())
            try:
                self._cmd_queue.put_nowait(item)
            except queue.Full:
                # Drop oldest to remain responsive.
                _ = self._cmd_queue.get_nowait()
                self._cmd_queue.put_nowait(item)
        except Exception:
            # Best effort: never raise from callback.
            return

    # ---------------------------------------------------------------------
    # Main processing loop
    # ---------------------------------------------------------------------

    def _process_command_queue(self) -> None:
        # Called on ROS executor thread (timer callback)
        processed = 0
        while processed < 5:  # bound work per tick
            try:
                item = self._cmd_queue.get_nowait()
            except queue.Empty:
                return

            processed += 1
            self._handle_one_command(item)

    def _handle_one_command(self, item: _QueuedCommand) -> None:
        topic = item.topic

        # Determine command kind from topic
        if topic.endswith('/navigateTo'):
            kind = 'navigateTo'
        elif topic.endswith('/cancel'):
            kind = 'cancel'
        else:
            return

        # Parse JSON
        try:
            cmd = json.loads(item.payload_raw) if item.payload_raw else {}
        except Exception:
            self.get_logger().warn(f'Received non-JSON command on {topic}: {item.payload_raw[:200]}')
            return

        command_id = str(cmd.get('command_id') or '')
        target_id = cmd.get('target_id') if isinstance(cmd.get('target_id'), str) else None

        # Emit immediate ack=received (idempotent replay handled below)
        if command_id:
            self.event_publisher.publish_ack(command_id, target_id, ack_status='received')

        # Idempotency / duplicate replay
        if command_id and self.command_validator.is_duplicate(command_id):
            self._replay_duplicate(command_id)
            return

        # Validate
        ok, err_code, err_msg = self.command_validator.validate_command_detailed(
            cmd,
            command_type=kind,
            position_registry=self.position_registry,
            expected_robot_id=self.robot_id,
        )
        if not ok:
            if command_id:
                self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                                      target_id=target_id, error_code=err_code, error_message=err_msg)
                self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=err_msg)
                self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                                    error_code=err_code, error_message=err_msg)
            self.state_manager.setState(NavigationState.ERROR, target_id=target_id, error_code=err_code, error_message=err_msg)
            return

        # Rate limiting: only for navigateTo (cancel must remain responsive)
        if kind == 'navigateTo':
            allowed, rl_msg = self.rate_limiter.check_rate_limit(bucket='navigate')
            if not allowed:
                if command_id:
                    self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                                          target_id=target_id, error_code='NAV_RATE_LIMIT_EXCEEDED', error_message=rl_msg)
                    self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=rl_msg)
                    self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                                        error_code='NAV_RATE_LIMIT_EXCEEDED', error_message=rl_msg)
                return

        # Dispatch
        if kind == 'cancel':
            self._handle_cancel(cmd)
        else:
            self._handle_navigate(cmd)

    # ---------------------------------------------------------------------
    # Command handlers
    # ---------------------------------------------------------------------

    def _handle_cancel(self, cmd: Dict[str, Any]) -> None:
        command_id = str(cmd.get('command_id'))

        # Cancel is treated as STOP.
        # 1) stop burst immediately
        self._start_stop_burst()

        # 2) cancel active Nav2 goal if any
        with self._active_goal_lock:
            active_cmd = self._active_command_id
            active_target = self._active_target_id

        if self.nav_action_client.is_goal_active():
            self.state_manager.onCancelRequested(target_id=active_target)
            self.nav_action_client.cancel_goal()

            # For the active navigate command, publish canceled terminal result (idempotently).
            if active_cmd:
                self._publish_terminal_once(active_cmd, active_target, result_status='canceled')

        # 3) acknowledge the cancel command itself as succeeded
        self._remember_outcome(command_id, ack_status='accepted', result_status='succeeded', target_id=None)
        self.event_publisher.publish_ack(command_id, None, ack_status='accepted')
        self.event_publisher.publish_result(command_id, None, result_status='succeeded')

        # 4) reset state to idle
        self._set_active_goal(None, None)
        self._reset_progress()
        self.state_manager.setState(NavigationState.IDLE)
        self.command_validator.mark_processed(command_id)

    def _handle_navigate(self, cmd: Dict[str, Any]) -> None:
        command_id = str(cmd.get('command_id'))
        target_id = cmd.get('target_id') if isinstance(cmd.get('target_id'), str) else None

        # Optional readiness check (Symovo)
        if self.symovo_ready_check_enabled:
            ok, reason, details = self._check_symovo_drive_ready()
            if not ok:
                if self.symovo_auto_enable_drive_mode:
                    ok2, reason2, details2 = self._try_enable_symovo_drive_mode()
                    if not ok2:
                        reason = f"{reason}; auto_enable_failed={reason2}"
                        details.update({"auto_enable": details2})
                    else:
                        # re-check
                        ok, reason, details = self._check_symovo_drive_ready()

            if not ok:
                # Fail-open option
                if self.symovo_ready_check_fail_open:
                    self.get_logger().warn(f"Symovo readiness check failed-open: {reason} details={details}")
                else:
                    self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                                          target_id=target_id, error_code='NAV_ROBOT_NOT_READY', error_message=reason)
                    self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=reason)
                    self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                                        error_code='NAV_ROBOT_NOT_READY', error_message=reason)
                    self.command_validator.mark_processed(command_id)
                    return

        pose = self._pose_from_command(cmd)
        if pose is None:
            err = 'Invalid target: cannot resolve target pose'
            self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                                  target_id=target_id, error_code='NAV_INVALID_TARGET', error_message=err)
            self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=err)
            self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                                error_code='NAV_INVALID_TARGET', error_message=err)
            self.command_validator.mark_processed(command_id)
            return

        # Set active goal tracking *optimistically*; goal could still be rejected.
        # We publish ack=accepted only after goal acceptance.
        self._set_active_goal(command_id, target_id)

        # Compute initial distance for progress estimation
        with self._pose_lock:
            cur = dict(self._current_pose)
        goal_x = float(pose.pose.position.x)
        goal_y = float(pose.pose.position.y)
        init_dist = math.hypot(goal_x - cur['x'], goal_y - cur['y'])
        with self._progress_lock:
            self._goal_initial_distance_m = init_dist if init_dist > 0.01 else None
            self._distance_remaining_m = init_dist
            self._eta_seconds = 0

        sent = self.nav_action_client.send_goal(pose, target_id=target_id or 'pose', command_id=command_id)
        if not sent:
            err = 'Nav2 action server not available'
            self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                                  target_id=target_id, error_code='NAV_SERVER_UNAVAILABLE', error_message=err)
            self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=err)
            self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                                error_code='NAV_SERVER_UNAVAILABLE', error_message=err)
            self._set_active_goal(None, None)
            self.command_validator.mark_processed(command_id)
            return

        # state becomes navigating when goal is accepted (see _on_nav_goal_response)

    # ---------------------------------------------------------------------
    # Nav2 callbacks
    # ---------------------------------------------------------------------

    def _on_nav_goal_response(self, future, target_id: Optional[str], command_id: Optional[str]) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            self.get_logger().error(f'Goal response future failed:\n{traceback.format_exc()}')
            if command_id:
                self._publish_rejected(command_id, target_id, 'NAV_GOAL_REJECTED', 'Goal response future failed')
            return

        if not goal_handle.accepted:
            if command_id:
                self._publish_rejected(command_id, target_id, 'NAV_GOAL_REJECTED', 'Nav2 rejected goal')
                self.command_validator.mark_processed(command_id)
            self._set_active_goal(None, None)
            self.state_manager.setState(NavigationState.IDLE)
            return

        # Goal accepted
        if command_id:
            self._remember_outcome(command_id, ack_status='accepted', result_status=None, target_id=target_id)
            self.event_publisher.publish_ack(command_id, target_id, ack_status='accepted')

        self.nav_action_client.set_current_goal(goal_handle, target_id=target_id or 'pose', command_id=command_id)
        self.state_manager.onGoalSent(target_id=target_id or 'pose')

    def _on_nav_feedback(self, feedback_msg) -> None:
        try:
            fb = feedback_msg.feedback
            dist = getattr(fb, 'distance_remaining', None)
            if dist is not None:
                with self._progress_lock:
                    self._distance_remaining_m = float(dist)

            eta = getattr(fb, 'estimated_time_remaining', None)
            if eta is not None and hasattr(eta, 'sec'):
                with self._progress_lock:
                    self._eta_seconds = int(getattr(eta, 'sec', 0))
        except Exception:
            # Do not crash on feedback parsing.
            return

    def _on_nav_result(self, future) -> None:
        # Called when result future completes.
        with self._active_goal_lock:
            command_id = self._active_command_id
            target_id = self._active_target_id

        try:
            res = future.result()
            status = res.status
            result = res.result
        except Exception:
            self.get_logger().error(f'Nav2 result future failed:\n{traceback.format_exc()}')
            if command_id:
                self._publish_terminal_once(command_id, target_id, result_status='error',
                                            error_code='NAV_UNKNOWN_ERROR', error_message='Result future failed')
                self.command_validator.mark_processed(command_id)
            self._set_active_goal(None, None)
            self.state_manager.setState(NavigationState.ERROR, target_id=target_id, error_code='NAV_UNKNOWN_ERROR',
                                       error_message='Result future failed')
            return

        # Nav2 GoalStatus codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:
            if command_id:
                self._publish_terminal_once(command_id, target_id, result_status='succeeded')
                self.command_validator.mark_processed(command_id)
            self.state_manager.onGoalSucceeded(target_id=target_id or 'pose')

        elif status == 5:
            if command_id:
                self._publish_terminal_once(command_id, target_id, result_status='canceled')
                self.command_validator.mark_processed(command_id)
            self.state_manager.onGoalCanceled(target_id=target_id or 'pose')

        elif status == 6:
            # Attempt to map Nav2 error codes, but keep it simple
            err_code = 'NAV_GOAL_ABORTED'
            err_msg = f'Nav2 aborted goal (result: {type(result).__name__})'
            if command_id:
                self._publish_terminal_once(command_id, target_id, result_status='aborted',
                                            error_code=err_code, error_message=err_msg)
                self.command_validator.mark_processed(command_id)
            self.state_manager.onGoalAborted(target_id=target_id or 'pose', error_code=err_code, error_message=err_msg)

        else:
            # Unknown terminal
            err_code = 'NAV_UNKNOWN_ERROR'
            err_msg = f'Unknown Nav2 result status={status}'
            if command_id:
                self._publish_terminal_once(command_id, target_id, result_status='error',
                                            error_code=err_code, error_message=err_msg)
                self.command_validator.mark_processed(command_id)
            self.state_manager.setState(NavigationState.ERROR, target_id=target_id, error_code=err_code, error_message=err_msg)

        self._set_active_goal(None, None)
        self._reset_progress()

    # ---------------------------------------------------------------------
    # Status publishing
    # ---------------------------------------------------------------------

    def _publish_periodic_status(self) -> None:
        # Snapshot state
        state = self.state_manager.getState()
        target_id = self.state_manager.getTargetId()

        with self._active_goal_lock:
            command_id = self._active_command_id

        with self._pose_lock:
            pose = dict(self._current_pose)

        progress_percent, eta_seconds = self._compute_progress()

        err_code, err_msg = self.state_manager.getError()

        self.status_publisher.updateStatus(
            state=state,
            target_id=target_id,
            progress_percent=int(progress_percent),
            eta_seconds=int(eta_seconds),
            error_code=err_code,
            error_message=err_msg,
            current_position=pose,
            command_id=command_id,
        )
        self.status_publisher.publishStatus()

        # Auto-return from SUCCEEDED to IDLE after 2 status ticks to avoid sticky "arrived" forever.
        if state == NavigationState.SUCCEEDED:
            # Transition after 1 second approx.
            self.state_manager.resetToIdle()

    def _compute_progress(self) -> Tuple[int, int]:
        with self._progress_lock:
            init = self._goal_initial_distance_m
            remaining = self._distance_remaining_m
            eta = self._eta_seconds

        if init is None or remaining is None or init <= 0.01:
            return 0, eta

        pct = 100.0 * max(0.0, min(1.0, 1.0 - (remaining / init)))
        return int(pct), int(eta)

    # ---------------------------------------------------------------------
    # STOP burst (cmd_vel zeros)
    # ---------------------------------------------------------------------

    def _start_stop_burst(self) -> None:
        # Cancel existing timer
        if self._stop_timer is not None:
            try:
                self._stop_timer.cancel()
            except Exception:
                pass
            self._stop_timer = None

        total_msgs = int(max(1.0, self.stop_burst_duration_s * self.stop_burst_rate_hz))
        self._stop_remaining_msgs = total_msgs

        period = 1.0 / max(self.stop_burst_rate_hz, 1.0)
        self._stop_timer = self.create_timer(period, self._stop_tick)

    def _stop_tick(self) -> None:
        try:
            self._cmd_vel_pub.publish(Twist())
        except Exception:
            pass

        self._stop_remaining_msgs -= 1
        if self._stop_remaining_msgs <= 0:
            try:
                self._stop_timer.cancel()
            except Exception:
                pass
            self._stop_timer = None

    # ---------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------

    def _set_active_goal(self, command_id: Optional[str], target_id: Optional[str]) -> None:
        with self._active_goal_lock:
            self._active_command_id = command_id
            self._active_target_id = target_id

    def _reset_progress(self) -> None:
        with self._progress_lock:
            self._goal_initial_distance_m = None
            self._distance_remaining_m = None
            self._eta_seconds = 0

    def _pose_from_command(self, cmd: Dict[str, Any]):
        # target_id -> positions.yaml
        target_id = cmd.get('target_id') if isinstance(cmd.get('target_id'), str) else None
        if target_id:
            pose = self.position_registry.getPosition(target_id)
            return pose

        # direct pose
        if 'x' in cmd and 'y' in cmd:
            try:
                x = float(cmd.get('x'))
                y = float(cmd.get('y'))
                theta = float(cmd.get('theta') or 0.0)
            except (TypeError, ValueError):
                return None

            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            return pose

        return None

    def _publish_rejected(self, command_id: str, target_id: Optional[str], error_code: str, error_message: str) -> None:
        self._remember_outcome(command_id, ack_status='rejected', result_status='error',
                              target_id=target_id, error_code=error_code, error_message=error_message)
        self.event_publisher.publish_ack(command_id, target_id, ack_status='rejected', reason=error_message)
        self.event_publisher.publish_result(command_id, target_id, result_status='error',
                                            error_code=error_code, error_message=error_message)

    def _publish_terminal_once(
        self,
        command_id: str,
        target_id: Optional[str],
        result_status: str,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> None:
        with self._outcome_lock:
            oc = self._outcomes.get(command_id) or {}
            if oc.get('terminal_result') is not None:
                return

        self._remember_outcome(
            command_id,
            ack_status=oc.get('ack_status') if 'oc' in locals() else None,
            result_status=result_status,
            target_id=target_id,
            error_code=error_code,
            error_message=error_message,
        )
        self.event_publisher.publish_result(command_id, target_id, result_status=result_status,
                                            error_code=error_code, error_message=error_message)

    def _remember_outcome(
        self,
        command_id: str,
        *,
        ack_status: Optional[str],
        result_status: Optional[str],
        target_id: Optional[str],
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
    ) -> None:
        now = time.time()
        with self._outcome_lock:
            cur = self._outcomes.get(command_id) or {}
            if ack_status is not None:
                cur['ack_status'] = ack_status
            if result_status is not None:
                cur['terminal_result'] = result_status
            cur['target_id'] = target_id
            if error_code is not None:
                cur['error_code'] = error_code
            if error_message is not None:
                cur['error_message'] = error_message
            cur['ts'] = now
            self._outcomes[command_id] = cur

            # prune
            ttl = float(self.get_parameter('duplicate_cache_ttl_s').value)
            cutoff = now - ttl
            for cid in list(self._outcomes.keys()):
                if self._outcomes[cid].get('ts', 0.0) < cutoff:
                    self._outcomes.pop(cid, None)

    def _replay_duplicate(self, command_id: str) -> None:
        with self._outcome_lock:
            oc = self._outcomes.get(command_id)

        if not oc:
            # unknown duplicate: accept and ignore
            return

        target_id = oc.get('target_id')
        if oc.get('ack_status'):
            # Replay last ack.
            self.event_publisher.publish_ack(command_id, target_id, ack_status=str(oc.get('ack_status')))

        if oc.get('terminal_result'):
            self.event_publisher.publish_result(
                command_id,
                target_id,
                result_status=str(oc.get('terminal_result')),
                error_code=oc.get('error_code'),
                error_message=oc.get('error_message'),
            )

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        # yaw (z-axis rotation) from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------------------------------------------------------------
    # Symovo readiness
    # ---------------------------------------------------------------------

    def _fetch_symovo_agv_item(self) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        url = f"{self.symovo_endpoint}/v0/agv"
        try:
            res = requests.get(url, verify=self.symovo_tls_verify, timeout=self.symovo_ready_check_timeout_s)
            if res.status_code != 200:
                return None, {"url": url, "reason": "symovo_http_error", "status": res.status_code}
            data = res.json()
            if not isinstance(data, list):
                return None, {"url": url, "reason": "symovo_invalid_response", "type": type(data).__name__}
            for item in data:
                if isinstance(item, dict) and item.get('id') == self.symovo_amr_id:
                    return item, None
            return None, {"url": url, "reason": "amr_id_not_found", "amr_id": self.symovo_amr_id}
        except Exception as e:
            return None, {"url": url, "reason": "symovo_unreachable", "error": str(e)}

    def _check_symovo_drive_ready(self) -> Tuple[bool, str, Dict[str, Any]]:
        item, err = self._fetch_symovo_agv_item()
        if err:
            return False, err.get('reason', 'symovo_unreachable'), err

        # Conservative criteria: must be in auto and drive-ready and not charging.
        auto_mode = bool(item.get('auto_mode', True))
        drive_ready = bool(item.get('drive_ready', True))
        charging = bool(item.get('charging', False))
        manual = bool(item.get('manual_mode', False))

        ok = (auto_mode and drive_ready and not charging and not manual)
        details = {
            'auto_mode': auto_mode,
            'drive_ready': drive_ready,
            'charging': charging,
            'manual_mode': manual,
        }
        if ok:
            return True, 'ok', details
        return False, 'robot_not_drive_ready', details

    def _try_enable_symovo_drive_mode(self) -> Tuple[bool, str, Dict[str, Any]]:
        # Best effort: call common endpoints used in older code.
        parsed = urlparse(self.symovo_endpoint)
        if parsed.scheme not in ('http', 'https'):
            return False, 'invalid_endpoint', {'endpoint': self.symovo_endpoint}

        attempts = []
        url = f"{self.symovo_endpoint}/v0/agv/{self.symovo_amr_id}/move/drive_mode"

        cert = None
        if self.symovo_client_cert_file and self.symovo_client_key_file:
            cert = (self.symovo_client_cert_file, self.symovo_client_key_file)

        try:
            res = requests.post(url, verify=self.symovo_tls_verify, timeout=self.symovo_ready_check_timeout_s, cert=cert)
            if res.status_code in (200, 204):
                return True, 'ok', {'url': url, 'status': res.status_code}
            attempts.append({'url': url, 'status': res.status_code, 'body': res.text[:200]})
        except Exception as e:
            attempts.append({'url': url, 'error': str(e)})

        return False, 'enable_drive_mode_failed', {'attempts': attempts}


def main(args=None):
    rclpy.init(args=args)
    node = NavigationIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
