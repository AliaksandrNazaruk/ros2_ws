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
Nav2LocalizationOrchestrator (Application layer for localization bringup).

Responsibilities:
- subscribe to MapStatus from map mirror
- when map changes: call nav2_map_server LoadMap service
- after successful load: publish /initialpose based on Symovo pose, using configured geometry policy
- publish health

Non-responsibilities:
- do not implement navigation capability
- do not publish /cmd_vel
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from aehub_msgs.msg import MapStatus

from aehub_symovo_map_mirror.symovo_api_client import SymovoApiClient
from aehub_symovo_map_mirror.symovo_map_geometry import SymovoMapGeometry


# region agent log
def _dbg_log(*, hypothesis_id: str, location: str, message: str, data: dict) -> None:
    try:
        log_path = "/home/boris/ros2_ws/.cursor/debug.log"
        payload = {
            "sessionId": "debug-session",
            "runId": os.environ.get("AEHUB_DEBUG_RUN_ID", "run1"),
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
    except Exception:
        # Best-effort fallback if workspace log path is not writable/available.
        try:
            with open("/tmp/aehub_debug_fallback.log", "a", encoding="utf-8") as f:
                f.write(json.dumps({"location": location, "message": message, "hypothesisId": hypothesis_id, "timestamp": int(time.time() * 1000)}) + "\n")
        except Exception:
            pass
# endregion


def _yaw_to_quat(yaw: float):
    # z-only rotation
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Nav2LocalizationOrchestratorNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("nav2_localization_orchestrator")
        # Allow nested spinning within subscription callback for LoadMap response handling.
        # (spin_until_future_complete is used inside _on_map_status → _reload_map_server)
        self._reentrant_group = ReentrantCallbackGroup()

        # Inputs
        self.declare_parameter("map_status_topic", "infra/map/status")

        # Symovo pose source
        self.declare_parameter("symovo_endpoint", "https://192.168.1.100")
        self.declare_parameter("amr_id", 15)
        self.declare_parameter("tls_verify", False)

        # Nav2 map_server reload service
        self.declare_parameter("map_server_load_map_service", "map_server/load_map")
        self.declare_parameter("load_map_timeout_sec", 5.0)

        # Initial pose publish
        self.declare_parameter("initialpose_topic", "initialpose")
        self.declare_parameter("initialpose_cov_xy", 0.25)
        self.declare_parameter("initialpose_cov_yaw", 0.0685)  # ~15deg^2

        self._client: Optional[SymovoApiClient] = None
        self._map_status_sub = None
        self._initialpose_pub = None
        self._diag_pub = None
        self._load_map_client = None

        self._mutex = threading.Lock()
        self._last_map_revision: Optional[int] = None
        self._last_map_yaml: str = ""
        self._last_error: str = ""
        self._last_ok_ts: float = 0.0
        self._is_active: bool = False
        # Cache last MapStatus received while INACTIVE.
        # MapStatus is often published as transient_local by the map mirror, meaning
        # we may receive a latched snapshot immediately after subscription creation
        # (during on_configure). If we ignore it, we may never see it again unless
        # the map changes. To avoid missing the initial MapStatus→LoadMap→initialpose
        # chain, we stash the latest snapshot and replay it on activation.
        self._pending_map_status: Optional[MapStatus] = None

    def _require_relative_name(self, name: str, *, param_name: str) -> str:
        n = str(name or "")
        if not n:
            raise RuntimeError(f"invalid {param_name}: must be non-empty")
        if n.startswith("/"):
            raise RuntimeError(f"invalid {param_name}: must be relative (no leading '/'): {n}")
        return n

    # ---------------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        tls_verify = bool(self.get_parameter("tls_verify").value)
        # region agent log
        _dbg_log(
            hypothesis_id="H1",
            location="nav2_localization_orchestrator_node.py:on_configure",
            message="configure_enter",
            data={
                "namespace": self.get_namespace(),
                "symovo_endpoint": endpoint,
                "tls_verify": tls_verify,
                "amr_id": int(self.get_parameter("amr_id").value),
            },
        )
        # endregion
        self._client = SymovoApiClient(endpoint, tls_verify=tls_verify, timeout_sec=10.0)

        # Subscribe as TRANSIENT_LOCAL so we immediately get the latest MapStatus snapshot
        # from the map mirror (which publishes latched status).
        qos_lat = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_status_topic = self._require_relative_name(
            str(self.get_parameter("map_status_topic").value),
            param_name="map_status_topic",
        )
        initialpose_topic = self._require_relative_name(
            str(self.get_parameter("initialpose_topic").value),
            param_name="initialpose_topic",
        )
        srv_name = self._require_relative_name(
            str(self.get_parameter("map_server_load_map_service").value),
            param_name="map_server_load_map_service",
        )
        # region agent log
        _dbg_log(
            hypothesis_id="H1",
            location="nav2_localization_orchestrator_node.py:on_configure",
            message="interfaces_resolved",
            data={
                "map_status_topic": map_status_topic,
                "initialpose_topic": initialpose_topic,
                "map_server_load_map_service": srv_name,
                "load_map_timeout_sec": float(self.get_parameter("load_map_timeout_sec").value),
            },
        )
        # endregion
        self._map_status_sub = self.create_subscription(
            MapStatus, map_status_topic, self._on_map_status, qos_lat, callback_group=self._reentrant_group
        )

        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, initialpose_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "health/nav2_localization_orchestrator", 10)

        self._load_map_client = self.create_client(LoadMap, srv_name, callback_group=self._reentrant_group)

        self._last_map_revision = None
        self._last_map_yaml = ""
        self._last_error = ""
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        # region agent log
        _dbg_log(
            hypothesis_id="H2",
            location="nav2_localization_orchestrator_node.py:on_activate",
            message="activated",
            data={"namespace": self.get_namespace()},
        )
        # endregion
        # Replay the most recent MapStatus snapshot (if any) now that we're ACTIVE.
        pending = self._pending_map_status
        if pending is not None:
            def _replay():
                try:
                    # Small delay to allow service discovery (map_server/load_map) after activation.
                    time.sleep(0.2)
                    self._on_map_status(pending)
                except Exception:
                    pass

            threading.Thread(target=_replay, daemon=True).start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._client = None
        self._last_map_revision = None
        self._last_map_yaml = ""
        self._last_error = ""
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------------
    # Map status handling
    # ---------------------------------------------------------------------

    def _on_map_status(self, msg: MapStatus) -> None:
        with self._mutex:
            # Lifecycle gating invariant: only act while ACTIVE.
            if not self._is_active:
                # Store the latest snapshot so it can be replayed on activation.
                self._pending_map_status = msg
                # region agent log
                _dbg_log(
                    hypothesis_id="H2",
                    location="nav2_localization_orchestrator_node.py:_on_map_status",
                    message="ignored_not_active",
                    data={"ready": bool(msg.ready), "revision": int(msg.revision), "yaml_path": str(msg.yaml_path)},
                )
                # endregion
                return
            # Clear pending snapshot once we start processing in ACTIVE state.
            self._pending_map_status = None
            # region agent log
            _dbg_log(
                hypothesis_id="H3",
                location="nav2_localization_orchestrator_node.py:_on_map_status",
                message="map_status_received",
                data={
                    "ready": bool(msg.ready),
                    "revision": int(msg.revision),
                    "yaml_path": str(msg.yaml_path),
                    "pose_transform_mode": str(msg.pose_transform_mode),
                    "origin_mode": str(msg.origin_mode),
                },
            )
            # endregion
            if not msg.ready:
                self._last_error = msg.error or "map_not_ready"
                self._publish_health(ok=False, error=self._last_error)
                return

            # Dedup by (revision, yaml_path)
            if self._last_map_revision == int(msg.revision) and self._last_map_yaml == str(msg.yaml_path):
                # region agent log
                _dbg_log(
                    hypothesis_id="H3",
                    location="nav2_localization_orchestrator_node.py:_on_map_status",
                    message="dedup_skip",
                    data={"revision": int(msg.revision), "yaml_path": str(msg.yaml_path)},
                )
                # endregion
                self._publish_health(ok=True, error="")
                return

            # Reload map_server
            ok, err = self._reload_map_server(map_yaml=str(msg.yaml_path))
            if not ok:
                self._last_error = err or "load_map_failed"
                self._publish_health(ok=False, error=self._last_error)
                return

            # Fetch pose from Symovo and publish /initialpose using MapStatus geometry policy
            ok2, err2 = self._publish_initialpose_from_symovo(msg)
            if not ok2:
                self._last_error = err2 or "initialpose_failed"
                self._publish_health(ok=False, error=self._last_error)
                return

            self._last_map_revision = int(msg.revision)
            self._last_map_yaml = str(msg.yaml_path)
            self._last_error = ""
            self._last_ok_ts = time.time()
            self._publish_health(ok=True, error="")

    def _reload_map_server(self, *, map_yaml: str) -> tuple[bool, str]:
        if self._load_map_client is None:
            return False, "load_map_client_not_ready"

        timeout = float(self.get_parameter("load_map_timeout_sec").value)
        # region agent log
        _dbg_log(
            hypothesis_id="H4",
            location="nav2_localization_orchestrator_node.py:_reload_map_server",
            message="load_map_begin",
            data={"srv": self._load_map_client.srv_name, "timeout_sec": timeout, "map_yaml": map_yaml},
        )
        # endregion
        if not self._load_map_client.wait_for_service(timeout_sec=timeout):
            # region agent log
            _dbg_log(
                hypothesis_id="H4",
                location="nav2_localization_orchestrator_node.py:_reload_map_server",
                message="load_map_service_unavailable",
                data={"srv": self._load_map_client.srv_name, "timeout_sec": timeout},
            )
            # endregion
            return False, f"load_map_service_unavailable: {self._load_map_client.srv_name}"

        req = LoadMap.Request()
        # nav2_msgs/srv/LoadMap: map_url string
        req.map_url = str(map_yaml)
        future = self._load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        # region agent log
        _dbg_log(
            hypothesis_id="H4",
            location="nav2_localization_orchestrator_node.py:_reload_map_server",
            message="load_map_future_wait_complete",
            data={"done": bool(future.done())},
        )
        # endregion
        if not future.done():
            # region agent log
            _dbg_log(
                hypothesis_id="H4",
                location="nav2_localization_orchestrator_node.py:_reload_map_server",
                message="load_map_timeout",
                data={"srv": self._load_map_client.srv_name, "timeout_sec": timeout},
            )
            # endregion
            return False, "load_map_timeout"
        res = future.result()
        if res is None:
            # region agent log
            _dbg_log(
                hypothesis_id="H4",
                location="nav2_localization_orchestrator_node.py:_reload_map_server",
                message="load_map_no_response",
                data={"srv": self._load_map_client.srv_name},
            )
            # endregion
            return False, "load_map_no_response"

        # nav2_msgs/srv/LoadMap response: {map: OccupancyGrid, result: uint8}
        # We accept any non-empty map.
        if res.map.info.width == 0 or res.map.info.height == 0:
            return False, "load_map_empty_map"
        # region agent log
        _dbg_log(
            hypothesis_id="H4",
            location="nav2_localization_orchestrator_node.py:_reload_map_server",
            message="load_map_ok",
            data={"width": int(res.map.info.width), "height": int(res.map.info.height), "result": int(res.result)},
        )
        # endregion
        return True, ""

    def _publish_initialpose_from_symovo(self, map_status: MapStatus) -> tuple[bool, str]:
        if self._client is None:
            return False, "symovo_client_not_configured"
        if self._initialpose_pub is None:
            return False, "initialpose_pub_not_configured"

        amr_id = int(self.get_parameter("amr_id").value)
        pose = self._client.get_amr_pose(amr_id=amr_id)

        geom = SymovoMapGeometry(
            width_px=int(map_status.width_px),
            height_px=int(map_status.height_px),
            resolution=float(map_status.resolution),
            offset_x=float(map_status.offset_x),
            offset_y=float(map_status.offset_y),
        )
        x_m, y_m, yaw = geom.transform_pose(
            x=float(pose.x),
            y=float(pose.y),
            theta=float(pose.theta),
            mode=str(map_status.pose_transform_mode),
        )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x_m)
        msg.pose.pose.position.y = float(y_m)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov_xy = float(self.get_parameter("initialpose_cov_xy").value)
        cov_yaw = float(self.get_parameter("initialpose_cov_yaw").value)
        # Row-major 6x6
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw

        self._initialpose_pub.publish(msg)
        return True, ""

    # ---------------------------------------------------------------------
    # Health
    # ---------------------------------------------------------------------

    def _publish_health(self, *, ok: bool, error: str) -> None:
        if self._diag_pub is None:
            return

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = f"{self.get_namespace()}/nav2_localization_orchestrator".replace("//", "/")
        st.hardware_id = "n/a"
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        st.message = "ok" if ok else (error or "error")
        st.values.append(KeyValue(key="last_ok_age_sec", value=str(max(0.0, time.time() - self._last_ok_ts))))
        if self._last_map_revision is not None:
            st.values.append(KeyValue(key="map_revision", value=str(self._last_map_revision)))
        if self._last_map_yaml:
            st.values.append(KeyValue(key="map_yaml", value=self._last_map_yaml))

        arr.status.append(st)
        self._diag_pub.publish(arr)


def main() -> int:
    rclpy.init()
    # region agent log
    _dbg_log(
        hypothesis_id="H0",
        location="nav2_localization_orchestrator_node.py:main",
        message="process_started",
        data={
            "pid": os.getpid(),
            "file": __file__,
            "argv0": sys.argv[0] if sys.argv else "",
            "cwd": os.getcwd(),
        },
    )
    # endregion
    node = Nav2LocalizationOrchestratorNode()
    try:
        # Let external lifecycle manager (launch) drive transitions.
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

