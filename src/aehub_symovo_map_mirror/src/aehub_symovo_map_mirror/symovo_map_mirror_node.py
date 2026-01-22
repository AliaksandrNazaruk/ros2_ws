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
SymovoMapMirror (ROS Interface layer).

Responsibilities:
- detect map changes (poll /v0/map; poll-only in production)
- download /v0/map/{id}/full.png
- write pair-atomic map artifacts: map.pgm + map.yaml in output_dir
- publish typed MapStatus
- publish health (diagnostic_msgs)

Non-responsibilities:
- do not manage Nav2 lifecycle
- do not publish /initialpose
- do not contain navigation/capability logic
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional
from io import BytesIO

import numpy as np
from PIL import Image
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from aehub_msgs.msg import MapStatus

from aehub_symovo_map_mirror.symovo_api_client import SymovoApiClient, SymovoMapInfo
from aehub_symovo_map_mirror.symovo_map_geometry import SymovoMapGeometry


# region agent log
def _dbg_log(*, hypothesis_id: str, location: str, message: str, data: dict) -> None:
    try:
        payload = {
            "sessionId": "debug-session",
            "runId": os.environ.get("AEHUB_DEBUG_RUN_ID", "run1"),
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }
        with open("/home/boris/ros2_ws/.cursor/debug.log", "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
    except Exception:
        pass
# endregion


def _safe_namespace_id(ns: str) -> str:
    return (ns or "").strip("/").replace("/", "_") or "root"


def _atomic_write_bytes(path: str, content: bytes) -> None:
    tmp = f"{path}.tmp"
    with open(tmp, "wb") as f:
        f.write(content)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _atomic_write_text(path: str, content: str) -> None:
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(content)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _atomic_symlink(*, link_path: str, target: str) -> None:
    """
    Atomically create/replace a symlink at link_path pointing to target.

    Uses a temp symlink + os.replace() so readers never observe a half-written link.
    """
    tmp = f"{link_path}.tmp"
    try:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        os.symlink(target, tmp)
        os.replace(tmp, link_path)
        # region agent log
        _dbg_log(
            hypothesis_id="H1",
            location="symovo_map_mirror_node.py:_atomic_symlink",
            message="symlink_swapped",
            data={"link_path": link_path, "target": target},
        )
        # endregion
    finally:
        # Best-effort cleanup if os.replace failed before moving tmp.
        try:
            if os.path.islink(tmp):
                os.unlink(tmp)
        except Exception:
            pass


def _require_relative_topic(name: str, *, param_name: str) -> str:
    """
    Enforce global SRS invariant: topic names must be relative (no leading '/'),
    so namespaces can be applied via launch.
    """
    n = str(name or "")
    if not n:
        raise RuntimeError(f"invalid {param_name}: must be non-empty")
    if n.startswith("/"):
        raise RuntimeError(f"invalid {param_name}: must be relative (no leading '/'): {n}")
    return n


class SymovoMapMirrorNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("symovo_map_mirror")

        # Standalone mode (SRS-friendly): allow this module to run without an external lifecycle manager.
        # When true, the process will auto-configure+activate itself in main().
        self.declare_parameter("standalone_autostart", False)

        # Connection / selection
        self.declare_parameter("symovo_endpoint", "https://192.168.1.100")
        self.declare_parameter("amr_id", 15)
        self.declare_parameter("tls_verify", False)
        self.declare_parameter("map_select_mode", "pose_map_id")  # pose_map_id | first_enabled | first

        # Change detection
        self.declare_parameter("update_mode", "poll")  # poll | wait_for_changes
        self.declare_parameter("poll_period_sec", 2.0)
        self.declare_parameter("wait_timeout_sec", 10.0)

        # Output
        self.declare_parameter("map_output_dir", "")  # if empty, defaults to /tmp/aehub_maps/<ns>
        self.declare_parameter("write_absolute_image_path", False)
        self.declare_parameter("map_status_topic", "infra/map/status")

        # Geometry policy (explicit; tuned by runtime contract)
        self.declare_parameter("origin_mode", "origin_offsets")
        self.declare_parameter("pose_transform_mode", "pose_subtract_offsets_flip_y")

        # Internal runtime state
        self._client: Optional[SymovoApiClient] = None
        self._timer = None
        self._mutex = threading.Lock()
        self._active_map: Optional[SymovoMapInfo] = None
        self._last_published_revision: Optional[int] = None
        self._last_error: str = ""
        self._last_ok_ts: float = 0.0

        self._pub_status = None
        self._pub_diag = None

    # ---------------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Enforce poll-only runtime contract (wait_for_changes is unsupported in production).
        update_mode = str(self.get_parameter("update_mode").value)
        # region agent log
        _dbg_log(
            hypothesis_id="H3",
            location="symovo_map_mirror_node.py:on_configure",
            message="configure_enter",
            data={"update_mode": update_mode},
        )
        # endregion
        if update_mode != "poll":
            raise RuntimeError(f"update_mode must be 'poll' (wait_for_changes unsupported); got: {update_mode}")

        endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        tls_verify = bool(self.get_parameter("tls_verify").value)
        self._client = SymovoApiClient(endpoint, tls_verify=tls_verify, timeout_sec=10.0)

        status_topic = _require_relative_topic(
            str(self.get_parameter("map_status_topic").value),
            param_name="map_status_topic",
        )
        # region agent log
        _dbg_log(
            hypothesis_id="H3",
            location="symovo_map_mirror_node.py:on_configure",
            message="topics_validated",
            data={"map_status_topic": status_topic},
        )
        # endregion
        # Publish MapStatus as TRANSIENT_LOCAL so late joiners (e.g., localization orchestrator)
        # immediately receive the latest snapshot without waiting for a map change.
        qos_status = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_status = self.create_publisher(MapStatus, status_topic, qos_status)
        self._pub_diag = self.create_publisher(DiagnosticArray, "health/symovo_map_mirror", 10)

        self._last_error = ""
        self._active_map = None
        self._last_published_revision = None
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        poll_period = float(self.get_parameter("poll_period_sec").value)
        self._timer = self.create_timer(max(poll_period, 0.2), self._tick)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
        self._timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._client = None
        self._active_map = None
        self._last_published_revision = None
        self._last_error = ""
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------------
    # Core loop
    # ---------------------------------------------------------------------

    def _tick(self) -> None:
        with self._mutex:
            try:
                self._refresh_if_needed()
                self._publish_health(ok=True, error="")
            except Exception as e:
                self._last_error = str(e)
                self._publish_status_ready_false(error=str(e))
                self._publish_health(ok=False, error=str(e))

    def _refresh_if_needed(self) -> None:
        if self._client is None:
            raise RuntimeError("client not configured")

        maps = self._client.list_maps()
        if not maps:
            raise RuntimeError("no maps returned by /v0/map")

        selected = self._select_map(maps)
        if selected is None:
            raise RuntimeError("failed to select map")

        # Detect change
        changed = False
        if self._active_map is None:
            changed = True
        else:
            changed = (
                selected.map_id != self._active_map.map_id
                or int(selected.last_changed) != int(self._active_map.last_changed)
            )

        if not changed:
            # still publish periodic status (heartbeat)
            self._publish_status(selected, yaml_path=self._current_yaml_path(), ready=True, error="")
            return

        # Map changed: fetch and write artifacts
        yaml_path = self._write_map_artifacts(selected)
        self._active_map = selected
        self._last_ok_ts = time.time()
        self._publish_status(selected, yaml_path=yaml_path, ready=True, error="")

    def _select_map(self, maps: list[SymovoMapInfo]) -> Optional[SymovoMapInfo]:
        mode = str(self.get_parameter("map_select_mode").value)
        if mode == "pose_map_id":
            # Try to match pose.map_id (int) to map.id (string/int) defensively.
            try:
                amr_id = int(self.get_parameter("amr_id").value)
                pose = self._client.get_amr_pose(amr_id=amr_id)
                if pose.map_id_ref is not None:
                    target = str(pose.map_id_ref)
                    for mm in maps:
                        if str(mm.map_id) == target:
                            return mm
                    # Secondary: allow int coercion equality.
                    for mm in maps:
                        try:
                            if int(str(mm.map_id)) == int(pose.map_id_ref):
                                return mm
                        except Exception:
                            continue
            except Exception:
                # Isolation/liveness policy: if pose lookup fails, degrade deterministically
                # to other selection strategies instead of failing the whole tick.
                # region agent log
                _dbg_log(
                    hypothesis_id="H4",
                    location="symovo_map_mirror_node.py:_select_map",
                    message="pose_lookup_failed_fallback",
                    data={"map_select_mode": mode},
                )
                # endregion
                pass

        if mode == "first_enabled":
            for m in maps:
                if m.enabled:
                    return m

        # Default fallback: first map
        return maps[0] if maps else None

    def _output_dir(self) -> str:
        out = str(self.get_parameter("map_output_dir").value)
        if out:
            return out
        base = "/tmp/aehub_maps"
        return os.path.join(base, _safe_namespace_id(self.get_namespace()))

    def _current_yaml_path(self) -> str:
        # Stable path in output dir (may be a symlink).
        return os.path.join(self._output_dir(), "map.yaml")

    def _write_map_artifacts(self, m: SymovoMapInfo) -> str:
        if self._client is None:
            raise RuntimeError("client not configured")

        out_dir = self._output_dir()
        os.makedirs(out_dir, exist_ok=True)

        png = self._client.fetch_map_full_png(map_id=m.map_id)
        image = Image.open(BytesIO(png))
        if image.mode != "L":
            image = image.convert("L")
        img_array = np.array(image)
        if img_array.ndim != 2:
            raise RuntimeError("unexpected map png shape")

        # Pair-atomic publication strategy:
        # - Write a complete artifact pair into a revision directory.
        # - Atomically switch a single pointer symlink `current` to that directory.
        # - Expose stable paths `map.yaml` and `map.pgm` in out_dir as symlinks to `current/...`.
        # This avoids torn (yaml, pgm) pairs without requiring multi-file atomic rename.
        rev = int(m.last_changed)
        rev_dir_name = f"rev_{rev}"
        rev_dir = os.path.join(out_dir, rev_dir_name)
        os.makedirs(rev_dir, exist_ok=True)

        pgm_path = os.path.join(rev_dir, "map.pgm")
        yaml_path = os.path.join(rev_dir, "map.yaml")

        # PGM (Nav2-compatible: 0 occupied (black), 255 free (white))
        height, width = img_array.shape
        header = f"P5\n{width} {height}\n255\n".encode()
        _atomic_write_bytes(pgm_path, header + img_array.tobytes())

        origin_mode = str(self.get_parameter("origin_mode").value)
        pose_mode = str(self.get_parameter("pose_transform_mode").value)
        write_abs = bool(self.get_parameter("write_absolute_image_path").value)

        geom = SymovoMapGeometry(
            width_px=int(m.width_px) if m.width_px > 0 else int(width),
            height_px=int(m.height_px) if m.height_px > 0 else int(height),
            resolution=float(m.resolution),
            offset_x=float(m.offset_x),
            offset_y=float(m.offset_y),
        )
        origin = geom.yaml_origin(origin_mode=origin_mode)  # type: ignore[arg-type]

        image_field = os.path.abspath(pgm_path) if write_abs else os.path.basename(pgm_path)
        yaml = (
            f"image: {image_field}\n"
            f"resolution: {geom.resolution}\n"
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
        _atomic_write_text(yaml_path, yaml)

        # Store last published revision (for debugging)
        self._last_published_revision = int(m.last_changed)

        # Atomically update the published artifact pointer.
        # Use a relative target for portability within out_dir.
        _atomic_symlink(link_path=os.path.join(out_dir, "current"), target=rev_dir_name)
        # Ensure stable top-level artifact paths exist (symlinks through `current`).
        _atomic_symlink(link_path=os.path.join(out_dir, "map.yaml"), target=os.path.join("current", "map.yaml"))
        _atomic_symlink(link_path=os.path.join(out_dir, "map.pgm"), target=os.path.join("current", "map.pgm"))

        # publish status containing the exact modes chosen
        published_yaml_path = self._current_yaml_path()
        # region agent log
        _dbg_log(
            hypothesis_id="H2",
            location="symovo_map_mirror_node.py:_write_map_artifacts",
            message="artifacts_published",
            data={
                "out_dir": out_dir,
                "rev_dir": rev_dir,
                "published_yaml_path": published_yaml_path,
                "write_abs": write_abs,
                "image_field": image_field,
            },
        )
        # endregion
        self._publish_status(
            m,
            yaml_path=published_yaml_path,
            ready=True,
            error="",
            pose_mode=pose_mode,
            origin_mode=origin_mode,
        )
        return published_yaml_path

    # ---------------------------------------------------------------------
    # Publishing
    # ---------------------------------------------------------------------

    def _publish_status(
        self,
        m: SymovoMapInfo,
        *,
        yaml_path: str,
        ready: bool,
        error: str,
        pose_mode: Optional[str] = None,
        origin_mode: Optional[str] = None,
    ) -> None:
        msg = MapStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.map_id = str(m.map_id)
        msg.revision = int(m.last_changed)
        msg.resolution = float(m.resolution)
        msg.offset_x = float(m.offset_x)
        msg.offset_y = float(m.offset_y)
        msg.width_px = int(m.width_px)
        msg.height_px = int(m.height_px)
        msg.yaml_path = str(yaml_path)
        msg.pose_transform_mode = str(pose_mode or self.get_parameter("pose_transform_mode").value)
        msg.origin_mode = str(origin_mode or self.get_parameter("origin_mode").value)
        msg.ready = bool(ready)
        msg.error = str(error or "")
        if self._pub_status is not None:
            self._pub_status.publish(msg)

    def _publish_status_ready_false(self, *, error: str) -> None:
        # Publish a minimal status even if map selection failed.
        msg = MapStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ready = False
        msg.error = str(error)
        msg.pose_transform_mode = str(self.get_parameter("pose_transform_mode").value)
        msg.origin_mode = str(self.get_parameter("origin_mode").value)
        if self._pub_status is not None:
            self._pub_status.publish(msg)

    def _publish_health(self, *, ok: bool, error: str) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = f"{self.get_namespace()}/symovo_map_mirror".replace("//", "/")
        st.hardware_id = "symovo"
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        st.message = "ok" if ok else (error or "error")

        active_map = self._active_map
        if active_map is not None:
            st.values.append(KeyValue(key="map_id", value=str(active_map.map_id)))
            st.values.append(KeyValue(key="revision", value=str(active_map.last_changed)))
            st.values.append(KeyValue(key="resolution", value=str(active_map.resolution)))
            st.values.append(KeyValue(key="offsetX", value=str(active_map.offset_x)))
            st.values.append(KeyValue(key="offsetY", value=str(active_map.offset_y)))
        st.values.append(KeyValue(key="last_ok_age_sec", value=str(max(0.0, time.time() - self._last_ok_ts))))
        st.values.append(KeyValue(key="pose_transform_mode", value=str(self.get_parameter("pose_transform_mode").value)))
        st.values.append(KeyValue(key="origin_mode", value=str(self.get_parameter("origin_mode").value)))

        arr.status.append(st)
        if self._pub_diag is not None:
            self._pub_diag.publish(arr)


def main() -> int:
    rclpy.init()
    node = SymovoMapMirrorNode()
    try:
        # Standalone mode (no lifecycle manager): self-transition into ACTIVE.
        # Default is false so managed launches can control lifecycle explicitly.
        if bool(node.get_parameter("standalone_autostart").value):
            ok = bool(node.trigger_configure())
            if not ok:
                raise RuntimeError("standalone_autostart: trigger_configure() failed")
            ok = bool(node.trigger_activate())
            if not ok:
                raise RuntimeError("standalone_autostart: trigger_activate() failed")

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

