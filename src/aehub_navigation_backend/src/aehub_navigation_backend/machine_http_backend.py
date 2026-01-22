from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from aehub_navigation_backend.symovo_http_client import SymovoHttpClient, SymovoPose


TransportState = int


class TransportStates:
    UNKNOWN = 0
    UNASSIGNED = 1
    ASSIGNED = 2
    RECEIVED = 3
    STARTING = 4
    RUNNING = 5
    CANCELING = 9
    CANCELED = 6
    ERROR = 7
    FINISHED = 8


@dataclass
class BackendTelemetry:
    progress_percent: float
    eta_seconds: int
    current_pose: SymovoPose


@dataclass
class BackendReadiness:
    ready: bool
    summary: str


def _safe_bool(v: Any) -> bool:
    return bool(v) if isinstance(v, bool) else False


class MachineHttpBackend:
    """
    Symovo Transport-based navigation backend (machine-owned).

    - navigateTo(target_id) -> create transport with GoToStationStep(station_id) -> start -> monitor -> finish
    - cancel(command_id) -> stop transport -> wait terminal (best-effort)
    """

    def __init__(
        self,
        *,
        endpoint: str,
        tls_verify: bool,
        timeout_sec: float,
        amr_id: int,
        targets: Dict[str, str],
    ) -> None:
        self._amr_id = int(amr_id)
        self._targets = dict(targets)
        self._http = SymovoHttpClient(endpoint, tls_verify=tls_verify, timeout_sec=timeout_sec)

        # cache station poses (id -> {pose})
        self._station_pose_cache: Dict[str, SymovoPose] = {}

    def resolve_target(self, target_id: str) -> Optional[str]:
        station_id = self._targets.get(target_id)
        if not station_id:
            return None
        return str(station_id)

    # ---------------- readiness ----------------
    def check_readiness(self) -> BackendReadiness:
        try:
            amr = self._http.get_amr(amr_id=self._amr_id)
        except Exception as e:
            return BackendReadiness(False, f"machine_unreachable: {e}")

        flags = amr.get("state_flags") if isinstance(amr, dict) else None
        if not isinstance(flags, dict):
            return BackendReadiness(False, "machine_state_flags_missing")

        # Required OKs
        drive_ready = _safe_bool(flags.get("drive_ready"))
        safety_cleared = _safe_bool(flags.get("safety_cleared"))
        robot_paused = _safe_bool(flags.get("robot_paused"))
        charging = _safe_bool(flags.get("charging_connector"))

        if not drive_ready:
            return BackendReadiness(False, "not_ready: drive_not_ready")
        if not safety_cleared:
            return BackendReadiness(False, "not_ready: safety_not_cleared")
        if robot_paused:
            return BackendReadiness(False, "not_ready: robot_paused")
        if charging:
            return BackendReadiness(False, "not_ready: charging_connector")

        # Deny-list (safety/timeouts/errors)
        deny_true = [
            ("backend_error", "backend_error"),
            ("odom_timeout", "odom_timeout"),
            ("imu_timeout", "imu_timeout"),
            ("laser_timeout", "laser_timeout"),
            ("waiting_for_scanner", "waiting_for_scanner"),
            ("sfuse_blown", "sfuse_blown"),
            ("safety_relais_reset_request", "safety_reset_required"),
            ("emergency_stop_reset_request", "estop_reset_required"),
            ("shutdown_request", "shutdown_request"),
            ("tmc_motor_enable_error", "motor_enable_error"),
            ("tmc_initialized", "motor_controller_not_initialized"),
        ]
        for key, reason in deny_true:
            v = flags.get(key)
            if key == "tmc_initialized":
                # inverted semantics: False blocks
                if isinstance(v, bool) and v is False:
                    return BackendReadiness(False, f"not_ready: {reason}")
                continue
            if isinstance(v, bool) and v is True:
                return BackendReadiness(False, f"not_ready: {reason}")

        return BackendReadiness(True, "READY")

    # ---------------- transport operations ----------------
    def create_and_start_transport(self, *, command_id: str, target_id: str) -> str:
        station_id = self.resolve_target(target_id)
        if not station_id:
            raise ValueError(f"unknown_target_id: {target_id}")

        payload = {
            "description": f"AEHUB navigateTo {target_id} ({command_id})",
            "steps": [
                {
                    "station": {"id": station_id},
                    "type_name": "GoToStationStep",
                }
            ],
        }
        created = self._http.create_transport(payload)
        transport_id = str(created.get("id") or "")
        if not transport_id:
            raise RuntimeError("transport_create_failed: missing id")

        self._http.start_transport(transport_id=transport_id)
        return transport_id

    def stop_transport(self, *, transport_id: str) -> None:
        self._http.stop_transport(transport_id=transport_id)

    def get_transport_state(self, *, transport_id: str) -> Tuple[TransportState, Dict[str, Any]]:
        tr = self._http.get_transport(transport_id=transport_id)
        state = tr.get("state")
        try:
            return int(state), tr
        except Exception:
            return TransportStates.UNKNOWN, tr

    # ---------------- telemetry ----------------
    def _get_station_pose(self, station_id: str) -> Optional[SymovoPose]:
        if station_id in self._station_pose_cache:
            return self._station_pose_cache[station_id]
        st = self._http.get_station(station_id=station_id)
        pose = st.get("pose")
        if not isinstance(pose, dict):
            return None
        p = SymovoPose(
            x=float(pose.get("x", 0.0) or 0.0),
            y=float(pose.get("y", 0.0) or 0.0),
            theta=float(pose.get("theta", 0.0) or 0.0),
            map_id_ref=int(pose.get("map_id")) if pose.get("map_id") is not None else None,
        )
        self._station_pose_cache[station_id] = p
        return p

    def compute_telemetry(
        self,
        *,
        target_id: str,
        started_at_monotonic_s: float,
        initial_distance_m: float,
    ) -> BackendTelemetry:
        pose = self._http.get_amr_pose(amr_id=self._amr_id)

        station_id = self.resolve_target(target_id) or ""
        target_pose = self._get_station_pose(station_id) if station_id else None
        if target_pose is None:
            return BackendTelemetry(progress_percent=0.0, eta_seconds=-1, current_pose=pose)

        dist = math.hypot(target_pose.x - pose.x, target_pose.y - pose.y)
        denom = initial_distance_m if initial_distance_m > 0.01 else max(dist, 0.01)
        progress = max(0.0, min(99.0, 100.0 * (1.0 - dist / denom)))

        # Best-effort ETA from AMR velocity
        try:
            amr = self._http.get_amr(amr_id=self._amr_id)
            vel = amr.get("velocity") if isinstance(amr, dict) else None
            vx = float(vel.get("x", 0.0) or 0.0) if isinstance(vel, dict) else 0.0
            vy = float(vel.get("y", 0.0) or 0.0) if isinstance(vel, dict) else 0.0
            speed = math.hypot(vx, vy)
            if speed < 0.05:
                eta = -1
            else:
                eta = int(max(0.0, dist / speed))
        except Exception:
            eta = -1

        # If we have no motion for a long time, ETA becomes unknown (avoid nonsense).
        if (time.monotonic() - started_at_monotonic_s) > 5.0 and eta == -1:
            eta = -1

        return BackendTelemetry(progress_percent=float(progress), eta_seconds=int(eta), current_pose=pose)

    def distance_to_target_m(self, *, target_id: str) -> Optional[float]:
        """
        Best-effort current distance (meters) from AMR pose to target station pose.
        Returns None if station pose is unavailable.
        """
        pose = self._http.get_amr_pose(amr_id=self._amr_id)
        station_id = self.resolve_target(target_id) or ""
        target_pose = self._get_station_pose(station_id) if station_id else None
        if target_pose is None:
            return None
        return float(math.hypot(target_pose.x - pose.x, target_pose.y - pose.y))

