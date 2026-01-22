from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Union

import requests


Json = Union[Dict[str, Any], List[Any], str, int, float, bool, None]


@dataclass(frozen=True)
class SymovoMapInfo:
    map_id: str
    last_changed: int
    name: str
    offset_x: float
    offset_y: float
    resolution: float
    width_px: int
    height_px: int
    enabled: bool


@dataclass(frozen=True)
class SymovoPose:
    x: float
    y: float
    theta: float
    map_id_ref: Optional[int]


class SymovoApiClient:
    """Minimal Symovo HTTP client for map/pose needs (no ROS dependencies)."""

    def __init__(self, endpoint: str, *, tls_verify: bool, timeout_sec: float = 10.0):
        self._endpoint = endpoint.rstrip("/")
        self._tls_verify = tls_verify
        self._timeout_sec = timeout_sec
        self._session = requests.Session()

    def _get_json(self, path: str) -> Json:
        url = f"{self._endpoint}{path}"
        res = self._session.get(url, verify=self._tls_verify, timeout=self._timeout_sec)
        res.raise_for_status()
        return res.json()

    def list_maps(self) -> List[SymovoMapInfo]:
        data = self._get_json("/v0/map")
        if not isinstance(data, list):
            raise RuntimeError("Unexpected /v0/map response type (expected list)")

        out: List[SymovoMapInfo] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            # Symovo API spec shows id as string; some deployments may use int-like values.
            map_id = str(item.get("id", ""))
            if not map_id:
                continue

            size = item.get("size")
            # Size-Output is usually {x: <w>, y: <h>} but be defensive:
            # some deployments may omit it or send non-object types.
            if not isinstance(size, dict):
                size = {}
            width = int(size.get("x", size.get("width", item.get("width", 0))) or 0)
            height = int(size.get("y", size.get("height", item.get("height", 0))) or 0)

            out.append(
                SymovoMapInfo(
                    map_id=map_id,
                    last_changed=int(item.get("last_changed", 0) or 0),
                    name=str(item.get("name", "") or ""),
                    offset_x=float(item.get("offsetX", 0.0) or 0.0),
                    offset_y=float(item.get("offsetY", 0.0) or 0.0),
                    resolution=float(item.get("resolution", 0.0) or 0.0),
                    width_px=width,
                    height_px=height,
                    enabled=bool(item.get("enabled", True)),
                )
            )
        return out

    def get_amr_pose(self, *, amr_id: int) -> SymovoPose:
        """
        Fetch AMR pose with robust fallbacks.

        Known deployments expose pose in one of these shapes:
        - GET /v0/amr/{id}/pose -> {x,y,theta,map_id}
        - GET /v0/amr/{id}      -> {..., pose: {x,y,theta,map_id}, ...}
        - GET /v0/agv           -> {..., pose: {x,y,theta,map_id}, ...}
        """

        def _extract_pose_obj(payload: Json) -> Optional[Dict[str, Any]]:
            # Some endpoints return a list of AGVs/AMRs; pick by id if possible.
            if isinstance(payload, list):
                for entry in payload:
                    if isinstance(entry, dict) and entry.get("id") == amr_id:
                        payload = entry
                        break
                else:
                    return None

            if not isinstance(payload, dict):
                return None

            # Direct pose object
            if "x" in payload and "y" in payload and "theta" in payload:
                return payload

            pose_obj = payload.get("pose")
            return pose_obj if isinstance(pose_obj, dict) else None

        last_err: Optional[Exception] = None
        for path in (f"/v0/amr/{amr_id}/pose", f"/v0/amr/{amr_id}", f"/v0/agv/{amr_id}", "/v0/agv"):
            try:
                payload = self._get_json(path)
            except requests.HTTPError as e:
                # Try the next fallback only for 404 (endpoint not present in this deployment).
                if getattr(e, "response", None) is not None and e.response.status_code == 404:
                    last_err = e
                    continue
                raise

            pose = _extract_pose_obj(payload)
            if pose is None:
                last_err = RuntimeError(f"Unexpected {path} response type/shape (expected pose object)")
                continue

            return SymovoPose(
                x=float(pose.get("x", 0.0) or 0.0),
                y=float(pose.get("y", 0.0) or 0.0),
                theta=float(pose.get("theta", 0.0) or 0.0),
                map_id_ref=int(pose.get("map_id")) if pose.get("map_id") is not None else None,
            )

        raise RuntimeError(f"Failed to fetch AMR pose from Symovo API: {last_err}")

    def fetch_map_full_png(self, *, map_id: Union[str, int]) -> bytes:
        map_id_str = str(map_id)
        url = f"{self._endpoint}/v0/map/{map_id_str}/full.png"
        res = self._session.get(url, verify=self._tls_verify, timeout=max(self._timeout_sec, 30.0))
        res.raise_for_status()
        return res.content

    def wait_for_map_changes(
        self,
        *,
        map_id: Union[str, int],
        last_changed: int,
        timeout_sec: float,
    ) -> Tuple[bool, Optional[int]]:
        """
        Best-effort wrapper around /v0/map/{id}/wait_for_changes.

        Returns: (changed, new_last_changed)
        - If endpoint is not supported or errors, raises.
        """
        map_id_str = str(map_id)
        url = f"{self._endpoint}/v0/map/{map_id_str}/wait_for_changes"
        res = self._session.get(
            url,
            params={"last_changed": int(last_changed)},
            verify=self._tls_verify,
            timeout=max(timeout_sec, 1.0),
        )
        res.raise_for_status()
        data = res.json()
        if not isinstance(data, dict):
            return True, None
        new_last_changed = data.get("last_changed")
        return True, int(new_last_changed) if new_last_changed is not None else None

