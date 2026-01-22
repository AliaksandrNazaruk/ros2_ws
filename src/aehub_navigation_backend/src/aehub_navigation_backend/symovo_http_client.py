from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

import requests


Json = Union[Dict[str, Any], List[Any], str, int, float, bool, None]


@dataclass(frozen=True)
class SymovoPose:
    x: float
    y: float
    theta: float
    map_id_ref: Optional[int]


class SymovoHttpClient:
    """
    Minimal Symovo REST client for machine-owned navigation backend.

    Notes:
    - Uses requests.Session for connection reuse.
    - Auth is not modeled here (Symovo deployments often rely on network-level access).
    """

    def __init__(self, endpoint: str, *, tls_verify: bool, timeout_sec: float = 10.0):
        self._endpoint = endpoint.rstrip("/")
        self._tls_verify = tls_verify
        self._timeout_sec = timeout_sec
        self._session = requests.Session()

    def _url(self, path: str) -> str:
        if not path.startswith("/"):
            path = "/" + path
        return f"{self._endpoint}{path}"

    def get_json(self, path: str, *, params: Optional[Dict[str, Any]] = None) -> Json:
        res = self._session.get(self._url(path), params=params, verify=self._tls_verify, timeout=self._timeout_sec)
        res.raise_for_status()
        return res.json()

    def post_json(self, path: str, payload: Dict[str, Any]) -> Json:
        res = self._session.post(self._url(path), json=payload, verify=self._tls_verify, timeout=self._timeout_sec)
        res.raise_for_status()
        return res.json() if res.content else {}

    def put_json(self, path: str, payload: Optional[Dict[str, Any]] = None) -> Json:
        res = self._session.put(self._url(path), json=payload, verify=self._tls_verify, timeout=self._timeout_sec)
        res.raise_for_status()
        return res.json() if res.content else {}

    # -------- domain helpers --------

    def get_amr(self, *, amr_id: Union[int, str]) -> Dict[str, Any]:
        data = self.get_json(f"/v0/amr/{amr_id}")
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected /v0/amr/{id} response type (expected object)")
        return data

    def get_amr_pose(self, *, amr_id: Union[int, str]) -> SymovoPose:
        payload = self.get_json(f"/v0/amr/{amr_id}/pose")
        if not isinstance(payload, dict):
            raise RuntimeError("Unexpected /v0/amr/{id}/pose response type (expected object)")
        return SymovoPose(
            x=float(payload.get("x", 0.0) or 0.0),
            y=float(payload.get("y", 0.0) or 0.0),
            theta=float(payload.get("theta", 0.0) or 0.0),
            map_id_ref=int(payload.get("map_id")) if payload.get("map_id") is not None else None,
        )

    def get_station(self, *, station_id: str) -> Dict[str, Any]:
        data = self.get_json(f"/v0/station/{station_id}")
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected /v0/station/{id} response type (expected object)")
        return data

    def create_transport(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        data = self.post_json("/v0/transport", payload)
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected /v0/transport create response type (expected object)")
        return data

    def get_transport(self, *, transport_id: str) -> Dict[str, Any]:
        data = self.get_json(f"/v0/transport/{transport_id}")
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected /v0/transport/{id} response type (expected object)")
        return data

    def start_transport(self, *, transport_id: str) -> None:
        self.put_json(f"/v0/transport/{transport_id}/start")

    def stop_transport(self, *, transport_id: str) -> None:
        self.put_json(f"/v0/transport/{transport_id}/stop")

