from __future__ import annotations

from typing import Any, Dict, List, Optional, Union

import requests

Json = Union[Dict[str, Any], List[Any], str, int, float, bool, None]


class SymovoStationClient:
    def __init__(self, endpoint: str, *, tls_verify: bool, timeout_sec: float = 10.0):
        self._endpoint = endpoint.rstrip("/")
        self._tls_verify = tls_verify
        self._timeout_sec = timeout_sec
        self._session = requests.Session()

    def _url(self, path: str) -> str:
        if not path.startswith("/"):
            path = "/" + path
        return f"{self._endpoint}{path}"

    @staticmethod
    def _raise_for_status_with_body(res: requests.Response, *, label: str) -> None:
        try:
            res.raise_for_status()
        except requests.HTTPError as e:
            # Include response body to help diagnose 4xx/5xx from the machine.
            body = ""
            try:
                body = (res.text or "")[:800]
            except Exception:
                body = ""
            raise RuntimeError(f"{label} -> HTTP {res.status_code}: {body}") from e

    def list_stations(self, *, limit: int = 150) -> List[Dict[str, Any]]:
        res = self._session.get(
            self._url("/v0/station"),
            params={"limit": int(limit)},
            verify=self._tls_verify,
            timeout=self._timeout_sec,
        )
        self._raise_for_status_with_body(res, label="GET /v0/station")
        data = res.json()
        if not isinstance(data, list):
            raise RuntimeError("Unexpected /v0/station response type (expected list)")
        return [x for x in data if isinstance(x, dict)]

    def get_station(self, *, station_id: str) -> Dict[str, Any]:
        res = self._session.get(
            self._url(f"/v0/station/{station_id}"),
            verify=self._tls_verify,
            timeout=self._timeout_sec,
        )
        self._raise_for_status_with_body(res, label="GET /v0/station/{id}")
        data = res.json()
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected /v0/station/{id} response type (expected object)")
        return data

    def create_station(self, *, station_input: Dict[str, Any]) -> Dict[str, Any]:
        res = self._session.post(
            self._url("/v0/station"),
            json=station_input,
            verify=self._tls_verify,
            timeout=self._timeout_sec,
        )
        self._raise_for_status_with_body(res, label="POST /v0/station")
        data = res.json()
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected POST /v0/station response type (expected object)")
        return data

    def update_station(self, *, station_id: str, station_input: Dict[str, Any]) -> Dict[str, Any]:
        res = self._session.put(
            self._url(f"/v0/station/{station_id}"),
            json=station_input,
            verify=self._tls_verify,
            timeout=self._timeout_sec,
        )
        self._raise_for_status_with_body(res, label="PUT /v0/station/{id}")
        data = res.json()
        if not isinstance(data, dict):
            raise RuntimeError("Unexpected PUT /v0/station/{id} response type (expected object)")
        return data

    def delete_station(self, *, station_id: str) -> None:
        res = self._session.delete(
            self._url(f"/v0/station/{station_id}"),
            verify=self._tls_verify,
            timeout=self._timeout_sec,
        )
        self._raise_for_status_with_body(res, label="DELETE /v0/station/{id}")

