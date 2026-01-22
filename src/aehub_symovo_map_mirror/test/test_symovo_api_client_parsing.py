from __future__ import annotations

from typing import Any

import pytest

from aehub_symovo_map_mirror.symovo_api_client import SymovoApiClient


class _FakeResponse:
    def __init__(self, payload: Any, *, status_code: int = 200):
        self._payload = payload
        self.status_code = status_code

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise RuntimeError(f"http {self.status_code}")

    def json(self) -> Any:
        return self._payload


class _FakeSession:
    def __init__(self, route_map: dict[str, Any]):
        self._route_map = route_map

    def get(self, url: str, **kwargs):
        # url is absolute; match by path suffix for test convenience
        for k, v in self._route_map.items():
            if url.endswith(k):
                return _FakeResponse(v)
        raise RuntimeError(f"unexpected url: {url}")


def test_list_maps_parses_size_and_offsets(monkeypatch):
    client = SymovoApiClient("https://example", tls_verify=False, timeout_sec=1.0)
    monkeypatch.setattr(client, "_session", _FakeSession({
        "/v0/map": [
            {
                "id": "123",
                "last_changed": 7,
                "name": "test",
                "offsetX": 1.5,
                "offsetY": -2.0,
                "resolution": 0.05,
                "size": {"x": 100, "y": 200},
                "enabled": True,
            }
        ]
    }))

    maps = client.list_maps()
    assert len(maps) == 1
    m = maps[0]
    assert m.map_id == "123"
    assert m.last_changed == 7
    assert m.offset_x == pytest.approx(1.5)
    assert m.offset_y == pytest.approx(-2.0)
    assert m.resolution == pytest.approx(0.05)
    assert m.width_px == 100
    assert m.height_px == 200


def test_get_amr_pose_parses_map_id(monkeypatch):
    client = SymovoApiClient("https://example", tls_verify=False, timeout_sec=1.0)
    monkeypatch.setattr(client, "_session", _FakeSession({
        "/v0/amr/15/pose": {"x": 1.0, "y": 2.0, "theta": 0.3, "map_id": 9}
    }))
    pose = client.get_amr_pose(amr_id=15)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.theta == pytest.approx(0.3)
    assert pose.map_id_ref == 9

