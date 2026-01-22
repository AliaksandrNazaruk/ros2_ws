import os
import socket
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO

import pytest


def _append_ndjson(message: str, *, data: dict):
    # Best-effort test evidence; keep it tiny and secret-free.
    try:
        import json

        os.makedirs("/home/boris/ros2_ws/.cursor", exist_ok=True)
        with open("/home/boris/ros2_ws/.cursor/debug.log", "a", encoding="utf-8") as f:
            f.write(json.dumps({"sessionId": "debug-session", "runId": "test", "location": "test_nav2_localization_orchestrator_integration.py", "message": message, "data": data, "timestamp": int(time.time() * 1000)}) + "\n")
    except Exception:
        pass


@pytest.fixture
def rclpy_context():
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    try:
        yield
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _free_tcp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    _, port = s.getsockname()
    s.close()
    return int(port)


def _make_png_bytes(*, w: int = 10, h: int = 10) -> bytes:
    # Pillow is an exec dependency of aehub_symovo_map_mirror
    from PIL import Image

    img = Image.new("L", (w, h), color=128)
    buf = BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


class _SymovoStub:
    """
    Minimal Symovo HTTP stub for the mirror+orchestrator integration path:
    - /v0/map
    - /v0/map/<id>/full.png
    - /v0/amr/<id>/pose
    """

    def __init__(self, *, port: int, revision: int, map_id: str = "1"):
        self.port = int(port)
        self.revision = int(revision)
        self.map_id = str(map_id)
        self._httpd: HTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._png = _make_png_bytes()

    @property
    def endpoint(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def start(self) -> None:
        stub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args, **kwargs):
                return

            def do_GET(self):  # noqa: N802
                if self.path == "/v0/map":
                    payload = (
                        "["
                        "{"
                        f"\"id\": {stub.map_id},"
                        "\"name\": \"Map\","
                        f"\"last_changed\": {stub.revision},"
                        "\"offsetX\": 1.0,"
                        "\"offsetY\": 2.0,"
                        "\"resolution\": 0.05,"
                        "\"size\": {\"x\": 10, \"y\": 10},"
                        "\"enabled\": true"
                        "}"
                        "]"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path == f"/v0/map/{stub.map_id}/full.png":
                    payload = stub._png
                    self.send_response(200)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path.startswith("/v0/amr/") and self.path.endswith("/pose"):
                    payload = b'{"x": 0.0, "y": 0.0, "theta": 0.0, "map_id": 1}'
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                self.send_response(404)
                self.end_headers()

        self._httpd = HTTPServer(("127.0.0.1", self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def _spin_executor_until(executor, predicate, *, timeout_sec: float = 5.0) -> bool:
    end = time.time() + timeout_sec
    while time.time() < end:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            return True
    return False


def _wait_until(predicate, *, timeout_sec: float = 5.0) -> bool:
    end = time.time() + timeout_sec
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def _spin_node_until(node, predicate, *, timeout_sec: float = 5.0) -> bool:
    import rclpy

    end = time.time() + timeout_sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def test_integration_mirror_to_orchestrator_loadmap_and_initialpose(rclpy_context):
    import rclpy
    from nav2_msgs.srv import LoadMap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    from aehub_msgs.msg import MapStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from aehub_symovo_map_mirror.symovo_map_mirror_node import SymovoMapMirrorNode
    from aehub_nav2_localization_orchestrator.nav2_localization_orchestrator_node import (
        Nav2LocalizationOrchestratorNode,
    )

    port = _free_tcp_port()
    stub = _SymovoStub(port=port, revision=123, map_id="1")
    stub.start()

    tmpdir = tempfile.mkdtemp(prefix="aehub_nav2_orchestrator_e2e_")

    # Stub map server (LoadMap service)
    svc_calls: dict[str, object] = {"count": 0, "last_map_url": ""}
    map_server = Node("fake_map_server")

    def on_load_map(req: LoadMap.Request, res: LoadMap.Response):
        svc_calls["count"] = int(svc_calls["count"]) + 1
        svc_calls["last_map_url"] = str(req.map_url)
        _append_ndjson("fake_load_map_called", data={"count": int(svc_calls["count"]), "map_url": str(req.map_url)})

        # Fill the provided response object (do not replace nested messages).
        res.result = 0
        res.map.info.width = 10
        res.map.info.height = 10
        res.map.info.resolution = 0.05
        res.map.data = [0] * (10 * 10)
        return res

    srv = map_server.create_service(LoadMap, "map_server/load_map", on_load_map)
    assert srv is not None

    # Orchestrator
    orchestrator = Nav2LocalizationOrchestratorNode()
    orchestrator.set_parameters(
        [
            rclpy.parameter.Parameter("symovo_endpoint", rclpy.Parameter.Type.STRING, stub.endpoint),
            rclpy.parameter.Parameter("tls_verify", rclpy.Parameter.Type.BOOL, False),
            rclpy.parameter.Parameter("map_status_topic", rclpy.Parameter.Type.STRING, "infra/map/status"),
            rclpy.parameter.Parameter("map_server_load_map_service", rclpy.Parameter.Type.STRING, "map_server/load_map"),
            rclpy.parameter.Parameter("initialpose_topic", rclpy.Parameter.Type.STRING, "initialpose"),
            rclpy.parameter.Parameter("load_map_timeout_sec", rclpy.Parameter.Type.DOUBLE, 5.0),
        ]
    )

    # Mirror
    mirror = SymovoMapMirrorNode()
    mirror.set_parameters(
        [
            rclpy.parameter.Parameter("standalone_autostart", rclpy.Parameter.Type.BOOL, False),
            rclpy.parameter.Parameter("symovo_endpoint", rclpy.Parameter.Type.STRING, stub.endpoint),
            rclpy.parameter.Parameter("tls_verify", rclpy.Parameter.Type.BOOL, False),
            rclpy.parameter.Parameter("map_output_dir", rclpy.Parameter.Type.STRING, tmpdir),
            rclpy.parameter.Parameter("poll_period_sec", rclpy.Parameter.Type.DOUBLE, 0.2),
            rclpy.parameter.Parameter("update_mode", rclpy.Parameter.Type.STRING, "poll"),
            rclpy.parameter.Parameter("map_status_topic", rclpy.Parameter.Type.STRING, "infra/map/status"),
            rclpy.parameter.Parameter("map_select_mode", rclpy.Parameter.Type.STRING, "pose_map_id"),
        ]
    )

    # Evidence collection
    got: dict[str, object] = {}

    def on_map_status(msg: MapStatus):
        got["map_status"] = msg

    def on_initialpose(msg: PoseWithCovarianceStamped):
        got["initialpose"] = msg
        _append_ndjson("initialpose_received", data={"frame_id": str(msg.header.frame_id)})

    sub_status = map_server.create_subscription(MapStatus, "infra/map/status", on_map_status, 10)
    sub_pose = map_server.create_subscription(PoseWithCovarianceStamped, "initialpose", on_initialpose, 10)
    assert sub_status is not None
    assert sub_pose is not None

    # Configure/activate nodes.
    #
    # IMPORTANT: orchestrator calls rclpy.spin_until_future_complete(self, ...) inside a subscription callback.
    # If we put both client+server in the same executor thread, this can deadlock (server won't spin while
    # orchestrator blocks). So we spin map_server+mirror in one thread, and orchestrator in another.
    assert bool(orchestrator.trigger_configure())
    assert bool(orchestrator.trigger_activate())
    assert bool(mirror.trigger_configure())

    stop_evt = threading.Event()

    ex_shared = SingleThreadedExecutor()
    ex_shared.add_node(map_server)
    ex_shared.add_node(mirror)

    def spin_loop(executor):
        from rclpy.executors import ExternalShutdownException

        while not stop_evt.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break

    t_shared = threading.Thread(target=spin_loop, args=(ex_shared,), daemon=True)
    t_shared.start()

    # Give DDS discovery time so orchestrator can see LoadMap service before first MapStatus.
    client = getattr(orchestrator, "_load_map_client", None)
    assert client is not None
    assert _spin_node_until(orchestrator, lambda: bool(client.service_is_ready()), timeout_sec=3.0), "LoadMap service not discovered in time"

    # Now let mirror start publishing.
    assert bool(mirror.trigger_activate())

    ok = _spin_node_until(
        orchestrator,
        lambda: ("map_status" in got and bool(getattr(got["map_status"], "ready", False)) and "initialpose" in got),
        timeout_sec=10.0,
    )
    assert ok, "did not observe MapStatus->LoadMap->initialpose chain in time"

    ms: MapStatus = got["map_status"]  # type: ignore[assignment]
    ip: PoseWithCovarianceStamped = got["initialpose"]  # type: ignore[assignment]

    # Verify mirror produced artifacts and status
    assert int(ms.revision) == 123
    assert ms.yaml_path.endswith("/map.yaml")
    assert os.path.exists(ms.yaml_path)

    # Verify orchestrator called LoadMap with MapStatus.yaml_path
    assert int(svc_calls["count"]) >= 1
    assert str(svc_calls["last_map_url"]) == str(ms.yaml_path)

    # Verify initialpose contract
    assert ip.header.frame_id == "map"
    assert len(list(ip.pose.covariance)) == 36
    assert float(ip.pose.covariance[0]) > 0.0
    assert float(ip.pose.covariance[7]) > 0.0
    assert float(ip.pose.covariance[35]) > 0.0

    # Dedup: map mirror may republish same revision; orchestrator must not reload repeatedly.
    # Allow a small grace window for repeated publications; verify service calls stay bounded.
    time.sleep(1.5)
    assert int(svc_calls["count"]) == 1

    try:
        stop_evt.set()
        t_shared.join(timeout=2.0)
        mirror.destroy_node()
        orchestrator.destroy_node()
        map_server.destroy_node()
    finally:
        stub.stop()

