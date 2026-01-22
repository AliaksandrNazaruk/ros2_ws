import os
import socket
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO

import pytest


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
    # Pillow is an exec dependency of the package
    from PIL import Image

    img = Image.new("L", (w, h), color=128)
    buf = BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


class _SymovoStub:
    def __init__(
        self,
        *,
        port: int,
        revision: int,
        map_id: str = "1",
        pose_404: bool = False,
        pose_fail_all: bool = False,
    ):
        self.port = port
        self.revision = int(revision)
        self.map_id = str(map_id)
        self.pose_404 = bool(pose_404)
        self.pose_fail_all = bool(pose_fail_all)
        self._httpd: HTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._png = _make_png_bytes()

    @property
    def endpoint(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def start(self) -> None:
        stub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args, **kwargs):  # noqa: D401
                return

            def do_GET(self):  # noqa: N802
                if self.path == "/v0/map":
                    payload = (
                        "["
                        "{"
                        f"\"id\": {stub.map_id},"
                        f"\"name\": \"Map\","
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
                    if stub.pose_fail_all or stub.pose_404:
                        self.send_response(404)
                        self.end_headers()
                        return
                    payload = b'{"x": 0.0, "y": 0.0, "theta": 0.0, "map_id": 1}'
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                if self.path.startswith("/v0/agv"):
                    if stub.pose_fail_all:
                        self.send_response(404)
                        self.end_headers()
                        return
                    # fallback path if needed
                    payload = b'{"id": 15, "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "map_id": 1}}'
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


def _spin_until(node, predicate, *, timeout_sec: float = 3.0) -> bool:
    import rclpy

    end = time.time() + timeout_sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


@pytest.mark.parametrize(
    "pose_mode",
    [
        "ok",
        "amr_pose_404_but_agv_ok",
        "all_pose_fail_fallback_to_first_map",
    ],
)
def test_symovo_map_mirror_produces_pair_atomic_artifacts_and_status(rclpy_context, pose_mode: str):
    import rclpy
    from aehub_symovo_map_mirror.symovo_map_mirror_node import SymovoMapMirrorNode
    from aehub_msgs.msg import MapStatus

    port = _free_tcp_port()
    stub = _SymovoStub(
        port=port,
        revision=123,
        map_id="1",
        pose_404=(pose_mode == "amr_pose_404_but_agv_ok"),
        pose_fail_all=(pose_mode == "all_pose_fail_fallback_to_first_map"),
    )
    stub.start()

    tmpdir = tempfile.mkdtemp(prefix="aehub_symovo_map_mirror_test_")

    got: dict[str, MapStatus] = {}
    node = SymovoMapMirrorNode()
    node.set_parameters(
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

    sub = node.create_subscription(MapStatus, "infra/map/status", lambda msg: got.setdefault("msg", msg), 10)
    assert sub is not None

    assert bool(node.trigger_configure())
    assert bool(node.trigger_activate())

    ok = _spin_until(node, lambda: "msg" in got and bool(got["msg"].ready))
    assert ok, "did not receive ready MapStatus in time"

    msg = got["msg"]
    assert int(msg.revision) == 123
    assert msg.yaml_path.endswith("/map.yaml")
    assert os.path.exists(msg.yaml_path)

    # Pair-atomic publication: stable map.yaml/map.pgm are symlinks through `current`.
    out_dir = tmpdir
    current_link = os.path.join(out_dir, "current")
    assert os.path.islink(current_link)
    assert os.readlink(current_link) == "rev_123"

    stable_yaml = os.path.join(out_dir, "map.yaml")
    stable_pgm = os.path.join(out_dir, "map.pgm")
    assert os.path.islink(stable_yaml)
    assert os.path.islink(stable_pgm)

    # Ensure the revision directory contains real files.
    rev_dir = os.path.join(out_dir, "rev_123")
    assert os.path.isdir(rev_dir)
    assert os.path.isfile(os.path.join(rev_dir, "map.yaml"))
    assert os.path.isfile(os.path.join(rev_dir, "map.pgm"))

    try:
        node.destroy_node()
    finally:
        stub.stop()


def test_symovo_map_mirror_rejects_non_poll_update_mode(rclpy_context):
    import rclpy
    from rclpy.lifecycle import TransitionCallbackReturn
    from aehub_symovo_map_mirror.symovo_map_mirror_node import SymovoMapMirrorNode

    node = SymovoMapMirrorNode()
    node.set_parameters([rclpy.parameter.Parameter("update_mode", rclpy.Parameter.Type.STRING, "wait_for_changes")])
    try:
        res = node.trigger_configure()
        assert res == TransitionCallbackReturn.ERROR
    finally:
        node.destroy_node()


def test_symovo_map_mirror_rejects_absolute_topic_names(rclpy_context):
    import rclpy
    from rclpy.lifecycle import TransitionCallbackReturn
    from aehub_symovo_map_mirror.symovo_map_mirror_node import SymovoMapMirrorNode

    node = SymovoMapMirrorNode()
    node.set_parameters([rclpy.parameter.Parameter("map_status_topic", rclpy.Parameter.Type.STRING, "/abs/topic")])
    try:
        res = node.trigger_configure()
        assert res == TransitionCallbackReturn.ERROR
    finally:
        node.destroy_node()

