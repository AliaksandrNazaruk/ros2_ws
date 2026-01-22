#!/usr/bin/env python3
from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String as StringMsg

from aehub_msgs.srv import ListStations, GetStation, CreateStation, UpdateStation, DeleteStation

from aehub_station_adapter.symovo_station_client import SymovoStationClient


QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

QOS_TRANSIENT_DIAGNOSTICS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _parse_json_object(text: str) -> Optional[Dict[str, Any]]:
    try:
        obj = json.loads(text)
    except Exception:
        return None
    return obj if isinstance(obj, dict) else None


class StationAdapterNode(LifecycleNode):
    """
    Admin/ops adapter for Symovo Stations.

    Exposes ROS services that forward CRUD to Symovo REST:
    - list_stations (aehub_msgs/srv/ListStations)
    - get_station  (aehub_msgs/srv/GetStation)
    - create_station (aehub_msgs/srv/CreateStation)
    - update_station (aehub_msgs/srv/UpdateStation)
    - delete_station (aehub_msgs/srv/DeleteStation)

    Also publishes current stations list as JSON on a topic for visibility (optional).
    """

    def __init__(self) -> None:
        super().__init__("station_adapter")

        self.declare_parameter("symovo_endpoint", "https://demo.symovo.app/api")
        self.declare_parameter("tls_verify", False)
        self.declare_parameter("http_timeout_sec", 5.0)

        self.declare_parameter("stations_limit", 150)
        self.declare_parameter("publish_topic", "status/stations_json")
        self.declare_parameter("publish_hz", 0.5)  # 0.5 Hz default
        self.declare_parameter("health_topic", "health/station_adapter")
        self.declare_parameter("health_publish_hz", 1.0)

        self.declare_parameter("srv_list", "stations/list")
        self.declare_parameter("srv_get", "stations/get")
        self.declare_parameter("srv_create", "stations/create")
        self.declare_parameter("srv_update", "stations/update")
        self.declare_parameter("srv_delete", "stations/delete")

        self._active = False
        self._cli: Optional[SymovoStationClient] = None

        self._pub = None
        self._health_pub = None
        self._pub_timer = None
        self._health_timer = None

        self._srv_list = None
        self._srv_get = None
        self._srv_create = None
        self._srv_update = None
        self._srv_delete = None

        self._last_ok_ts: float = 0.0
        self._last_err: str = ""

    # ---------------- lifecycle ----------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        tls_verify = bool(self.get_parameter("tls_verify").value)
        timeout_sec = float(self.get_parameter("http_timeout_sec").value)

        self._cli = SymovoStationClient(endpoint, tls_verify=tls_verify, timeout_sec=timeout_sec)

        topic = str(self.get_parameter("publish_topic").value)
        self._pub = self.create_publisher(StringMsg, topic, QOS_RELIABLE)

        health_topic = str(self.get_parameter("health_topic").value)
        self._health_pub = self.create_publisher(DiagnosticArray, health_topic, QOS_TRANSIENT_DIAGNOSTICS)

        self._srv_list = self.create_service(ListStations, str(self.get_parameter("srv_list").value), self._handle_list)
        self._srv_get = self.create_service(GetStation, str(self.get_parameter("srv_get").value), self._handle_get)
        self._srv_create = self.create_service(CreateStation, str(self.get_parameter("srv_create").value), self._handle_create)
        self._srv_update = self.create_service(UpdateStation, str(self.get_parameter("srv_update").value), self._handle_update)
        self._srv_delete = self.create_service(DeleteStation, str(self.get_parameter("srv_delete").value), self._handle_delete)

        self.get_logger().info(f"Configured station_adapter: endpoint={endpoint}, tls_verify={tls_verify}, timeout={timeout_sec}s")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = True
        hz = float(self.get_parameter("publish_hz").value)
        self._pub_timer = self.create_timer(max(1.0 / max(hz, 0.01), 0.5), self._publish_tick)
        hh = float(self.get_parameter("health_publish_hz").value)
        self._health_timer = self.create_timer(max(1.0 / max(hh, 0.1), 0.5), self._publish_health)
        self.get_logger().info("Activated station_adapter")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        if self._pub_timer:
            self._pub_timer.cancel()
            self._pub_timer = None
        if self._health_timer:
            self._health_timer.cancel()
            self._health_timer = None
        self.get_logger().info("Deactivated station_adapter")
        return TransitionCallbackReturn.SUCCESS

    # ---------------- services ----------------
    def _handle_list(self, req: ListStations.Request, resp: ListStations.Response) -> ListStations.Response:
        del req
        if not self._cli:
            resp.success = False
            resp.message = "not_configured"
            resp.stations_json = "[]"
            return resp
        try:
            limit = int(self.get_parameter("stations_limit").value)
            stations = self._cli.list_stations(limit=limit)
            resp.success = True
            resp.message = ""
            resp.stations_json = json.dumps(stations, ensure_ascii=False)
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            resp.success = False
            resp.message = f"{e}"
            resp.stations_json = "[]"
            self._last_err = str(e)
        return resp

    def _handle_get(self, req: GetStation.Request, resp: GetStation.Response) -> GetStation.Response:
        if not self._cli:
            resp.success = False
            resp.message = "not_configured"
            resp.station_json = "{}"
            return resp
        station_id = str(req.id or "").strip()
        if not station_id:
            resp.success = False
            resp.message = "missing_id"
            resp.station_json = "{}"
            return resp
        try:
            st = self._cli.get_station(station_id=station_id)
            resp.success = True
            resp.message = ""
            resp.station_json = json.dumps(st, ensure_ascii=False)
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            resp.success = False
            resp.message = f"{e}"
            resp.station_json = "{}"
            self._last_err = str(e)
        return resp

    def _handle_create(self, req: CreateStation.Request, resp: CreateStation.Response) -> CreateStation.Response:
        if not self._cli:
            resp.success = False
            resp.message = "not_configured"
            resp.id = ""
            resp.station_json = "{}"
            return resp
        payload = _parse_json_object(req.station_json or "")
        if payload is None:
            resp.success = False
            resp.message = "invalid_json"
            resp.id = ""
            resp.station_json = "{}"
            return resp
        if "name" not in payload or "pose" not in payload or "state" not in payload:
            resp.success = False
            resp.message = "invalid_station_input: requires name, pose, state"
            resp.id = ""
            resp.station_json = "{}"
            return resp
        try:
            created = self._cli.create_station(station_input=payload)
            resp.success = True
            resp.message = ""
            resp.id = str(created.get("id") or "")
            resp.station_json = json.dumps(created, ensure_ascii=False)
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            resp.success = False
            resp.message = f"{e}"
            resp.id = ""
            resp.station_json = "{}"
            self._last_err = str(e)
        return resp

    def _handle_update(self, req: UpdateStation.Request, resp: UpdateStation.Response) -> UpdateStation.Response:
        if not self._cli:
            resp.success = False
            resp.message = "not_configured"
            resp.station_json = "{}"
            return resp
        station_id = str(req.id or "").strip()
        if not station_id:
            resp.success = False
            resp.message = "missing_id"
            resp.station_json = "{}"
            return resp
        payload = _parse_json_object(req.station_json or "")
        if payload is None:
            resp.success = False
            resp.message = "invalid_json"
            resp.station_json = "{}"
            return resp
        if "name" not in payload or "pose" not in payload or "state" not in payload:
            resp.success = False
            resp.message = "invalid_station_input: requires name, pose, state"
            resp.station_json = "{}"
            return resp
        try:
            updated = self._cli.update_station(station_id=station_id, station_input=payload)
            resp.success = True
            resp.message = ""
            resp.station_json = json.dumps(updated, ensure_ascii=False)
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            resp.success = False
            resp.message = f"{e}"
            resp.station_json = "{}"
            self._last_err = str(e)
        return resp

    def _handle_delete(self, req: DeleteStation.Request, resp: DeleteStation.Response) -> DeleteStation.Response:
        if not self._cli:
            resp.success = False
            resp.message = "not_configured"
            return resp
        station_id = str(req.id or "").strip()
        if not station_id:
            resp.success = False
            resp.message = "missing_id"
            return resp
        try:
            self._cli.delete_station(station_id=station_id)
            resp.success = True
            resp.message = ""
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            resp.success = False
            resp.message = f"{e}"
            self._last_err = str(e)
        return resp

    # ---------------- periodic publishers ----------------
    def _publish_tick(self) -> None:
        if not self._active or not self._cli or not self._pub:
            return
        try:
            limit = int(self.get_parameter("stations_limit").value)
            stations = self._cli.list_stations(limit=limit)
            msg = StringMsg()
            msg.data = json.dumps(stations, ensure_ascii=False)
            self._pub.publish(msg)
            self._last_ok_ts = time.time()
            self._last_err = ""
        except Exception as e:
            self._last_err = str(e)

    def _publish_health(self) -> None:
        if not self._active or not self._health_pub:
            return
        st = DiagnosticStatus()
        st.name = "station_adapter"
        st.hardware_id = ""
        if self._last_err:
            st.level = DiagnosticStatus.WARN
            st.message = self._last_err
        else:
            st.level = DiagnosticStatus.OK
            st.message = "OK"
        st.values.append(KeyValue(key="last_ok_timestamp", value=str(self._last_ok_ts)))
        st.values.append(KeyValue(key="symovo_endpoint", value=str(self.get_parameter("symovo_endpoint").value)))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self._health_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationAdapterNode()
    # This node is a LifecycleNode, but for admin/ops usage we auto-configure and auto-activate
    # to avoid launch-side lifecycle orchestration complexity.
    try:
        node.trigger_configure()
        node.trigger_activate()
    except Exception:
        # If lifecycle transition fails, keep spinning so services like /change_state are still available.
        pass
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

