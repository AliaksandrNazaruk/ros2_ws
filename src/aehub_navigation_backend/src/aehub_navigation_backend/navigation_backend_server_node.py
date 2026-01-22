#!/usr/bin/env python3
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from aehub_msgs.action import NavigationExecute
from aehub_msgs.msg import NavigationStatus as NavigationStatusMsg

from aehub_navigation_backend.machine_http_backend import MachineHttpBackend, TransportStates

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


@dataclass
class _ActiveExecution:
    command_id: str
    target_id: str
    transport_id: str
    started_monotonic_s: float
    initial_distance_m: float
    done_event: threading.Event
    final_status: Optional[str] = None  # succeeded|canceled|aborted|error
    final_reason: str = ""


NAV_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

QOS_TRANSIENT_DIAGNOSTICS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class NavigationBackendServerNode(LifecycleNode):
    """
    Machine-owned navigation backend server.

    Exposes the same internal action API as the Nav2 capability server:
      - capabilities/navigation/execute (aehub_msgs/action/NavigationExecute)

    Uses Symovo Transport as the source of truth for:
      - readiness
      - busy
      - progress/eta/pose
      - errors
    """

    def __init__(self) -> None:
        super().__init__("navigation_backend_server")

        # --- parameters ---
        self.declare_parameter("action_name", "capabilities/navigation/execute")

        self.declare_parameter("symovo_endpoint", "https://demo.symovo.app/api")
        self.declare_parameter("amr_id", 15)
        self.declare_parameter("tls_verify", False)
        self.declare_parameter("http_timeout_sec", 5.0)

        self.declare_parameter("targets_yaml", "")

        self.declare_parameter("status_topic", "status/navigation")
        self.declare_parameter("status_publish_hz", 2.0)
        self.declare_parameter("health_topic", "health/navigation_backend_server")
        self.declare_parameter("health_publish_hz", 1.0)

        # --- runtime ---
        self._active = False
        self._backend: Optional[MachineHttpBackend] = None

        self._action_server: Optional[ActionServer] = None
        self._status_pub = None
        self._health_pub = None
        self._status_timer = None
        self._health_timer = None

        self._lock = threading.Lock()
        self._active_exec: Optional[_ActiveExecution] = None
        self._known_by_command_id: Dict[str, _ActiveExecution] = {}

        self._last_machine_http_ok_ts: Optional[float] = None

    # ---------------- lifecycle ----------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state

        endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        amr_id = int(self.get_parameter("amr_id").value)
        tls_verify = bool(self.get_parameter("tls_verify").value)
        timeout_sec = float(self.get_parameter("http_timeout_sec").value)

        targets_yaml = str(self.get_parameter("targets_yaml").value or "").strip()
        targets = self._load_targets_yaml(targets_yaml)
        if not targets:
            self.get_logger().error("No targets configured. Set targets_yaml to a YAML with targets: {target_id: station_id}.")
            return TransitionCallbackReturn.FAILURE

        self._backend = MachineHttpBackend(
            endpoint=endpoint,
            tls_verify=tls_verify,
            timeout_sec=timeout_sec,
            amr_id=amr_id,
            targets=targets,
        )

        status_topic = str(self.get_parameter("status_topic").value)
        self._status_pub = self.create_publisher(NavigationStatusMsg, status_topic, NAV_STATUS_QOS)

        health_topic = str(self.get_parameter("health_topic").value)
        self._health_pub = self.create_publisher(DiagnosticArray, health_topic, QOS_TRANSIENT_DIAGNOSTICS)

        self.get_logger().info(
            f"Configured machine-owned backend: endpoint={endpoint}, amr_id={amr_id}, tls_verify={tls_verify}, "
            f"targets={list(targets.keys())}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        del state
        if not self._backend:
            self.get_logger().error("Cannot activate: backend not configured")
            return TransitionCallbackReturn.FAILURE

        action_name = str(self.get_parameter("action_name").value)
        self._action_server = ActionServer(
            self,
            NavigationExecute,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            handle_accepted_callback=self._on_accepted,
        )

        hz = float(self.get_parameter("status_publish_hz").value)
        self._status_timer = self.create_timer(max(1.0 / max(hz, 0.1), 0.1), self._publish_status_tick)

        hh = float(self.get_parameter("health_publish_hz").value)
        self._health_timer = self.create_timer(max(1.0 / max(hh, 0.1), 0.5), self._publish_health)

        self._active = True
        self.get_logger().info("Activated navigation backend server")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False

        if self._status_timer:
            self._status_timer.cancel()
            self._status_timer = None
        if self._health_timer:
            self._health_timer.cancel()
            self._health_timer = None

        # rclpy ActionServer doesn't have explicit destroy helper; drop ref and rely on node cleanup.
        self._action_server = None

        self.get_logger().info("Deactivated navigation backend server")
        return TransitionCallbackReturn.SUCCESS

    # ---------------- action callbacks ----------------
    def _on_goal(self, goal_request: NavigationExecute.Goal) -> GoalResponse:
        if not self._active or not self._backend:
            return GoalResponse.REJECT

        cmd_id = str(goal_request.command_id or "")
        target_id = str(goal_request.target_id or "")
        if not cmd_id or not target_id:
            return GoalResponse.REJECT

        if self._backend.resolve_target(target_id) is None:
            return GoalResponse.REJECT

        with self._lock:
            # Single-active invariant (MVP)
            if self._active_exec and self._active_exec.command_id != cmd_id and not self._active_exec.done_event.is_set():
                # Accept then finish with "busy" would be nicer, but GoalResponse has no payload.
                # We accept and will immediately finish with error in execute path.
                return GoalResponse.ACCEPT

            # Idempotency: if same command_id already exists, accept and attach to same execution.
            return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        # Always accept cancel; backend will attempt stop if it owns a transport.
        return CancelResponse.ACCEPT

    def _on_accepted(self, goal_handle) -> None:
        # Standard rclpy pattern: start execute_callback in executor thread.
        goal_handle.execute()

    def _execute_callback(self, goal_handle) -> NavigationExecute.Result:
        """
        Blocking execute callback (runs in action server execution context).
        """
        res = NavigationExecute.Result()

        if not self._backend:
            res.command_id = str(goal_handle.request.command_id or "")
            res.status = "error"
            res.reason = "backend_not_configured"
            goal_handle.abort()
            return res

        goal = goal_handle.request
        cmd_id = str(goal.command_id or "")
        target_id = str(goal.target_id or "")
        res.command_id = cmd_id

        # Busy policy: only one active command at a time (MVP)
        with self._lock:
            if self._active_exec and self._active_exec.command_id != cmd_id and not self._active_exec.done_event.is_set():
                res.status = "error"
                res.reason = "busy: active_navigation_in_progress"
                goal_handle.abort()
                return res

            existing = self._known_by_command_id.get(cmd_id)
            exec_obj = existing if existing else None

        if exec_obj is None:
            readiness = self._backend.check_readiness()
            if readiness.ready:
                self._last_machine_http_ok_ts = time.time()
            else:
                res.status = "error"
                res.reason = readiness.summary
                goal_handle.abort()
                return res

            try:
                dist0 = self._backend.distance_to_target_m(target_id=target_id)
                initial_distance = max(0.01, float(dist0 if dist0 is not None else 0.01))
                transport_id = self._backend.create_and_start_transport(command_id=cmd_id, target_id=target_id)
            except Exception as e:
                res.status = "error"
                res.reason = f"machine_http_error: {e}"
                goal_handle.abort()
                return res

            exec_obj = _ActiveExecution(
                command_id=cmd_id,
                target_id=target_id,
                transport_id=transport_id,
                started_monotonic_s=time.monotonic(),
                initial_distance_m=float(initial_distance),
                done_event=threading.Event(),
            )
            with self._lock:
                self._active_exec = exec_obj
                self._known_by_command_id[cmd_id] = exec_obj

            mt = threading.Thread(target=self._monitor_transport, args=(exec_obj,), daemon=True)
            mt.start()

        # Wait for completion or cancellation request
        while rclpy.ok() and not exec_obj.done_event.is_set():
            if goal_handle.is_cancel_requested:
                try:
                    self._backend.stop_transport(transport_id=exec_obj.transport_id)
                except Exception:
                    pass
                exec_obj.final_status = "canceled"
                exec_obj.final_reason = ""
                exec_obj.done_event.set()
                break
            time.sleep(0.2)

        status = exec_obj.final_status or "error"
        res.status = status
        res.reason = exec_obj.final_reason or ""

        if status == "succeeded":
            goal_handle.succeed()
        elif status == "canceled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return res

    # ---------------- monitoring ----------------
    def _monitor_transport(self, exec_obj: _ActiveExecution) -> None:
        assert self._backend is not None

        while rclpy.ok() and not exec_obj.done_event.is_set():
            try:
                state, tr = self._backend.get_transport_state(transport_id=exec_obj.transport_id)
                self._last_machine_http_ok_ts = time.time()
            except Exception as e:
                exec_obj.final_status = "error"
                exec_obj.final_reason = f"machine_http_error: {e}"
                exec_obj.done_event.set()
                break

            if state in (TransportStates.STARTING, TransportStates.RUNNING, TransportStates.CANCELING):
                time.sleep(0.5)
                continue

            if state == TransportStates.FINISHED:
                exec_obj.final_status = "succeeded"
                exec_obj.final_reason = ""
            elif state == TransportStates.CANCELED:
                exec_obj.final_status = "canceled"
                exec_obj.final_reason = ""
            elif state == TransportStates.ERROR:
                exec_obj.final_status = "error"
                exec_obj.final_reason = self._extract_transport_error_reason(tr)
            else:
                exec_obj.final_status = "error"
                exec_obj.final_reason = f"unexpected_transport_state: {state}"

            exec_obj.done_event.set()
            break

        with self._lock:
            if self._active_exec and self._active_exec.command_id == exec_obj.command_id:
                self._active_exec = None

    @staticmethod
    def _extract_transport_error_reason(transport_obj: dict) -> str:
        # Best-effort extraction from state_log (contains error codes).
        sl = transport_obj.get("state_log")
        if isinstance(sl, list) and sl:
            last = sl[-1]
            if isinstance(last, dict):
                code = last.get("status_code")
                try:
                    return f"transport_error_code:{int(code)}"
                except Exception:
                    return "transport_error"
        return "transport_error"

    # ---------------- periodic publishers ----------------
    def _publish_status_tick(self) -> None:
        if not self._active or not self._backend or not self._status_pub:
            return

        with self._lock:
            ex = self._active_exec

        msg = NavigationStatusMsg()
        msg.stamp = self.get_clock().now().to_msg()

        if not ex:
            msg.status = "idle"
            msg.command_id = ""
            msg.target_id = ""
            msg.progress_percent = 0.0
            msg.eta_seconds = -1
            msg.current_x = 0.0
            msg.current_y = 0.0
            msg.current_theta = 0.0
            self._status_pub.publish(msg)
            return

        try:
            telem = self._backend.compute_telemetry(
                target_id=ex.target_id,
                started_at_monotonic_s=ex.started_monotonic_s,
                initial_distance_m=ex.initial_distance_m,
            )
            self._last_machine_http_ok_ts = time.time()
        except Exception:
            # Keep publishing without crashing; publish unknowns.
            telem = None

        msg.status = "navigating" if not ex.done_event.is_set() else (ex.final_status or "error")
        # Normalize final statuses to UI vocabulary
        if msg.status == "succeeded":
            msg.status = "arrived"
        elif msg.status == "canceled":
            msg.status = "canceled"
        elif msg.status == "error":
            msg.status = "error"

        msg.command_id = ex.command_id
        msg.target_id = ex.target_id
        msg.progress_percent = float(telem.progress_percent) if telem else 0.0
        msg.eta_seconds = int(telem.eta_seconds) if telem else -1
        if telem:
            msg.current_x = float(telem.current_pose.x)
            msg.current_y = float(telem.current_pose.y)
            msg.current_theta = float(telem.current_pose.theta)

        self._status_pub.publish(msg)

    def _publish_health(self) -> None:
        if not self._active or not self._health_pub:
            return

        st = DiagnosticStatus()
        st.name = "navigation_backend_server"
        st.hardware_id = ""

        with self._lock:
            ex = self._active_exec

        if not self._backend:
            st.level = DiagnosticStatus.ERROR
            st.message = "backend_not_configured"
        else:
            readiness = self._backend.check_readiness()
            st.level = DiagnosticStatus.OK if readiness.ready else DiagnosticStatus.WARN
            st.message = readiness.summary

        st.values.append(KeyValue(key="active_command_id", value=ex.command_id if ex else ""))
        st.values.append(KeyValue(key="active_target_id", value=ex.target_id if ex else ""))
        st.values.append(KeyValue(key="active_transport_id", value=ex.transport_id if ex else ""))
        st.values.append(
            KeyValue(key="last_machine_http_ok_timestamp", value=str(self._last_machine_http_ok_ts or 0.0))
        )

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self._health_pub.publish(arr)

    # ---------------- config helpers ----------------
    def _load_targets_yaml(self, path: str) -> Dict[str, str]:
        if not path:
            # No default: force explicit config in production.
            return {}
        if not os.path.exists(path):
            self.get_logger().error(f"targets_yaml not found: {path}")
            return {}
        if yaml is None:
            self.get_logger().error("pyyaml not available (python3-yaml missing)")
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to read targets_yaml: {e}")
            return {}
        targets = data.get("targets") if isinstance(data, dict) else None
        if not isinstance(targets, dict):
            return {}
        out: Dict[str, str] = {}
        for k, v in targets.items():
            if not isinstance(k, str) or not k.strip():
                continue
            if not isinstance(v, str) or not v.strip():
                continue
            out[k.strip()] = v.strip()
        return out


def main() -> None:
    rclpy.init()
    node = NavigationBackendServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

