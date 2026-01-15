#!/usr/bin/env python3
"""
E2E test: MQTT -> navigation_integrated_node -> Nav2 -> /cmd_vel -> base_controller -> robot moves.

This uses the existing FakeHub (AE.HUB simulator) to publish a navigateTo MQTT command
and simultaneously monitors /cmd_vel and /odom to verify the robot actually starts moving.

Usage:
  cd /home/boris/ros2_ws && source install/setup.bash
  python3 scripts/e2e_mqtt_to_motion.py --robot-id robot_001 --target-id test_position

Requirements:
  - Config Service reachable (default: http://localhost:7900)
  - navigation_integrated_node running and connected to broker
  - Nav2 running (planner/controller/bt_navigator + map_server + costmaps ACTIVE)
"""

import argparse
import math
import os
import sys
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# Reuse existing FakeHub implementation
SCRIPT_DIR = os.path.dirname(__file__)
sys.path.insert(0, SCRIPT_DIR)
from fake_hub import FakeHub  # noqa: E402


class MotionMonitor(Node):
    def __init__(self):
        super().__init__("e2e_mqtt_to_motion_monitor")
        self._last_cmd_vel_time = None
        self._last_cmd_vel_nonzero_time = None
        self._last_cmd_vel = None

        self._odom_start = None
        self._odom_last = None
        self._odom_max_dist = 0.0
        self._odom_received = False

        self._amcl_start = None
        self._amcl_last = None
        self._amcl_max_dist = 0.0
        self._amcl_received = False

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 20)
        # /odom is often published with sensor-data QoS (best effort) → subscribe with compatible QoS
        self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, qos_profile_sensor_data)

    def reset_baseline(self):
        """Reset baseline (start) to current last-known positions and clear max-distance trackers."""
        if self._odom_last is not None:
            self._odom_start = self._odom_last
        else:
            self._odom_start = None
        if self._amcl_last is not None:
            self._amcl_start = self._amcl_last
        else:
            self._amcl_start = None
        self._odom_max_dist = 0.0
        self._amcl_max_dist = 0.0

    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        self._last_cmd_vel_time = now
        self._last_cmd_vel = msg
        if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self._last_cmd_vel_nonzero_time = now

    def _odom_cb(self, msg: Odometry):
        self._odom_received = True
        p = msg.pose.pose.position
        if self._odom_start is None:
            self._odom_start = (p.x, p.y)
        self._odom_last = (p.x, p.y)
        if self._odom_start is not None:
            dx = self._odom_last[0] - self._odom_start[0]
            dy = self._odom_last[1] - self._odom_start[1]
            self._odom_max_dist = max(self._odom_max_dist, math.hypot(dx, dy))

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_received = True
        p = msg.pose.pose.position
        if self._amcl_start is None:
            self._amcl_start = (p.x, p.y)
        self._amcl_last = (p.x, p.y)
        if self._amcl_start is not None:
            dx = self._amcl_last[0] - self._amcl_start[0]
            dy = self._amcl_last[1] - self._amcl_start[1]
            self._amcl_max_dist = max(self._amcl_max_dist, math.hypot(dx, dy))

    def odom_distance(self) -> float:
        return float(self._odom_max_dist or 0.0)

    def amcl_distance(self) -> float:
        return float(self._amcl_max_dist or 0.0)

    def odom_start(self):
        return self._odom_start

    def odom_last(self):
        return self._odom_last

    def amcl_start(self):
        return self._amcl_start

    def amcl_last(self):
        return self._amcl_last

    def odom_received(self) -> bool:
        return bool(self._odom_received)

    def amcl_received(self) -> bool:
        return bool(self._amcl_received)

    def has_recent_cmd_vel(self, window_s: float = 2.0) -> bool:
        if self._last_cmd_vel_time is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self._last_cmd_vel_time) <= window_s

    def has_recent_nonzero_cmd_vel(self, window_s: float = 2.0) -> bool:
        if self._last_cmd_vel_nonzero_time is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self._last_cmd_vel_nonzero_time) <= window_s


def _fmt_dt(t0: float | None, t: float | None) -> str:
    if t0 is None or t is None:
        return "n/a"
    return f"{(t - t0) * 1000.0:.0f} ms"


def _print_latency_breakdown(hub: FakeHub, command_id: str, t_cmd_vel_nonzero: float | None, t_moved: float | None):
    timing = hub.get_timing(command_id) if command_id else {}
    t0 = timing.get("t0_sent")
    # Record ROS-observed milestones into a local copy (do not mutate hub data)
    report = dict(timing)
    if t_cmd_vel_nonzero is not None:
        report.setdefault("t_cmd_vel_nonzero", t_cmd_vel_nonzero)
    if t_moved is not None:
        report.setdefault("t_moved", t_moved)

    print("\n=== Latency breakdown (t0 = hub publish) ===")
    print(f"t0_sent:                0 ms")
    print(f"ack_received:           {_fmt_dt(t0, report.get('t_ack_received'))}")
    print(f"ack_accepted:           {_fmt_dt(t0, report.get('t_ack_accepted'))}")
    print(f"status_navigating:      {_fmt_dt(t0, report.get('t_status_navigating'))}")
    print(f"cmd_vel_nonzero:        {_fmt_dt(t0, report.get('t_cmd_vel_nonzero'))}")
    print(f"odom/amcl_moved:        {_fmt_dt(t0, report.get('t_moved'))}")
    # Terminal events (may be missing if timeout)
    print(f"result_succeeded:       {_fmt_dt(t0, report.get('t_result_succeeded'))}")
    print(f"result_aborted:         {_fmt_dt(t0, report.get('t_result_aborted'))}")
    print(f"result_canceled:        {_fmt_dt(t0, report.get('t_result_canceled'))}")
    print(f"result_error:           {_fmt_dt(t0, report.get('t_result_error'))}")
    print(f"status_arrived:         {_fmt_dt(t0, report.get('t_status_arrived'))}")
    print(f"status_idle:            {_fmt_dt(t0, report.get('t_status_idle'))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default=os.getenv("ROBOT_ID", "robot_001"))
    ap.add_argument("--target-id", default=None, help="PositionRegistry target_id (alternative to --x/--y)")
    ap.add_argument("--x", type=float, default=None, help="Direct goal X (meters) (alternative to --target-id)")
    ap.add_argument("--y", type=float, default=None, help="Direct goal Y (meters) (alternative to --target-id)")
    ap.add_argument("--theta", type=float, default=None, help="Direct goal yaw theta (radians, optional with --x/--y)")
    ap.add_argument("--frame-id", default="map", help="Direct goal frame_id (default: map)")
    ap.add_argument("--priority", default="normal", choices=["normal", "high", "emergency"])
    ap.add_argument("--config-service-url", default="http://localhost:7900")
    ap.add_argument(
        "--api-key",
        default="tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4",
        help="Config Service API key (must match navigation_integrated_node config)",
    )
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--min-odom-meters", type=float, default=0.01)
    ap.add_argument("--pre-spin", type=float, default=3.0, help="Seconds to spin before sending command")
    args = ap.parse_args()

    print("=" * 78)
    print("E2E MQTT → Nav2 → /cmd_vel → base_controller → MOVEMENT")
    print("=" * 78)
    print(f"robot_id: {args.robot_id}")
    if args.x is not None or args.y is not None:
        print(f"goal: x={args.x}, y={args.y}, theta={args.theta}, frame_id={args.frame_id}")
    else:
        print(f"target_id: {args.target_id or 'test_position'}")
    print(f"priority: {args.priority}")
    print(f"timeout: {args.timeout}s, min_odom_meters: {args.min_odom_meters}m, pre_spin: {args.pre_spin}s")
    print()

    # Connect fake hub (MQTT)
    hub = FakeHub(config_service_url=args.config_service_url, api_key=args.api_key, robot_id=args.robot_id)
    if not hub.fetch_mqtt_config():
        print("❌ Failed to fetch MQTT config from Config Service")
        return 2
    if not hub.connect_mqtt():
        print("❌ Failed to connect to MQTT broker")
        return 2

    # Start ROS monitor
    rclpy.init()
    node = MotionMonitor()

    # Give ROS a moment to receive baseline odom/cmd_vel
    start_spin = time.time()
    while time.time() - start_spin < float(args.pre_spin):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Baseline: odom_received={node.odom_received()}, amcl_received={node.amcl_received()}")
    print(f"Baseline odom last: {node.odom_last()}")
    print(f"Baseline amcl last: {node.amcl_last()}")
    print("Sending MQTT navigateTo...")
    node.reset_baseline()

    t_cmd_vel_nonzero = None
    t_moved = None

    if args.x is not None or args.y is not None:
        if args.x is None or args.y is None:
            print("❌ For direct goal mode you must provide both --x and --y")
            hub.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            return 2
        cmd_id = hub.send_navigate_pose_command(
            x=float(args.x),
            y=float(args.y),
            theta=float(args.theta) if args.theta is not None else 0.0,
            priority=args.priority,
            frame_id=args.frame_id,
        )
    else:
        cmd_id = hub.send_navigate_command(args.target_id or "test_position", priority=args.priority)
    if not cmd_id:
        print("❌ Failed to publish MQTT command")
        hub.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2

    print(f"✅ MQTT command published. command_id={cmd_id}")
    print("Waiting for movement signals (/cmd_vel non-zero and /odom delta)...")

    deadline = time.time() + args.timeout
    moved = False
    got_cmd_vel = False
    got_nonzero_cmd_vel = False

    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

        got_cmd_vel = got_cmd_vel or node.has_recent_cmd_vel()
        got_nonzero_cmd_vel = got_nonzero_cmd_vel or node.has_recent_nonzero_cmd_vel()
        moved = (node.odom_distance() >= args.min_odom_meters) or (node.amcl_distance() >= args.min_odom_meters)

        # Capture first timepoints (monotonic, aligned with FakeHub timings)
        now_mono = time.monotonic()
        if t_cmd_vel_nonzero is None and node.has_recent_nonzero_cmd_vel(window_s=9999.0):
            t_cmd_vel_nonzero = now_mono
        if t_moved is None and moved:
            t_moved = now_mono

        if got_nonzero_cmd_vel and moved:
            break

    print()
    print("=== Results ===")
    print(f"cmd_vel received recently: {got_cmd_vel}")
    print(f"cmd_vel non-zero seen:    {got_nonzero_cmd_vel}")
    print(f"odom_received:            {node.odom_received()}")
    print(f"amcl_received:            {node.amcl_received()}")
    print(f"odom start → last:        {node.odom_start()} → {node.odom_last()}")
    print(f"amcl start → last:        {node.amcl_start()} → {node.amcl_last()}")
    print(f"odom distance:            {node.odom_distance():.3f} m")
    print(f"amcl distance:            {node.amcl_distance():.3f} m")

    # Also fetch last event/status from fake hub (helps debug chain)
    status_obj = hub.get_status_by_command_id(cmd_id, timeout=2.0, wait_for_error=True)
    if status_obj:
        print(f"mqtt last event/status: event_type={status_obj.get('event_type')}, status={status_obj.get('status')}, "
              f"result={status_obj.get('result_type')}, error={status_obj.get('error_code')}")
    else:
        print("mqtt last event/status: (none received)")

    # Latency breakdown from hub publish (t0) to key milestones
    _print_latency_breakdown(hub, cmd_id, t_cmd_vel_nonzero=t_cmd_vel_nonzero, t_moved=t_moved)

    hub.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    if got_nonzero_cmd_vel and moved:
        print("\n✅ E2E PASS: robot started moving from MQTT command.")
        return 0

    print("\n❌ E2E FAIL: command published, but robot did not move.")
    print("Next checks:")
    print("- Is Nav2 ACTIVE? (map_server, amcl, global_costmap, local_costmap, planner_server, controller_server)")
    print("- Is /scan publishing? (AMCL + obstacle layers need it)")
    print("- Is base_controller receiving /cmd_vel and executing speed API calls?")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

