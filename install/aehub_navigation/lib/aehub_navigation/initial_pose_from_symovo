#!/usr/bin/env python3

# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Initial pose publisher for Nav2 AMCL based on Symovo API pose.

Why: Nav2 navigation lifecycle autostart can fail if map->odom doesn't exist yet.
AMCL will only publish map->odom after receiving /initialpose.

This node:
- Fetches current pose from Symovo API: GET /v0/agv (finds amr_id)
- Publishes /initialpose (map frame) repeatedly for a short time
- Stops early once /amcl_pose starts publishing
"""

import math
import time
from typing import Optional, Tuple

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseFromSymovo(Node):
    def __init__(self):
        super().__init__("initial_pose_from_symovo")

        self.declare_parameter("symovo_endpoint", "https://192.168.1.100")
        self.declare_parameter("amr_id", 15)
        self.declare_parameter("tls_verify", False)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("max_publish_seconds", 15.0)
        self.declare_parameter("cov_xy", 0.25)
        self.declare_parameter("cov_yaw", 0.06853891945200942)

        self.endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        self.amr_id = int(self.get_parameter("amr_id").value)
        self.tls_verify = bool(self.get_parameter("tls_verify").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_publish_seconds = float(self.get_parameter("max_publish_seconds").value)
        self.cov_xy = float(self.get_parameter("cov_xy").value)
        self.cov_yaw = float(self.get_parameter("cov_yaw").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

        self._amcl_pose_seen = False
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)

        self._start_time = time.time()
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"InitialPoseFromSymovo: endpoint={self.endpoint}, amr_id={self.amr_id}, "
            f"tls_verify={self.tls_verify}, rate={self.publish_rate_hz}Hz"
        )

    def _amcl_cb(self, _msg: PoseWithCovarianceStamped):
        self._amcl_pose_seen = True

    def _fetch_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            url = f"{self.endpoint}/v0/agv"
            res = requests.get(url, verify=self.tls_verify, timeout=5.0)
            res.raise_for_status()
            arr = res.json()
            if not isinstance(arr, list):
                return None
            for item in arr:
                if item.get("id") == self.amr_id:
                    pose = item.get("pose") or {}
                    return float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("theta", 0.0))
            return None
        except Exception as e:
            self.get_logger().warn(f"Failed to fetch pose from Symovo API: {e}")
            return None

    def _publish_initialpose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # Use "time 0" to request the latest available TF when AMCL transforms the pose.
        # This avoids occasional "extrapolation into the future" errors when /odom TF lags by a few ms.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance[0] = self.cov_xy
        msg.pose.covariance[7] = self.cov_xy
        msg.pose.covariance[35] = self.cov_yaw

        self.pub.publish(msg)

    def _tick(self):
        if self._amcl_pose_seen:
            self.get_logger().info("AMCL pose detected; stopping initial pose publisher.")
            self.timer.cancel()
            return

        if (time.time() - self._start_time) > self.max_publish_seconds:
            self.get_logger().warn("Timeout waiting for AMCL pose; stopping initial pose publisher.")
            self.timer.cancel()
            return

        pose = self._fetch_pose()
        if not pose:
            return

        x, y, yaw = pose
        self.get_logger().info(f"Publishing /initialpose from Symovo: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        self._publish_initialpose(x, y, yaw)


def main():
    rclpy.init()
    node = InitialPoseFromSymovo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

