#!/usr/bin/env python3
"""
SimInitialPosePublisher

Publishes a single /initialpose message (AMCL initial pose) after a delay.
This replaces the RViz "Set Initial Pose" step for headless simulation.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped


class SimInitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_initial_pose_publisher")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("theta", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("delay_s", 2.0)
        self.declare_parameter("topic", "initialpose")

        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._theta = float(self.get_parameter("theta").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._delay_s = float(self.get_parameter("delay_s").value)
        self._topic = str(self.get_parameter("topic").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PoseWithCovarianceStamped, self._topic, qos)

        self._timer = self.create_timer(max(self._delay_s, 0.0), self._publish_once)
        self._published = False

        self.get_logger().info(
            f"Will publish initial pose to {self._topic} after {self._delay_s}s: "
            f"(x={self._x}, y={self._y}, theta={self._theta}, frame={self._frame_id})"
        )

    def _publish_once(self) -> None:
        if self._published:
            return
        self._published = True
        self._timer.cancel()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        # yaw -> quaternion (z,w only)
        msg.pose.pose.orientation.z = math.sin(self._theta * 0.5)
        msg.pose.pose.orientation.w = math.cos(self._theta * 0.5)

        # Reasonable small covariance for sim startup
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.5

        self._pub.publish(msg)
        self.get_logger().info("Published initial pose.")


def main() -> None:
    rclpy.init()
    node = SimInitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

