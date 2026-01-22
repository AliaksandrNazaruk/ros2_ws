#!/usr/bin/env python3
"""
SimRobotStatusPublisher

Publishes aehub_msgs/RobotStatus for simulation / loopback profiles.

Why: RobotReadinessGate blocks navigation when motors/estop validity is unknown.
In sim we want deterministic readiness toggling without Symovo.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_srvs.srv import SetBool

from aehub_msgs.msg import RobotStatus
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class SimRobotStatusPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_robot_status_publisher")

        # Parameters
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("blocked", False)
        self.declare_parameter("robot_status_topic", "robot/status")
        self.declare_parameter("publish_fake_amcl_pose", True)
        self.declare_parameter("publish_fake_odom", True)

        self._blocked: bool = bool(self.get_parameter("blocked").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        topic = str(self.get_parameter("robot_status_topic").value)
        self._publish_fake_amcl_pose = bool(self.get_parameter("publish_fake_amcl_pose").value)
        self._publish_fake_odom = bool(self.get_parameter("publish_fake_odom").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(RobotStatus, topic, qos)

        # Nav2ReadinessGate expects amcl_pose evidence by default.
        # Loopback sim doesn't run AMCL, so we publish a minimal pose in map frame.
        self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 1)

        # RobotReadinessGate requires fresh /odom evidence.
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Ensure RobotReadinessGate sees a /cmd_vel consumer in simulation.
        # CmdVelWatcher uses graph introspection for subscriptions to topic name "cmd_vel".
        self._cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        # Service to toggle block/unblock
        self._srv = self.create_service(SetBool, "sim/set_blocked", self._on_set_blocked)

        period = 1.0 / max(self._publish_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)

        # Publish immediately so late-joiners get a latched snapshot.
        self._publish()
        self.get_logger().info(
            f"SimRobotStatusPublisher started: topic={topic}, blocked={self._blocked}, publish_hz={self._publish_hz}"
        )

    def _on_cmd_vel(self, _msg: Twist) -> None:
        # Intentionally ignore: this subscriber exists to satisfy readiness checks.
        return

    def _on_set_blocked(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._blocked = bool(req.data)
        self._publish()
        res.success = True
        res.message = "blocked" if self._blocked else "unblocked"
        return res

    def _tick(self) -> None:
        self._publish()

    def _publish(self) -> None:
        if self._publish_fake_odom:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)

        if self._publish_fake_amcl_pose:
            amcl = PoseWithCovarianceStamped()
            amcl.header.stamp = self.get_clock().now().to_msg()
            amcl.header.frame_id = "map"
            amcl.pose.pose.position.x = 0.0
            amcl.pose.pose.position.y = 0.0
            amcl.pose.pose.position.z = 0.0
            amcl.pose.pose.orientation.w = 1.0
            # Small covariance (not meaningful, just non-zero).
            amcl.pose.covariance[0] = 0.05
            amcl.pose.covariance[7] = 0.05
            amcl.pose.covariance[35] = 0.1
            self._amcl_pub.publish(amcl)

        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()

        # These fields are used by robot readiness:
        msg.driver_connected = True
        msg.source = "sim"

        # Pose/velocity are not enforced by readiness gates today, but keep them valid.
        msg.pose_valid = True
        msg.velocity_valid = True

        # Make validity explicit (this is the critical part).
        msg.motors_enabled_valid = True
        msg.estop_active_valid = True

        if self._blocked:
            msg.motors_enabled = False
            msg.estop_active = True
            msg.details = "sim_blocked"
        else:
            msg.motors_enabled = True
            msg.estop_active = False
            msg.details = "sim_ok"

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimRobotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

