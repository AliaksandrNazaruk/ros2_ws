#!/usr/bin/env python3
"""
Set initial pose for AMCL localization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

def main():
    rclpy.init()
    
    node = Node('set_initial_pose')
    
    # Get pose from arguments or use default
    if len(sys.argv) >= 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
    else:
        # Default: current odom position
        x = 0.743
        y = -1.825
        yaw = 0.0
    
    # AMCL's /initialpose subscription uses BEST_EFFORT + VOLATILE.
    # Use matching QoS to ensure delivery.
    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
    # Give ROS graph time to discover the subscription (best-effort can be picky if we exit too fast)
    time.sleep(1.0)
    
    # Create message
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    import math
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    
    # Set covariance (uncertainty)
    msg.pose.covariance[0] = 0.25  # x
    msg.pose.covariance[7] = 0.25  # y
    msg.pose.covariance[35] = 0.06853891945200942  # yaw
    
    # Publish
    print(f"Setting initial pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")
    pub.publish(msg)

    # Allow message to go out on the wire
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("✅ Initial pose set")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
