#!/usr/bin/env python3
"""
Mock odometry publisher for testing Nav2 without real Symovo API
This publishes fake /odom and TF (odom -> base_link) so Nav2 can work
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__('mock_odom_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_odom)
        
        self.get_logger().info('Mock odometry publisher started')
        self.get_logger().info('Publishing /odom and TF (odom -> base_link)')
        self.get_logger().warn('⚠️  This is a MOCK - robot will not actually move!')
    
    def publish_odom(self):
        now = self.get_clock().now()
        
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set pose (static for now - robot at origin)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Set velocity (zero for now)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Set covariance
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish TF: odom -> base_link
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    
    try:
        node = MockOdomPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
