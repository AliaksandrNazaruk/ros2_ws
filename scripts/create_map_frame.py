#!/usr/bin/env python3
"""
Create map frame in TF tree by publishing a static transform.
This is a workaround for Nav2 which requires map frame to exist.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class MapFramePublisher(Node):
    def __init__(self):
        super().__init__('map_frame_publisher')
        
        # Create static transform broadcaster
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publish static transform: map -> odom
        # This creates the map frame in TF tree
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        
        # Identity transform (map and odom aligned at origin)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish transform
        self.tf_static_broadcaster.sendTransform(transform)
        self.get_logger().info('✅ Published static transform: map -> odom')
        self.get_logger().info('   This creates the map frame in TF tree')
        
        # Keep node alive to maintain the transform
        self.get_logger().info('Map frame publisher running. Press Ctrl+C to stop.')

def main(args=None):
    rclpy.init(args=args)
    node = MapFramePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
