#!/usr/bin/env python3
"""
Publish static TF transformation map->odom for testing.

This is a temporary solution for testing navigation without AMCL localization.
In production, AMCL should provide this transformation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticMapOdomTF(Node):
    def __init__(self):
        super().__init__('static_map_odom_tf')
        
        # Create static transform broadcaster
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publish static transform: map -> odom
        # This assumes the robot is at position (0.836, -2.188) in odom frame
        # and we want to align it with the map origin
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        
        # Identity transform (map and odom are aligned at origin)
        # For testing: align map origin with odom origin
        # This allows Nav2 to work without AMCL localization
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(transform)
        self.get_logger().info('Published static transform: map -> odom')
        self.get_logger().info('  Translation: (0.0, 0.0, 0.0)')
        self.get_logger().info('  Rotation: (0, 0, 0, 1)')

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapOdomTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
