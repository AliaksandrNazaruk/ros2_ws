#!/usr/bin/env python3
"""
Check base_controller status - verify if it's getting data from Symovo API
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class BaseControllerChecker(Node):
    def __init__(self):
        super().__init__('base_controller_checker')
        
        self.odom_received = False
        self.last_odom = None
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Checking base_controller status...')
    
    def odom_callback(self, msg):
        self.odom_received = True
        self.last_odom = msg
        print(f"✅ /odom received: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
    
    def check(self, timeout=5.0):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.odom_received:
                break
        
        print("\n" + "="*60)
        if self.odom_received:
            print("✅ base_controller is working!")
            print(f"   Position: x={self.last_odom.pose.pose.position.x:.3f}, y={self.last_odom.pose.pose.position.y:.3f}")
            print(f"   Frame: {self.last_odom.header.frame_id} -> {self.last_odom.child_frame_id}")
        else:
            print("❌ base_controller is NOT publishing /odom")
            print("   This means:")
            print("   1. Symovo API is not available")
            print("   2. Symovo API is not returning data for configured amr_id")
            print("   3. base_controller cannot connect to Symovo API")
            print("\n   Without /odom, Nav2 cannot work!")
        print("="*60)

def main():
    rclpy.init()
    
    try:
        checker = BaseControllerChecker()
        checker.check(timeout=5.0)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
