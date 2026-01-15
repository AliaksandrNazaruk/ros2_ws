#!/usr/bin/env python3
"""
Диагностика проблем Nav2/AMCL/TF
Проверяет все компоненты системы навигации
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import time
import sys

class Nav2Diagnostics(Node):
    def __init__(self):
        super().__init__('nav2_diagnostics')
        
        # Subscribers
        self.scan_received = False
        self.scan_count = 0
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.initialpose_received = False
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10
        )
        
        self.cmd_vel_received = False
        self.cmd_vel_count = 0
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.odom_received = False
        self.odom_count = 0
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Timer for checking
        self.timer = self.create_timer(1.0, self.check_status)
        self.start_time = time.time()
        self.check_duration = 10.0
        
    def scan_callback(self, msg):
        self.scan_received = True
        self.scan_count += 1
        
    def initialpose_callback(self, msg):
        self.initialpose_received = True
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True
        self.cmd_vel_count += 1
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.get_logger().info(f'cmd_vel non-zero: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}')
            
    def odom_callback(self, msg):
        self.odom_received = True
        self.odom_count += 1
        
    def check_status(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.check_duration:
            self.print_report()
            rclpy.shutdown()
            sys.exit(0)
            
    def print_report(self):
        print("\n" + "="*60)
        print("NAV2 DIAGNOSTICS REPORT")
        print("="*60)
        
        print("\n1. /scan topic:")
        print(f"   Received: {self.scan_received} (count: {self.scan_count})")
        if not self.scan_received:
            print("   ❌ PROBLEM: No /scan messages received!")
        else:
            print("   ✅ OK")
            
        print("\n2. /initialpose topic:")
        print(f"   Received: {self.initialpose_received}")
        if not self.initialpose_received:
            print("   ⚠️  WARNING: No /initialpose messages (AMCL may not be initialized)")
        else:
            print("   ✅ OK")
            
        print("\n3. /cmd_vel topic:")
        print(f"   Received: {self.cmd_vel_received} (count: {self.cmd_vel_count})")
        if not self.cmd_vel_received:
            print("   ❌ PROBLEM: No /cmd_vel messages received!")
        else:
            print("   ✅ OK")
            
        print("\n4. /odom topic:")
        print(f"   Received: {self.odom_received} (count: {self.odom_count})")
        if not self.odom_received:
            print("   ❌ PROBLEM: No /odom messages received!")
        else:
            print("   ✅ OK")
            
        print("\n" + "="*60)
        print("RECOMMENDATIONS:")
        print("="*60)
        
        if not self.scan_received:
            print("- Check symovo_scan_converter node is running")
            print("- Check Symovo API /v0/agv/{id}/scan.png endpoint")
            print("- Check network connectivity to Symovo API")
            
        if not self.initialpose_received:
            print("- Publish /initialpose to initialize AMCL")
            print("- Check initial_pose_from_symovo node is running")
            
        if not self.odom_received:
            print("- Check base_controller node is running")
            print("- Check Symovo API /v0/agv endpoint")
            
        if not self.cmd_vel_received:
            print("- Check Nav2 controller_server is active")
            print("- Check velocity_smoother is active")
            
        print()

def main():
    rclpy.init()
    node = Nav2Diagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_report()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
