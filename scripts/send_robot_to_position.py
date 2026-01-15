#!/usr/bin/env python3
"""
Send robot to any position on the map
Supports both predefined positions and custom coordinates
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math
import sys
import yaml
import os

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        
        # Action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber for current pose
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.goal_handle = None
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def get_current_pose(self, timeout=5.0):
        """Get current robot pose"""
        start_time = time.time()
        while self.current_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose
    
    def wait_for_server(self, timeout=10.0):
        """Wait for Nav2 action server to be available"""
        print(f"⏳ Waiting for Nav2 action server...")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"❌ Nav2 action server not available after {timeout}s")
            return False
        self.get_logger().info("✅ Nav2 action server is ready")
        return True
    
    def send_goal(self, x, y, yaw=0.0, frame_id="map"):
        """Send navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.get_logger().info(
            f"🎯 Sending goal: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°"
        )
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected!")
            return False
        
        self.get_logger().info("✅ Goal accepted, navigating...")
        return True
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from navigation"""
        feedback = feedback_msg.feedback
        current = feedback.current_pose.pose.position
        
        print(
            f"  📍 Current: x={current.x:.3f}, y={current.y:.3f}, "
            f"Distance: {feedback.distance_remaining:.3f}m"
        )
    
    def wait_for_result(self, timeout=120.0):
        """Wait for navigation to complete"""
        if not self.goal_handle:
            return False
        
        result_future = self.goal_handle.get_result_async()
        
        start_time = time.time()
        last_feedback_time = start_time
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Print progress every 2 seconds
            if time.time() - last_feedback_time >= 2.0:
                elapsed = time.time() - start_time
                print(f"  ⏱️  Elapsed: {elapsed:.1f}s")
                last_feedback_time = time.time()
            
            if result_future.done():
                result = result_future.result().result
                status = result_future.result().status
                
                if status == 4:  # SUCCEEDED
                    self.get_logger().info("✅ Navigation completed successfully!")
                    return True
                else:
                    status_names = {
                        1: "ABORTED",
                        2: "REJECTED", 
                        3: "CANCELED",
                        4: "SUCCEEDED"
                    }
                    status_name = status_names.get(status, f"UNKNOWN({status})")
                    self.get_logger().warn(f"⚠️ Navigation ended with status: {status_name}")
                    return False
        
        self.get_logger().error(f"❌ Navigation timeout after {timeout}s")
        return False

def load_positions(file_path):
    """Load predefined positions from YAML file"""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            return data.get('positions', {})
    except Exception as e:
        print(f"⚠️  Could not load positions file: {e}")
        return {}

def list_positions(positions):
    """List available predefined positions"""
    if not positions:
        print("  No predefined positions available")
        return
    
    print("\n  Available predefined positions:")
    for name, pos in positions.items():
        print(f"    - {name}: x={pos['x']:.3f}, y={pos['y']:.3f}, theta={pos.get('theta', 0.0):.3f}")
        if 'description' in pos:
            print(f"      {pos['description']}")

def main():
    rclpy.init()
    
    navigator = RobotNavigator()
    
    print("="*70)
    print("🤖 Send Robot to Position on Map")
    print("="*70)
    
    # Wait for Nav2 server
    if not navigator.wait_for_server(timeout=10.0):
        print("\n❌ Cannot connect to Nav2. Make sure Nav2 is running.")
        rclpy.shutdown()
        return
    
    # Get current pose
    print("\n📊 Getting current robot position...")
    current_pose = navigator.get_current_pose(timeout=5.0)
    if current_pose:
        print(f"  Current position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}")
    else:
        print("  ⚠️  Could not get current position (using map frame)")
    
    # Load predefined positions
    positions_file = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'src', 'aehub_navigation', 'config', 'positions.yaml'
    )
    positions = load_positions(positions_file)
    
    # Parse command line arguments
    goal_x = None
    goal_y = None
    goal_yaw = 0.0
    position_name = None
    
    if len(sys.argv) >= 3:
        # Custom coordinates provided
        try:
            goal_x = float(sys.argv[1])
            goal_y = float(sys.argv[2])
            goal_yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
        except ValueError:
            print("❌ Invalid coordinates. Usage:")
            print("   python3 send_robot_to_position.py [x] [y] [yaw]")
            print("   python3 send_robot_to_position.py [position_name]")
            rclpy.shutdown()
            return
    elif len(sys.argv) == 2:
        # Position name provided
        position_name = sys.argv[1]
        if position_name in positions:
            pos = positions[position_name]
            goal_x = pos['x']
            goal_y = pos['y']
            goal_yaw = pos.get('theta', 0.0)
            print(f"\n📋 Using predefined position: {position_name}")
            if 'description' in pos:
                print(f"   Description: {pos['description']}")
        else:
            print(f"❌ Position '{position_name}' not found in positions file")
            list_positions(positions)
            rclpy.shutdown()
            return
    else:
        # Interactive mode
        print("\n📋 Available options:")
        print("  1. Use predefined position")
        print("  2. Enter custom coordinates")
        print("")
        list_positions(positions)
        print("\n💡 Usage examples:")
        print("  python3 send_robot_to_position.py position_A")
        print("  python3 send_robot_to_position.py 1.5 1.5 0.0")
        print("  python3 send_robot_to_position.py 2.0 0.5")
        rclpy.shutdown()
        return
    
    # Send goal
    print(f"\n🎯 Sending navigation goal...")
    print(f"   Target: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.1f}°")
    
    if not navigator.send_goal(goal_x, goal_y, goal_yaw):
        rclpy.shutdown()
        return
    
    # Wait for result
    print("\n⏳ Navigating to goal...")
    print("   (Press Ctrl+C to cancel)")
    print("")
    success = navigator.wait_for_result(timeout=120.0)
    
    # Results
    print("\n" + "="*70)
    if success:
        print("✅ NAVIGATION SUCCESSFUL!")
        print(f"   Robot arrived at: x={goal_x:.3f}, y={goal_y:.3f}")
    else:
        print("❌ NAVIGATION FAILED")
        print("   Check Nav2 logs and robot status")
    print("="*70)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
