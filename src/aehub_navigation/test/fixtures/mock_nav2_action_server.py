#!/usr/bin/env python3

"""
Mock Nav2 Action Server for testing

Provides a mock implementation of Nav2 NavigateToPose action server
for use in integration and system tests.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import PoseStamped
import threading
import time


class MockNav2ActionServer(Node):
    """
    Mock Nav2 Action Server for testing.
    
    Simulates Nav2 NavigateToPose action server behavior.
    """
    
    def __init__(self):
        super().__init__('mock_nav2_action_server')
        
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback
        )
        
        self.current_goal_handle = None
        self.auto_succeed = True  # Automatically succeed goals
        self.auto_succeed_delay = 0.5  # Delay before succeeding
        self.reject_next_goal = False  # Flag to reject next goal
        self.abort_next_goal = False  # Flag to abort next goal
        
        self.get_logger().info('MockNav2ActionServer initialized')
    
    def goal_callback(self, goal_request):
        """Handle goal request"""
        if self.reject_next_goal:
            self.reject_next_goal = False
            self.get_logger().info('Rejecting goal (test mode)')
            return rclpy.action.server.GoalResponse.REJECT
        
        self.get_logger().info('Goal accepted')
        return rclpy.action.server.GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute goal"""
        self.current_goal_handle = goal_handle
        goal = goal_handle.request
        
        self.get_logger().info(f'Executing goal: pose=({goal.pose.pose.position.x}, {goal.pose.pose.position.y})')
        
        # Send feedback
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.current_pose = goal.pose
        feedback_msg.distance_remaining = 0.5
        goal_handle.publish_feedback(feedback_msg)
        
        # Check if should abort
        if self.abort_next_goal:
            self.abort_next_goal = False
            self.get_logger().info('Aborting goal (test mode)')
            goal_handle.abort()
            result = NavigateToPose.Result()
            return result
        
        # Wait a bit (simulate navigation)
        time.sleep(self.auto_succeed_delay)
        
        # Succeed goal
        if self.auto_succeed:
            self.get_logger().info('Goal succeeded (test mode)')
            goal_handle.succeed()
            result = NavigateToPose.Result()
            return result
        else:
            # Keep goal active (for cancel testing)
            while rclpy.ok():
                time.sleep(0.1)
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled (test mode)')
                    goal_handle.canceled()
                    result = NavigateToPose.Result()
                    return result
    
    def set_reject_next_goal(self, reject=True):
        """Set flag to reject next goal"""
        self.reject_next_goal = reject
    
    def set_abort_next_goal(self, abort=True):
        """Set flag to abort next goal"""
        self.abort_next_goal = abort
    
    def set_auto_succeed(self, auto_succeed=True, delay=0.5):
        """Set auto-succeed behavior"""
        self.auto_succeed = auto_succeed
        self.auto_succeed_delay = delay


def main(args=None):
    rclpy.init(args=args)
    server = MockNav2ActionServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

