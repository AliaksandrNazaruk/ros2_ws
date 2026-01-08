#!/usr/bin/env python3

"""
MQTT → Nav2 Navigation Adapter

Subscribes to: aroc/robot/{ROBOT_ID}/commands/navigateTo
Validates command, resolves target_id → pose, sends Nav2 goal.

AE.HUB MVP requirement: Nav2 is NOT autonomous.
Nav2 is a subordinate executor of AE.HUB commands.
"""

import rclpy
from rclpy.node import Node
import json
import uuid
import os
from datetime import datetime
from typing import Optional

import paho.mqtt.client as mqtt
import threading

from aehub_navigation.navigation_state_manager import NavigationStateManager, NavigationState
from aehub_navigation.position_registry import PositionRegistry
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class NavigationCommandHandler(Node):
    def __init__(self):
        super().__init__('navigation_command_handler')
        
        # Parameters
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('positions_file', 'config/positions.yaml')
        
        self.robot_id = self.get_parameter('robot_id').value
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.positions_file = self.get_parameter('positions_file').value
        
        # Components
        self.state_manager = NavigationStateManager()
        self.position_registry = PositionRegistry()
        
        # Load positions
        # Try to resolve path
        positions_path = self.positions_file
        if not os.path.isabs(positions_path):
            from ament_index_python.packages import get_package_share_directory
            try:
                pkg_dir = get_package_share_directory('aehub_navigation')
                positions_path = os.path.join(pkg_dir, 'config', 'positions.yaml')
            except:
                # Fallback
                positions_path = self.positions_file
        
        if not self.position_registry.loadFromYAML(positions_path):
            self.get_logger().error(f'Failed to load positions from {positions_path}')
            raise RuntimeError('Failed to load positions')
        
        # MQTT client
        self.mqtt_client = mqtt.Client(client_id=f"nav_handler_{self.robot_id}")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Nav2 Action Client
        self.nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav2_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 action server not available, will retry on goal send')
        
        # Set state manager callbacks
        self.state_manager.setStateChangeCallback(self.on_state_change)
        
        # Current goal tracking
        self.current_goal_future = None
        self.current_target_id = None
        self.current_goal_handle = None
        
        self.get_logger().info(
            f'NavigationCommandHandler initialized: '
            f'robot_id={self.robot_id}, broker={self.mqtt_broker}:{self.mqtt_port}'
        )
    
    def cancelCurrentGoal(self):
        """Cancel current Nav2 goal"""
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.state_manager.onGoalCanceled(self.current_target_id)
            self.current_goal_handle = None
            self.current_target_id = None
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.state_manager.onGoalAborted(
                self.current_target_id,
                'GOAL_REJECTED',
                'Nav2 rejected the goal'
            )
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        self.current_goal_handle = goal_handle
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2"""
        feedback = feedback_msg.feedback
        # Can be used for progress calculation
        # For now, just log
        self.get_logger().debug(f'Nav2 feedback: distance_remaining={feedback.distance_remaining:.2f}')
    
    def result_callback(self, future):
        """Handle result from Nav2"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation goal succeeded')
            self.state_manager.onGoalSucceeded(self.current_target_id)
        elif status == 2:  # CANCELED
            self.get_logger().info('Navigation goal canceled')
            self.state_manager.onGoalCanceled(self.current_target_id)
        else:  # ABORTED or other
            self.get_logger().warn(f'Navigation goal aborted (status={status})')
            self.state_manager.onGoalAborted(
                self.current_target_id,
                'NAV_GOAL_ABORTED',
                f'Nav2 goal aborted with status {status}'
            )
        
        self.current_goal_handle = None
        self.current_target_id = None
    
    def cancel_done_callback(self, future):
        """Handle cancel completion"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancel request accepted')
        else:
            self.get_logger().warn('Goal cancel request rejected')
    
    def on_state_change(self, state, target_id, error_code, error_message):
        """Callback for state changes"""
        self.get_logger().info(
            f'State changed: {state.value}, target_id={target_id}, '
            f'error={error_code}:{error_message}'
        )
    
    def validateCommand(self, command: dict) -> tuple[bool, Optional[str]]:
        """
        Validate navigation command
        
        Returns: (is_valid, error_message)
        """
        required_fields = ['command_id', 'timestamp', 'target_id', 'priority']
        
        for field in required_fields:
            if field not in command:
                return (False, f'Missing required field: {field}')
        
        # Validate target_id exists in registry
        target_id = command['target_id']
        if not self.position_registry.hasPosition(target_id):
            return (False, f'Unknown target_id: {target_id}')
        
        # Validate priority (optional, but should be present)
        priority = command.get('priority', 'normal')
        if priority not in ['low', 'normal', 'high', 'emergency']:
            return (False, f'Invalid priority: {priority}')
        
        return (True, None)
    
    def handleNavigationCommand(self, command: dict):
        """
        Handle navigation command from MQTT
        
        Command format:
        {
            "command_id": "uuid-v4",
            "timestamp": "ISO-8601",
            "target_id": "position_A",
            "priority": "normal"
        }
        """
        # Validate command
        is_valid, error_msg = self.validateCommand(command)
        if not is_valid:
            self.get_logger().error(f'Invalid command: {error_msg}')
            # Publish error status via MQTT
            return
        
        target_id = command['target_id']
        command_id = command['command_id']
        
        self.get_logger().info(
            f'Received navigation command: command_id={command_id}, '
            f'target_id={target_id}'
        )
        
        # Resolve target_id → pose
        pose = self.position_registry.getPosition(target_id)
        if pose is None:
            self.get_logger().error(f'Failed to resolve target_id: {target_id}')
            return
        
        # Cancel active goal if navigating
        if self.state_manager.getState() == NavigationState.NAVIGATING:
            self.get_logger().info('Cancelling current goal')
            self.cancelCurrentGoal()
        
        # Send new Nav2 goal
        self.get_logger().info(
            f'Sending Nav2 goal: target_id={target_id}, '
            f'pose=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        # Update pose timestamp
        from rclpy.clock import Clock
        pose.header.stamp = Clock().now().to_msg()
        
        # Send goal via Nav2 Action Client
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        send_goal_future = self.nav2_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Store future and target_id for result handling
        self.current_goal_future = send_goal_future
        self.current_target_id = target_id
        
        # Set up result callback
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.state_manager.onGoalSent(target_id)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            topic = f'aroc/robot/{self.robot_id}/commands/navigateTo'
            client.subscribe(topic, qos=1)
            self.get_logger().info(f'Subscribed to: {topic}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker: {rc}')
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload_str = msg.payload.decode('utf-8')
            payload = json.loads(payload_str)
            self.handleNavigationCommand(payload)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse MQTT message: {e}, payload: {msg.payload}')
        except Exception as e:
            self.get_logger().error(f'Error handling MQTT message: {e}', exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommandHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        if node.mqtt_client:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

