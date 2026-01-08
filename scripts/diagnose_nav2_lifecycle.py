#!/usr/bin/env python3
"""
Diagnostic script for Nav2 lifecycle nodes

Checks the state of all lifecycle nodes and identifies why action server might not be available.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
import sys


class Nav2LifecycleDiagnostic(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_diagnostic')
        
    def get_lifecycle_node_state(self, node_name: str) -> tuple:
        """
        Get state of a lifecycle node
        
        Returns:
            (state_id, state_label) or (None, None) if service not available
        """
        service_name = f'{node_name}/get_state'
        client = self.create_client(GetState, service_name)
        
        if not client.wait_for_service(timeout_sec=2.0):
            return (None, 'SERVICE_NOT_AVAILABLE')
        
        request = GetState.Request()
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response:
                    state_id = response.current_state.id
                    state_label = response.current_state.label
                    return (state_id, state_label)
        except Exception as e:
            self.get_logger().error(f'Error getting state for {node_name}: {e}')
        
        return (None, 'ERROR')
    
    def diagnose(self):
        """Run diagnostics on all Nav2 lifecycle nodes"""
        lifecycle_nodes = [
            'controller_server',
            'planner_server',
            'recoveries_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
            'local_costmap',
            'global_costmap',
        ]
        
        print("=" * 70)
        print("Nav2 Lifecycle Nodes Diagnostic")
        print("=" * 70)
        print()
        
        results = {}
        all_active = True
        
        for node_name in lifecycle_nodes:
            state_id, state_label = self.get_lifecycle_node_state(node_name)
            results[node_name] = (state_id, state_label)
            
            if state_id is None:
                status = " SERVICE NOT AVAILABLE"
                all_active = False
            elif state_id == State.PRIMARY_STATE_ACTIVE:
                status = " ACTIVE"
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                status = "  INACTIVE"
                all_active = False
            elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
                status = " UNCONFIGURED"
                all_active = False
            else:
                status = f"  {state_label}"
                if state_id != State.PRIMARY_STATE_ACTIVE:
                    all_active = False
            
            print(f"{node_name:30s} {status:30s} (id={state_id}, label={state_label})")
        
        print()
        print("=" * 70)
        
        # Check bt_navigator specifically (creates action server)
        bt_state_id, bt_state_label = results.get('bt_navigator', (None, None))
        if bt_state_id == State.PRIMARY_STATE_ACTIVE:
            print(" bt_navigator is ACTIVE - action server should be available")
        else:
            print(f" bt_navigator is NOT ACTIVE (state: {bt_state_label})")
            print("   This is why action server is not available!")
            print()
            print("   To activate bt_navigator:")
            print("   1. Check lifecycle_manager_navigation is running")
            print("   2. Check that autostart=true in launch file")
            print("   3. Manually activate via lifecycle service if needed")
        
        print()
        print("=" * 70)
        
        if all_active:
            print(" All lifecycle nodes are ACTIVE")
        else:
            print("  Some lifecycle nodes are NOT ACTIVE")
            print("   Nav2 action server requires bt_navigator to be ACTIVE")
        
        return results


def main():
    rclpy.init()
    
    try:
        diagnostic = Nav2LifecycleDiagnostic()
        results = diagnostic.diagnose()
        
        # Check action server availability
        print()
        print("=" * 70)
        print("Action Server Check")
        print("=" * 70)
        
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        
        action_client = ActionClient(diagnostic, NavigateToPose, 'navigate_to_pose')
        
        if action_client.wait_for_server(timeout_sec=2.0):
            print(" /navigate_to_pose action server is AVAILABLE")
        else:
            print(" /navigate_to_pose action server is NOT AVAILABLE")
            print("   This is likely because bt_navigator is not ACTIVE")
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

