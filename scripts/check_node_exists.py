#!/usr/bin/env python3
"""
Check if a ROS2 node exists before launching
Used in launch files to prevent duplicate nodes
"""

import rclpy
from rclpy.node import Node
import sys
import time


def node_exists(node_name: str, timeout: float = 2.0) -> bool:
    """
    Check if a node with the given name exists in the ROS2 graph
    
    Args:
        node_name: Name of the node to check (with or without leading /)
        timeout: Maximum time to wait for ROS2 to initialize
    
    Returns:
        True if node exists, False otherwise
    """
    # Normalize node name (remove leading / if present)
    normalized_name = node_name.lstrip('/')
    
    try:
        rclpy.init()
        node = Node('node_existence_checker')
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                all_nodes = node.get_node_names()
                # Check both with and without leading /
                if normalized_name in all_nodes or f'/{normalized_name}' in all_nodes:
                    rclpy.shutdown()
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        
        rclpy.shutdown()
        return False
        
    except Exception as e:
        print(f"Error checking node existence: {e}", file=sys.stderr)
        try:
            rclpy.shutdown()
        except:
            pass
        return False


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: check_node_exists.py <node_name>")
        sys.exit(1)
    
    node_name = sys.argv[1]
    exists = node_exists(node_name)
    sys.exit(0 if exists else 1)
