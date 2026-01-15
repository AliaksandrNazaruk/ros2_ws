#!/usr/bin/env python3
"""
Check for duplicate ROS2 nodes and provide cleanup recommendations
"""

import rclpy
from rclpy.node import Node
import sys


def main():
    rclpy.init()
    
    try:
        node = Node('duplicate_checker')
        
        # Get all nodes
        all_nodes = node.get_node_names()
        
        # Count duplicates
        from collections import Counter
        node_counts = Counter(all_nodes)
        
        duplicates = {name: count for name, count in node_counts.items() if count > 1}
        
        print("="*70)
        print("ROS2 Node Duplicate Check")
        print("="*70)
        print()
        
        if duplicates:
            print("⚠️  DUPLICATE NODES DETECTED:")
            print()
            for node_name, count in sorted(duplicates.items(), key=lambda x: x[1], reverse=True):
                print(f"  {node_name:50s} : {count} instances")
            print()
            print("⚠️  This can cause:")
            print("   - Topic conflicts (multiple publishers/subscribers)")
            print("   - Service conflicts")
            print("   - Lifecycle management issues")
            print("   - Unpredictable behavior")
            print()
            print("💡 RECOMMENDATION:")
            print("   1. Stop all launch processes")
            print("   2. Kill duplicate processes manually if needed")
            print("   3. Restart with single launch file")
            print()
        else:
            print("✅ No duplicate nodes found")
            print()
        
        print(f"Total unique node names: {len(node_counts)}")
        print(f"Total node instances: {sum(node_counts.values())}")
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
