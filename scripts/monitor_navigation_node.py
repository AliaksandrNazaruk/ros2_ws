#!/usr/bin/env python3
"""
Монandторandнг логов navigation_integrated_node в реальном временand
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import sys
import time

class LogMonitor(Node):
    def __init__(self):
        super().__init__('log_monitor')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            10
        )
        self.filter_keywords = [
            'navigation_integrated_node',
            'command',
            'goal',
            'status',
            'error',
            'block',
            'stop',
            'emergency',
            'base'
        ]
    
    def log_callback(self, msg):
        # Фandльтруем только логand от navigation_integrated_node
        if 'navigation_integrated_node' in msg.name:
            # Проверяем налandчandе ключевых слов
            msg_text = msg.msg.lower()
            if any(keyword.lower() in msg_text for keyword in self.filter_keywords):
                level = msg.level
                level_str = {1: 'DEBUG', 2: 'INFO', 4: 'WARN', 8: 'ERROR', 16: 'FATAL'}.get(level, 'UNKNOWN')
                print(f"[{level_str}] {msg.msg}")

def main():
    rclpy.init()
    monitor = LogMonitor()
    
    print("=" * 70)
    print("МОНANDТОРANDНГ ЛОГОВ navigation_integrated_node")
    print("=" * 70)
    print("Waiting логов... (Ctrl+C для выхоyes)")
    print("=" * 70)
    print()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nОстановка монandторandнга...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

