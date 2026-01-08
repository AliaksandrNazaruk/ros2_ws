#!/usr/bin/env python3
"""
Comprehensive test script for full navigation flow

Tests the complete flow from MQTT command to base controller, including:
- MQTT connection
- Nav2 action server availability
- Command processing
- Status monitoring
- /cmd_vel verification
- Base blocking error handling
"""

import sys
import os
import asyncio
import json
import time
import argparse
from datetime import datetime, timezone
from typing import Optional, Dict, Any
import logging

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# Add paths for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
nav2_test_server_dir = os.path.join(script_dir, "nav2_test_server")
sys.path.insert(0, nav2_test_server_dir)
sys.path.insert(0, script_dir)

# Import modules
import importlib.util

# Load config module first (needed by mqtt_client)
config_path = os.path.join(nav2_test_server_dir, "config.py")
spec_config = importlib.util.spec_from_file_location("nav2_test_server.config", config_path)
config_module = importlib.util.module_from_spec(spec_config)
sys.modules['nav2_test_server'] = type(sys)('nav2_test_server')
sys.modules['nav2_test_server.config'] = config_module
spec_config.loader.exec_module(config_module)

# Load mqtt_client module (depends on config)
mqtt_client_path = os.path.join(nav2_test_server_dir, "mqtt_client.py")
spec_mqtt = importlib.util.spec_from_file_location("nav2_test_server.mqtt_client", mqtt_client_path)
mqtt_client_module = importlib.util.module_from_spec(spec_mqtt)
sys.modules['nav2_test_server.mqtt_client'] = mqtt_client_module
spec_mqtt.loader.exec_module(mqtt_client_module)

# Import classes
MQTTTestClient = mqtt_client_module.MQTTTestClient
Config = config_module.Config

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node as ROS2Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    logger.warning("ROS2 not available, some checks will be skipped")
    ROS2_AVAILABLE = False


class NavigationFlowTester:
    """Comprehensive navigation flow tester"""
    
    def __init__(self, target_id: str = "position_B", timeout: float = 30.0):
        self.target_id = target_id
        self.timeout = timeout
        
        # Load config with explicit env file path
        env_file = os.path.join(script_dir, "nav2_test_server", ".env")
        if os.path.exists(env_file):
            self.config = Config(_env_file=env_file)
        else:
            # Try to load from current directory
            self.config = Config()
        
        self.mqtt_client = None
        self.status_messages = []
        self.test_results = {
            'mqtt_connection': False,
            'nav2_action_server': False,
            'command_sent': False,
            'command_received': False,
            'status_received': False,
            'cmd_vel_active': False,
            'errors': []
        }
        
    def log_step(self, step_num: int, description: str, status: str = "ok"):
        """Log a test step"""
        icon = "" if status == "ok" else "" if status == "error" else ""
        logger.info(f"{icon} Step {step_num}: {description}")
    
    def check_mqtt_connection(self) -> bool:
        """Step 1: Check MQTT connection"""
        self.log_step(1, "Checking MQTT connection")
        try:
            self.mqtt_client = MQTTTestClient(self.config)
            if self.mqtt_client.connect():
                self.log_step(1, "MQTT connection", "ok")
                logger.info(f"   Connected to: {self.mqtt_client.client._host}:{self.mqtt_client.client._port}")
                self.test_results['mqtt_connection'] = True
                return True
            else:
                self.log_step(1, "MQTT connection", "error")
                self.test_results['errors'].append("Failed to connect to MQTT")
                return False
        except Exception as e:
            self.log_step(1, "MQTT connection", "error")
            logger.error(f"   Error: {e}")
            self.test_results['errors'].append(f"MQTT connection error: {e}")
            return False
    
    def check_nav2_action_server(self) -> bool:
        """Step 2: Check Nav2 action server"""
        self.log_step(2, "Checking Nav2 action server")
        if not ROS2_AVAILABLE:
            logger.warning("   ROS2 not available, skipping check")
            return False
        
        try:
            rclpy.init()
            temp_node = rclpy.create_node('nav2_checker')
            action_client = ActionClient(temp_node, NavigateToPose, 'navigate_to_pose')
            
            if action_client.wait_for_server(timeout_sec=5.0):
                self.log_step(2, "Nav2 action server", "ok")
                logger.info("   Action server is available")
                self.test_results['nav2_action_server'] = True
                temp_node.destroy_node()
                return True
            else:
                self.log_step(2, "Nav2 action server", "error")
                logger.warning("   Action server is NOT available")
                logger.warning("   Run: python3 scripts/diagnose_nav2_lifecycle.py")
                self.test_results['errors'].append("Nav2 action server not available")
                temp_node.destroy_node()
                return False
        except Exception as e:
            self.log_step(2, "Nav2 action server", "error")
            logger.error(f"   Error: {e}")
            self.test_results['errors'].append(f"Nav2 check error: {e}")
            return False
    
    def send_navigation_command(self) -> Optional[str]:
        """Step 3: Send navigation command"""
        self.log_step(3, "Sending navigation command")
        try:
            command_id = self.mqtt_client.send_navigate_command(self.target_id, 'normal')
            self.log_step(3, "Navigation command sent", "ok")
            logger.info(f"   command_id: {command_id}")
            logger.info(f"   target_id: {self.target_id}")
            self.test_results['command_sent'] = True
            return command_id
        except Exception as e:
            self.log_step(3, "Navigation command", "error")
            logger.error(f"   Error: {e}")
            self.test_results['errors'].append(f"Command send error: {e}")
            return None
    
    def wait_for_status(self, timeout: float) -> bool:
        """Step 4: Wait for navigation status"""
        self.log_step(4, f"Waiting for navigation status (timeout: {timeout}s)")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.status_messages:
                status = self.status_messages[-1]
                self.log_step(4, "Navigation status received", "ok")
                try:
                    data = json.loads(status.payload.decode())
                    logger.info(f"   Status: {data.get('status')}")
                    logger.info(f"   Target: {data.get('target_id')}")
                    logger.info(f"   Progress: {data.get('progress_percent')}%")
                    if data.get('error_code'):
                        logger.warning(f"   Error: {data.get('error_code')} - {data.get('error_message')}")
                    self.test_results['status_received'] = True
                    return True
                except:
                    logger.info(f"   Raw status: {status.payload.decode()[:200]}")
                    self.test_results['status_received'] = True
                    return True
            time.sleep(0.5)
        
        self.log_step(4, "Navigation status", "error")
        logger.warning(f"   No status received within {timeout}s")
        self.test_results['errors'].append("Status timeout")
        return False
    
    def check_cmd_vel(self) -> bool:
        """Step 5: Check /cmd_vel topic"""
        self.log_step(5, "Checking /cmd_vel topic")
        if not ROS2_AVAILABLE:
            logger.warning("   ROS2 not available, skipping check")
            return False
        
        try:
            rclpy.init()
            temp_node = rclpy.create_node('cmd_vel_checker')
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            
            cmd_vel_received = [False]
            cmd_vel_data = [None]
            
            def cmd_vel_callback(msg):
                cmd_vel_received[0] = True
                cmd_vel_data[0] = {
                    'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
                    'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
                }
            
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            sub = temp_node.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, qos)
            
            # Wait for messages
            start_time = time.time()
            while (time.time() - start_time) < 5.0:
                rclpy.spin_once(temp_node, timeout_sec=0.5)
                if cmd_vel_received[0]:
                    self.log_step(5, "/cmd_vel topic", "ok")
                    logger.info(f"   Received cmd_vel: linear.x={cmd_vel_data[0]['linear']['x']:.3f}, angular.z={cmd_vel_data[0]['angular']['z']:.3f}")
                    self.test_results['cmd_vel_active'] = True
                    temp_node.destroy_node()
                    return True
            
            self.log_step(5, "/cmd_vel topic", "error")
            logger.warning("   No cmd_vel messages received (base may be blocked)")
            temp_node.destroy_node()
            return False
        except Exception as e:
            self.log_step(5, "/cmd_vel topic", "error")
            logger.error(f"   Error: {e}")
            self.test_results['errors'].append(f"cmd_vel check error: {e}")
            return False
    
    def on_status_message(self, client, userdata, msg):
        """MQTT status message callback"""
        self.status_messages.append(msg)
        logger.info(f" Status received: {msg.topic}")
    
    def run_test(self) -> Dict[str, Any]:
        """Run complete test flow"""
        logger.info("=" * 70)
        logger.info("COMPREHENSIVE NAVIGATION FLOW TEST")
        logger.info("=" * 70)
        logger.info(f"Target position: {self.target_id}")
        logger.info(f"Timeout: {self.timeout}s")
        logger.info("=" * 70)
        logger.info("")
        
        # Step 1: MQTT connection
        if not self.check_mqtt_connection():
            return self.test_results
        
        # Setup status subscription
        self.mqtt_client.client.subscribe(f"aroc/robot/{self.config.robot_id}/status/navigation", qos=1)
        self.mqtt_client.client.on_message = self.on_status_message
        
        # Step 2: Nav2 action server
        self.check_nav2_action_server()
        
        # Step 3: Send command
        command_id = self.send_navigation_command()
        if not command_id:
            return self.test_results
        
        # Step 4: Wait for status
        self.wait_for_status(self.timeout)
        
        # Step 5: Check cmd_vel
        self.check_cmd_vel()
        
        # Summary
        logger.info("")
        logger.info("=" * 70)
        logger.info("TEST SUMMARY")
        logger.info("=" * 70)
        logger.info(f"MQTT Connection: {'' if self.test_results['mqtt_connection'] else ''}")
        logger.info(f"Nav2 Action Server: {'' if self.test_results['nav2_action_server'] else ''}")
        logger.info(f"Command Sent: {'' if self.test_results['command_sent'] else ''}")
        logger.info(f"Status Received: {'' if self.test_results['status_received'] else ''}")
        logger.info(f"cmd_vel Active: {'' if self.test_results['cmd_vel_active'] else ''}")
        
        if self.test_results['errors']:
            logger.info("")
            logger.info("Errors:")
            for error in self.test_results['errors']:
                logger.info(f"  - {error}")
        
        logger.info("=" * 70)
        
        return self.test_results


def main():
    parser = argparse.ArgumentParser(description='Comprehensive navigation flow test')
    parser.add_argument('--target', default='position_B', help='Target position ID')
    parser.add_argument('--timeout', type=float, default=30.0, help='Status timeout in seconds')
    
    args = parser.parse_args()
    
    tester = NavigationFlowTester(target_id=args.target, timeout=args.timeout)
    results = tester.run_test()
    
    # Cleanup
    if tester.mqtt_client:
        tester.mqtt_client.disconnect()
    
    if ROS2_AVAILABLE and rclpy.ok():
        rclpy.shutdown()
    
    # Exit code
    if results['status_received'] and results['command_sent']:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()

