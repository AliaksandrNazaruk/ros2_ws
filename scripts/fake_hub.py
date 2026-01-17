#!/usr/bin/env python3
"""
Fake AE.HUB - MQTT Command Sender and Status Monitor

Simulator of AE.HUB for robot testing:
- Gets MQTT configuration from Config Service (http://localhost:7900)
- Connects to MQTT broker
- Sends navigateTo commands to robot
- Subscribes to statuses from robot
- Logs all interactions

Usage:
    python3 scripts/fake_hub.py --robot-id fahrdummy-01 --target-id position_A
    python3 scripts/fake_hub.py --robot-id fahrdummy-01 --interactive
"""

import argparse
import json
import time
import uuid
import sys
import threading
from datetime import datetime, timezone
from typing import Optional, Dict, Any
import requests
import paho.mqtt.client as mqtt
import ssl


class FakeHub:
    """Fake AE.HUB for robot testing"""
    
    def __init__(self, config_service_url: str, api_key: str, robot_id: str):
        """
        Initialize fake hub
        
        Args:
            config_service_url: URL Config Service (http://localhost:7900)
            api_key: API key for Config Service
            robot_id: Robot ID for testing
        """
        self.config_service_url = config_service_url.rstrip('/')
        self.api_key = api_key
        self.robot_id = robot_id
        
        self.mqtt_client: Optional[mqtt.Client] = None
        self.mqtt_config: Optional[Dict[str, Any]] = None
        
        # Topics
        self.command_topic = f"aroc/robot/{robot_id}/commands/navigateTo"
        self.cancel_topic = f"aroc/robot/{robot_id}/commands/cancel"
        self.status_topic = f"aroc/robot/{robot_id}/status/navigation"
        self.events_topic = f"aroc/robot/{robot_id}/commands/events"
        
        # Statistics
        self.commands_sent = 0
        self.statuses_received = 0
        self.last_status: Optional[Dict[str, Any]] = None
        
        # Track statuses by command_id
        self.status_by_command_id: Dict[str, Dict[str, Any]] = {}
        # Track command events by command_id (ack/result)
        self.events_by_command_id: Dict[str, Dict[str, Any]] = {}
        # First fixed status with error by command_id (do not overwrite)
        self.error_status_by_command_id: Dict[str, Dict[str, Any]] = {}
        # Timing (measured on the hub side, monotonic clock, in seconds)
        # command_id -> times
        self._timing_by_command_id: Dict[str, Dict[str, float]] = {}
        self._status_lock = threading.Lock()

    def _now_mono(self) -> float:
        """Monotonic timestamp (seconds) for latency measurement."""
        return time.monotonic()

    def mark_command_sent(self, command_id: str) -> None:
        """Mark the hub-side 't0' when the command was published."""
        if not command_id:
            return
        with self._status_lock:
            d = self._timing_by_command_id.setdefault(command_id, {})
            d.setdefault("t0_sent", self._now_mono())

    def get_timing(self, command_id: str) -> Dict[str, float]:
        """Get recorded hub-side timing data for a command_id."""
        with self._status_lock:
            return dict(self._timing_by_command_id.get(command_id, {}))
    
    def fetch_mqtt_config(self) -> bool:
        """
        Get MQTT configuration from Config Service
        
        Returns:
            True if successful, False otherwise
        """
        url = f"{self.config_service_url}/api/v1/config/broker?include_password=true"
        headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }
        
        print(f" Fetching MQTT config from {url}...")
        
        try:
            response = requests.get(url, headers=headers, timeout=5.0)
            
            if response.status_code != 200:
                print(f" Failed to fetch config: HTTP {response.status_code}")
                print(f"   Response: {response.text}")
                return False
            
            self.mqtt_config = response.json()
            
            print(f" MQTT config fetched:")
            print(f"   Broker: {self.mqtt_config.get('broker')}:{self.mqtt_config.get('broker_port')}")
            print(f"   TLS: {self.mqtt_config.get('mqtt_use_tls', False)}")
            print(f"   User: {self.mqtt_config.get('mqtt_user', 'N/A')}")
            
            return True
            
        except requests.exceptions.RequestException as e:
            print(f" Error fetching config: {e}")
            return False
    
    def connect_mqtt(self) -> bool:
        """
        Connect to MQTT broker
        
        Returns:
            True if successful, False otherwise
        """
        if not self.mqtt_config:
            print(" MQTT config not fetched")
            return False
        
        broker = self.mqtt_config['broker']
        port = self.mqtt_config['broker_port']
        use_tls = self.mqtt_config.get('mqtt_use_tls', False)
        username = self.mqtt_config.get('mqtt_user', '')
        password = self.mqtt_config.get('mqtt_password', '')
        tls_insecure = self.mqtt_config.get('mqtt_tls_insecure', False)
        
        # Create MQTT client
        client_id = f"fake_hub_{uuid.uuid4().hex[:8]}"
        self.mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
        
        # Configure TLS if needed
        if use_tls:
            context = ssl.create_default_context()
            if tls_insecure:
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
            self.mqtt_client.tls_set_context(context)
        
        # Set credentials
        if username:
            self.mqtt_client.username_pw_set(username, password)
        
        # Callbacks
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # Connect
        print(f"🔌 Connecting to MQTT broker: {broker}:{port} (TLS: {use_tls})...")
        
        try:
            if use_tls:
                self.mqtt_client.connect(broker, port, keepalive=60)
            else:
                self.mqtt_client.connect(broker, port, keepalive=60)
            
            self.mqtt_client.loop_start()
            
            # Wait for connection
            timeout = 10.0
            start_time = time.time()
            while not self.mqtt_client.is_connected() and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.mqtt_client.is_connected():
                print(" MQTT connection timeout")
                return False
            
            print(" Connected to MQTT broker")
            return True
            
        except Exception as e:
            print(f" Failed to connect to MQTT: {e}")
            return False
    
    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connection callback"""
        if rc == 0:
            print(f" MQTT connected (rc={rc})")
            # Subscribe to statuses
            client.subscribe(self.status_topic, qos=1)
            print(f" Subscribed to: {self.status_topic}")
            # Subscribe to command events
            client.subscribe(self.events_topic, qos=1)
            print(f" Subscribed to: {self.events_topic}")
        else:
            print(f" MQTT connection failed (rc={rc})")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message received callback"""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            self.statuses_received += 1
            recv_t = self._now_mono()
            
            # Handle command events
            if msg.topic == self.events_topic:
                ev_type = payload.get('event_type')
                cmd_id = payload.get('command_id')
                if cmd_id:
                    with self._status_lock:
                        self.events_by_command_id[cmd_id] = payload
                        td = self._timing_by_command_id.setdefault(cmd_id, {})
                        if ev_type == 'ack':
                            ack_status = payload.get('ack_status')
                            if ack_status:
                                td.setdefault(f"t_ack_{ack_status}", recv_t)
                        elif ev_type == 'result':
                            result_status = payload.get('result_status')
                            if result_status:
                                td.setdefault(f"t_result_{result_status}", recv_t)
                    print(f"   🔔 EVENT [{ev_type}] command_id={cmd_id}, result={payload.get('result_status')}, ack={payload.get('ack_status')}, error={payload.get('error_code')}")
                return
            
            # Save status by command_id for tracking (for compatibility)
            command_id = payload.get('command_id')
            if command_id and command_id != 'N/A':
                with self._status_lock:
                    old_status = self.status_by_command_id.get(command_id, {}).get('status')
                    status_val_store = payload.get('status') or 'idle'
                    stored = dict(payload)
                    stored['status'] = status_val_store
                    self.status_by_command_id[command_id] = stored
                    td = self._timing_by_command_id.setdefault(command_id, {})
                    # Record first time we saw a given status for this command
                    if status_val_store:
                        td.setdefault(f"t_status_{status_val_store}", recv_t)
                    if stored.get('error_code') and command_id not in self.error_status_by_command_id:
                        self.error_status_by_command_id[command_id] = stored
                    print(f"    Saved status for command_id={command_id}: status={stored.get('status')}, error={stored.get('error_code')}, (was: {old_status})")
            
            self.last_status = payload
            print(f"\n STATUS RECEIVED [{self.statuses_received}]:")
            print(f"   Topic: {msg.topic}")
            status_val = payload.get('status')
            if status_val is None or status_val == 'N/A':
                status_val = 'idle'  # Default fallback
            print(f"   Status: {status_val}")
            print(f"   Target ID: {payload.get('target_id', 'N/A')}")
            print(f"   Command ID: {payload.get('command_id', 'N/A')}")
            print(f"   Progress: {payload.get('progress_percent', 0)}%")
            print(f"   ETA: {payload.get('eta_seconds', 0)}s")
            
            if payload.get('error_code'):
                print(f"     ERROR: {payload.get('error_code')} - {payload.get('error_message', 'N/A')}")
            
            if payload.get('current_position'):
                pos = payload['current_position']
                print(f"   Position: ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('theta', 0):.2f})")
            
            print()  # Empty line for readability
            
        except Exception as e:
            print(f" Error parsing status message: {e}")
            print(f"   Raw payload: {msg.payload}")
    
    def get_status_by_command_id(self, command_id: str, timeout: float = 5.0, wait_for_error: bool = True) -> Optional[Dict[str, Any]]:
        """
        Get status by command_id
        
        Args:
            command_id: Command ID
            timeout: Maximum wait time (seconds)
            wait_for_error: If True, wait for status with error_code (for errors)
        
        Returns:
            Status or None if not found
        """
        start_time = time.time()
        last_status = None
        found_error_status = None
        
        print(f"    Looking for status: command_id={command_id}, timeout={timeout}, wait_for_error={wait_for_error}")
        # If already fixed error earlier - return immediately
        with self._status_lock:
            if wait_for_error and command_id in self.error_status_by_command_id:
                res = self.error_status_by_command_id[command_id]
                print(f"    Found error status immediately: {res.get('status')}, error={res.get('error_code')}")
                return res
        
        while (time.time() - start_time) < timeout:
            with self._status_lock:
                if command_id in self.status_by_command_id:
                    status = self.status_by_command_id[command_id]
                    # Always save last status
                    last_status = status
                    # If waiting for error and found status with error_code, save it
                    if wait_for_error and status.get('error_code'):
                        found_error_status = status
                        print(f"    Found error status: {status.get('status')}, error={status.get('error_code')}")
                    # Also check if persisted error-status appeared earlier
                    if wait_for_error and command_id in self.error_status_by_command_id:
                        found_error_status = self.error_status_by_command_id[command_id]
                        print(f"    Found persisted error status: {found_error_status.get('status')}, error={found_error_status.get('error_code')}")
            time.sleep(0.1)
        
        result = found_error_status if found_error_status else last_status
        print(f"    Returning: status={result.get('status') if result else None}, error={result.get('error_code') if result else None}")
        return result
    
    def clear_status_history(self):
        """Clear status history"""
        with self._status_lock:
            self.status_by_command_id.clear()
            self.events_by_command_id.clear()
            self.error_status_by_command_id.clear()

    def wait_for_result_event(self, command_id: str, timeout: float = 30.0) -> Optional[Dict[str, Any]]:
        """
        Wait for terminal result event for a command_id.

        Returns the last seen event if timeout is reached.
        """
        start_time = time.time()
        last_event = None
        while (time.time() - start_time) < timeout:
            with self._status_lock:
                ev = self.events_by_command_id.get(command_id)
                if ev:
                    last_event = ev
                    if ev.get('event_type') == 'result':
                        return ev
            time.sleep(0.1)

        return last_event
    
    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnection callback"""
        print(f"  MQTT disconnected (rc={rc})")
    
    def send_navigate_command(self, target_id: str, priority: str = "normal") -> Optional[str]:
        """
        Send navigateTo command
        
        Args:
            target_id: Target position ID
            priority: Priority (normal, high, emergency)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.mqtt_client or not self.mqtt_client.is_connected():
            print(" MQTT not connected")
            return False
        
        # SPECIFICATION.md strict schema fields (Canonical v2.0)
        command = {
            "schema_version": "2.0",
            "robot_id": self.robot_id,
            "command_id": str(uuid.uuid4()),
            # Use Z-suffix ISO-8601 for consistency with SPECIFICATION.md
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "target_id": target_id,
            "priority": priority,
        }
        
        payload = json.dumps(command)
        
        print(f"\n SENDING COMMAND [{self.commands_sent + 1}]:")
        print(f"   Topic: {self.command_topic}")
        print(f"   Command ID: {command['command_id']}")
        print(f"   Target ID: {target_id}")
        print(f"   Priority: {priority}")
        print()
        
        result = self.mqtt_client.publish(self.command_topic, payload, qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.commands_sent += 1
            self.mark_command_sent(command['command_id'])
            print(f" Command sent successfully (mid={result.mid})")
            return command['command_id']  # Return command_id, not True
        else:
            print(f" Failed to send command (rc={result.rc})")
            return None  # Return None on failure

    def send_navigate_pose_command(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        priority: str = "normal",
        frame_id: str = "map",
    ) -> Optional[str]:
        """
        Send navigateTo command with direct coordinates (x/y/theta), not only target_id.
        """
        if not self.mqtt_client or not self.mqtt_client.is_connected():
            print(" MQTT not connected")
            return None

        # SPECIFICATION.md strict schema fields (Canonical v2.0)
        # NOTE: We use target_id="__pose__" as a reserved ID for direct pose mode.
        command = {
            "schema_version": "2.0",
            "robot_id": self.robot_id,
            "command_id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "priority": priority,
            "target_id": "__pose__",
            "x": float(x),
            "y": float(y),
            "theta": float(theta),
            "frame_id": str(frame_id) if frame_id else "map",
        }

        payload = json.dumps(command)

        print(f"\n SENDING COMMAND [{self.commands_sent + 1}]:")
        print(f"   Topic: {self.command_topic}")
        print(f"   Command ID: {command['command_id']}")
        print(f"   Goal: x={command['x']:.3f}, y={command['y']:.3f}, theta={command['theta']:.3f} rad, frame_id={command['frame_id']}")
        print(f"   Priority: {priority}")
        print()

        result = self.mqtt_client.publish(self.command_topic, payload, qos=1)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.commands_sent += 1
            self.mark_command_sent(command["command_id"])
            print(f" Command sent successfully (mid={result.mid})")
            return command["command_id"]

        print(f" Failed to send command (rc={result.rc})")
        return None
    
    def send_cancel_command(self, reason: str = "user") -> bool:
        """
        Send cancel command
        
        Args:
            reason: Cancellation reason (user, system, emergency)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.mqtt_client or not self.mqtt_client.is_connected():
            print(" MQTT not connected")
            return False
        
        # SPECIFICATION.md strict schema fields (Canonical v2.0)
        command = {
            "schema_version": "2.0",
            "robot_id": self.robot_id,
            "command_id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "reason": reason,
        }
        
        payload = json.dumps(command)
        
        print(f"\n SENDING CANCEL COMMAND:")
        print(f"   Topic: {self.cancel_topic}")
        print(f"   Command ID: {command['command_id']}")
        print(f"   Reason: {reason}")
        print()
        
        result = self.mqtt_client.publish(self.cancel_topic, payload, qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f" Cancel command sent successfully (mid={result.mid})")
            return True
        else:
            print(f" Failed to send cancel command (rc={result.rc})")
            return False
    
    def print_statistics(self):
        """Print statistics"""
        print("\n" + "="*60)
        print(" STATISTICS:")
        print(f"   Commands sent: {self.commands_sent}")
        print(f"   Statuses received: {self.statuses_received}")
        if self.last_status:
            print(f"   Last status: {self.last_status.get('status', 'N/A')}")
        print("="*60 + "\n")
    
    def shutdown(self):
        """Disconnect from MQTT"""
        # Give time for final QoS1 statuses to arrive
        try:
            time.sleep(2.0)
        except Exception:
            pass
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
            except Exception:
                pass
            try:
                self.mqtt_client.disconnect()
            except Exception:
                pass
            print(" Disconnected from MQTT")


def interactive_mode(hub: FakeHub):
    """Interactive mode"""
    print("\n" + "="*60)
    print(" INTERACTIVE MODE")
    print("="*60)
    print("Commands:")
    print("  navigate <target_id> [priority]  - Send navigate command")
    print("  navigate_pose <x> <y> [theta] [priority] [frame_id]  - Send navigate command by direct pose (meters, radians)")
    print("  cancel [reason]                  - Send cancel command")
    print("  status                           - Show last status")
    print("  stats                            - Show statistics")
    print("  quit                              - Exit")
    print("="*60 + "\n")
    
    while True:
        try:
            cmd = input("fake_hub> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == "quit" or cmd[0] == "exit":
                break
            
            elif cmd[0] == "navigate":
                if len(cmd) < 2:
                    print(" Usage: navigate <target_id> [priority]")
                    continue
                target_id = cmd[1]
                priority = cmd[2] if len(cmd) > 2 else "normal"
                hub.send_navigate_command(target_id, priority)

            elif cmd[0] == "navigate_pose":
                if len(cmd) < 3:
                    print(" Usage: navigate_pose <x> <y> [theta] [priority] [frame_id]")
                    continue
                x = float(cmd[1])
                y = float(cmd[2])
                theta = float(cmd[3]) if len(cmd) > 3 else 0.0
                priority = cmd[4] if len(cmd) > 4 else "normal"
                frame_id = cmd[5] if len(cmd) > 5 else "map"
                hub.send_navigate_pose_command(x=x, y=y, theta=theta, priority=priority, frame_id=frame_id)
            
            elif cmd[0] == "cancel":
                reason = cmd[1] if len(cmd) > 1 else "user"
                hub.send_cancel_command(reason)
            
            elif cmd[0] == "status":
                if hub.last_status:
                    print("\n Last Status:")
                    print(json.dumps(hub.last_status, indent=2))
                else:
                    print(" No status received yet")
            
            elif cmd[0] == "stats":
                hub.print_statistics()
            
            else:
                print(f" Unknown command: {cmd[0]}")
        
        except KeyboardInterrupt:
            break
        except EOFError:
            break


def main():
    parser = argparse.ArgumentParser(
        description="Fake AE.HUB - MQTT Command Sender and Status Monitor"
    )
    parser.add_argument(
        '--config-service-url',
        default='http://localhost:7900',
        help='Config Service URL (default: http://localhost:7900)'
    )
    parser.add_argument(
        '--api-key',
        required=True,
        help='API key for Config Service'
    )
    parser.add_argument(
        '--robot-id',
        default='fahrdummy-01',
        help='Robot ID (default: fahrdummy-01)'
    )
    parser.add_argument(
        '--target-id',
        help='Target position ID (for single command mode)'
    )
    parser.add_argument(
        '--x',
        type=float,
        help='Direct goal X in meters (for single command mode)'
    )
    parser.add_argument(
        '--y',
        type=float,
        help='Direct goal Y in meters (for single command mode)'
    )
    parser.add_argument(
        '--theta',
        type=float,
        default=0.0,
        help='Direct goal theta (yaw) in radians (default: 0.0)'
    )
    parser.add_argument(
        '--frame-id',
        default='map',
        help='Direct goal frame_id (default: map)'
    )
    parser.add_argument(
        '--priority',
        default='normal',
        choices=['normal', 'high', 'emergency'],
        help='Command priority (default: normal)'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='Interactive mode'
    )
    parser.add_argument(
        '--wait',
        type=float,
        default=5.0,
        help='Wait time after command in seconds (default: 5.0)'
    )
    
    args = parser.parse_args()
    
    # Create hub
    hub = FakeHub(
        config_service_url=args.config_service_url,
        api_key=args.api_key,
        robot_id=args.robot_id
    )
    
    # Fetch configuration
    if not hub.fetch_mqtt_config():
        print(" Failed to fetch MQTT config")
        sys.exit(1)
    
    # Connect to MQTT
    if not hub.connect_mqtt():
        print(" Failed to connect to MQTT")
        sys.exit(1)
    
    try:
        if args.interactive:
            # Interactive mode
            interactive_mode(hub)
        elif args.target_id or (args.x is not None and args.y is not None):
            # Single command
            if args.x is not None and args.y is not None:
                hub.send_navigate_pose_command(
                    x=float(args.x),
                    y=float(args.y),
                    theta=float(args.theta),
                    priority=args.priority,
                    frame_id=args.frame_id,
                )
            else:
                hub.send_navigate_command(args.target_id, args.priority)
            print(f" Waiting {args.wait}s for status updates...")
            time.sleep(args.wait)
            hub.print_statistics()
        else:
            print(" Either --target-id, (--x and --y), or --interactive must be specified")
            sys.exit(1)
    
    finally:
        hub.shutdown()


if __name__ == "__main__":
    main()

