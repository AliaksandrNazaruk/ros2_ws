#!/usr/bin/env python3
"""
Test MQTT Navigation Commands
Sends real navigation commands via MQTT to test the full integration
Gets MQTT configuration from Config Service
"""

import paho.mqtt.client as mqtt
import json
import uuid
import time
import sys
import requests
import ssl
from datetime import datetime, timezone

# Config Service settings
CONFIG_SERVICE_URL = "http://localhost:7900"
CONFIG_SERVICE_API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = "robot_001"

class MQTTNavigationTester:
    def __init__(self):
        self.client = None
        self.connected = False
        self.last_status = None
        self.status_received = False
        self.mqtt_config = None
    
    def fetch_mqtt_config(self):
        """Fetch MQTT configuration from Config Service"""
        print(f" Fetching MQTT configuration from Config Service...")
        url = f"{CONFIG_SERVICE_URL}/config/broker?include_password=true"
        headers = {
            "X-API-Key": CONFIG_SERVICE_API_KEY,
            "Content-Type": "application/json"
        }
        
        try:
            response = requests.get(url, headers=headers, timeout=5)
            if response.status_code == 200:
                config = response.json()
                
                # Validate and normalize data
                validated = self._validate_config(config)
                if not validated:
                    return False
                
                self.mqtt_config = validated
                print(f" MQTT configuration loaded:")
                print(f"   Broker: {self.mqtt_config['broker']}:{self.mqtt_config['port']}")
                print(f"   Username: {self.mqtt_config['username']}")
                print(f"   TLS: {self.mqtt_config['use_tls']}")
                return True
            else:
                print(f" Failed to fetch config: HTTP {response.status_code}")
                return False
        except json.JSONDecodeError as e:
            print(f" Invalid JSON response from Config Service: {e}")
            return False
        except Exception as e:
            print(f" Error fetching config: {e}")
            return False
    
    def _validate_config(self, config: dict) -> dict:
        """
        Validate and normalize configuration data from Config Service.
        
        Args:
            config: Raw configuration data from Config Service
            
        Returns:
            Validated and normalized config dict, or None if validation fails
        """
        # Normalize broker (convert None to empty string, strip whitespace)
        broker = config.get('broker')
        if broker is None:
            broker = ""
        else:
            broker = str(broker).strip()
        
        # Validate broker (required, non-empty)
        if not broker:
            print(" Invalid config: broker field is missing or empty")
            return None
        
        # Validate broker length (prevent DoS)
        if len(broker) > 255:
            print(f" Invalid config: broker field too long: {len(broker)} characters (max 255)")
            return None
        
        # Normalize and validate port
        broker_port = config.get('broker_port')
        if broker_port is None:
            print(" Invalid config: broker_port field is missing")
            return None
        
        # Convert to int if string
        try:
            port = int(broker_port)
        except (ValueError, TypeError):
            print(f" Invalid config: broker_port must be an integer, got {type(broker_port).__name__}")
            return None
        
        # Validate port range
        if port <= 0 or port > 65535:
            print(f" Invalid config: broker_port must be 1-65535, got {port}")
            return None
        
        # Normalize username (convert None to empty string)
        username = config.get('mqtt_user')
        if username is None:
            username = ""
        else:
            username = str(username).strip()
        
        # Validate username (required, non-empty)
        if not username:
            print(" Invalid config: mqtt_user field is missing or empty")
            return None
        
        # Normalize password (convert None to empty string)
        password = config.get('mqtt_password')
        if password is None:
            password = ""
        else:
            password = str(password)
        
        # Validate password (required, non-empty)
        if not password:
            print(" Invalid config: mqtt_password field is missing or empty")
            return None
        
        # Normalize boolean fields
        use_tls = bool(config.get('mqtt_use_tls', False))
        tls_insecure = bool(config.get('mqtt_tls_insecure', False))
        
        return {
            'broker': broker,
            'port': port,
            'username': username,
            'password': password,
            'use_tls': use_tls,
            'tls_insecure': tls_insecure
        }
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f" Connected to MQTT broker")
            self.connected = True
            # Subscribe to status topic
            status_topic = f"aroc/robot/{ROBOT_ID}/status/navigation"
            client.subscribe(status_topic, qos=1)
            print(f" Subscribed to: {status_topic}")
        else:
            print(f" Connection failed with code {rc}")
            self.connected = False
    
    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            self.last_status = payload
            self.status_received = True
            print(f"\n📨 Status received from {msg.topic}:")
            print(json.dumps(payload, indent=2))
        except Exception as e:
            print(f" Error parsing message: {e}")
    
    def connect(self):
        """Connect to MQTT broker"""
        if not self.mqtt_config:
            print(" MQTT configuration not loaded")
            return False
        
        self.client = mqtt.Client(client_id=f"nav_tester_{uuid.uuid4()}")
        self.client.username_pw_set(
            self.mqtt_config['username'],
            self.mqtt_config['password']
        )
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Configure TLS if needed
        if self.mqtt_config['use_tls']:
            context = ssl.create_default_context()
            if self.mqtt_config['tls_insecure']:
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
            self.client.tls_set_context(context)
        
        try:
            broker = self.mqtt_config['broker']
            port = self.mqtt_config['port']
            print(f"🔌 Connecting to MQTT broker: {broker}:{port}")
            self.client.connect(broker, port, 60)
            self.client.loop_start()
            
            # Wait for connection
            timeout = 10
            start = time.time()
            while not self.connected and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not self.connected:
                print(" Connection timeout")
                return False
            return True
        except Exception as e:
            print(f" Connection error: {e}")
            return False
    
    def send_navigation_command(self, target_id, priority="normal"):
        """Send navigation command"""
        if not self.connected:
            print(" Not connected to MQTT")
            return False
        
        command = {
            "command_id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "target_id": target_id,
            "priority": priority
        }
        
        topic = f"aroc/robot/{ROBOT_ID}/commands/navigateTo"
        
        try:
            result = self.client.publish(topic, json.dumps(command), qos=1)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f" Navigation command sent:")
                print(f"   Topic: {topic}")
                print(f"   Command ID: {command['command_id']}")
                print(f"   Target ID: {target_id}")
                print(f"   Priority: {priority}")
                return True
            else:
                print(f" Failed to publish: {result.rc}")
                return False
        except Exception as e:
            print(f" Error sending command: {e}")
            return False
    
    def wait_for_status(self, timeout=10):
        """Wait for status update"""
        print(f"\n Waiting for status update (timeout: {timeout}s)...")
        start = time.time()
        while not self.status_received and (time.time() - start) < timeout:
            time.sleep(0.5)
        
        if self.status_received:
            return self.last_status
        else:
            print("⏱  Timeout waiting for status")
            return None
    
    def disconnect(self):
        """Disconnect from MQTT"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            print("🔌 Disconnected from MQTT broker")

def main():
    print("=" * 60)
    print("  MQTT Navigation Command Tester")
    print("=" * 60)
    print(f"Config Service: {CONFIG_SERVICE_URL}")
    print(f"Robot ID: {ROBOT_ID}")
    print()
    
    tester = MQTTNavigationTester()
    
    # Fetch MQTT configuration from Config Service
    if not tester.fetch_mqtt_config():
        print(" Failed to fetch MQTT configuration from Config Service")
        return 1
    
    if not tester.connect():
        print(" Failed to connect to MQTT broker")
        return 1
    
    time.sleep(1)  # Wait for subscription
    
    # Test navigation command
    target_id = "position_default"  # Use first available position
    print(f"\n Sending navigation command to: {target_id}")
    
    if tester.send_navigation_command(target_id, "normal"):
        # Wait for status
        status = tester.wait_for_status(timeout=10)
        if status:
            print("\n Navigation command processed successfully!")
            print(f"   Status: {status.get('status', 'unknown')}")
            print(f"   Target ID: {status.get('target_id', 'unknown')}")
        else:
            print("\n  No status received (node may not be running)")
    else:
        print("\n Failed to send navigation command")
    
    tester.disconnect()
    return 0

if __name__ == '__main__':
    sys.exit(main())

