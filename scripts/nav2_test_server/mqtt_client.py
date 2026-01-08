"""
MQTT Test Client

Client for sending navigation commands and receiving status updates via MQTT.
"""

import paho.mqtt.client as mqtt
import ssl
import json
import threading
import time
import uuid
from typing import Optional, Dict, Any, Callable
from datetime import datetime
from .config import Config


class MQTTTestClient:
    """MQTT client for testing navigation commands"""
    
    def __init__(self, config: Config):
        self.config = config
        self.client: Optional[mqtt.Client] = None
        self.is_connected = False
        self.last_status: Optional[Dict[str, Any]] = None
        self.last_status_time: Optional[datetime] = None
        self.lock = threading.Lock()
        
        # Callbacks
        self.on_status_callback: Optional[Callable] = None
    
    def _fetch_config_from_service(self) -> Optional[Dict[str, Any]]:
        """Fetch MQTT configuration from Config Service on port 7900"""
        if not self.config.config_service_url or not self.config.config_service_api_key:
            return None
        
        try:
            import requests
            url = f"{self.config.config_service_url}/config/broker?include_password=true"
            headers = {
                "X-API-Key": self.config.config_service_api_key,
                "Content-Type": "application/json"
            }
            response = requests.get(url, headers=headers, timeout=5)
            if response.status_code == 200:
                config = response.json()
                # Validate and normalize data
                validated = self._validate_config(config)
                if validated:
                    return validated
                else:
                    print("ERROR: Invalid configuration data from Config Service")
                    return None
        except Exception as e:
            print(f"Warning: Failed to fetch config from Config Service: {e}")
        return None
    
    def _validate_config(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
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
            print("ERROR: Broker field is missing or empty")
            return None
        
        # Validate broker length (prevent DoS)
        if len(broker) > 255:
            print(f"ERROR: Broker field too long: {len(broker)} characters (max 255)")
            return None
        
        # Normalize and validate port
        broker_port = config.get('broker_port')
        if broker_port is None:
            print("ERROR: Broker port is missing")
            return None
        
        # Convert to int if string
        try:
            port = int(broker_port)
        except (ValueError, TypeError):
            print(f"ERROR: Broker port must be an integer, got {type(broker_port).__name__}")
            return None
        
        # Validate port range
        if port <= 0 or port > 65535:
            print(f"ERROR: Broker port must be 1-65535, got {port}")
            return None
        
        # Normalize username (convert None to empty string)
        username = config.get('mqtt_user')
        if username is None:
            username = ""
        else:
            username = str(username).strip()
        
        # Validate username (required, non-empty)
        if not username:
            print("ERROR: MQTT user field is missing or empty")
            return None
        
        # Normalize password (convert None to empty string)
        password = config.get('mqtt_password')
        if password is None:
            password = ""
        else:
            password = str(password)
        
        # Validate password (required, non-empty)
        if not password:
            print("ERROR: MQTT password field is missing or empty")
            return None
        
        # Normalize boolean fields
        use_tls = bool(config.get('mqtt_use_tls', False))
        tls_insecure = bool(config.get('mqtt_tls_insecure', False))
        
        return {
            'broker': broker,
            'broker_port': port,
            'mqtt_user': username,
            'mqtt_password': password,
            'mqtt_use_tls': use_tls,
            'mqtt_tls_insecure': tls_insecure
        }
    
    def connect(self, broker_config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Connect to MQTT broker.
        
        Args:
            broker_config: Optional broker configuration dict.
                          If None, tries Config Service (port 7900), then Config object.
        
        Returns:
            True if connected successfully, False otherwise.
        """
        try:
            # Priority: provided config > Config Service > Config object
            if broker_config:
                broker = broker_config.get('broker', self.config.mqtt_broker)
                port = broker_config.get('broker_port', self.config.mqtt_port)
                username = broker_config.get('mqtt_user', self.config.mqtt_username)
                password = broker_config.get('mqtt_password', self.config.mqtt_password)
                use_tls = broker_config.get('mqtt_use_tls', self.config.mqtt_use_tls)
                tls_insecure = broker_config.get('mqtt_tls_insecure', self.config.mqtt_tls_insecure)
            else:
                # Try Config Service first (port 7900)
                service_config = self._fetch_config_from_service()
                if service_config:
                    print("Using MQTT configuration from Config Service (port 7900)")
                    broker = service_config.get('broker')
                    port = service_config.get('broker_port')
                    username = service_config.get('mqtt_user')
                    password = service_config.get('mqtt_password')
                    use_tls = service_config.get('mqtt_use_tls', False)
                    tls_insecure = service_config.get('mqtt_tls_insecure', False)
                else:
                    # Fall back to Config object
                    broker = self.config.mqtt_broker
                    port = self.config.mqtt_port
                    username = self.config.mqtt_username
                    password = self.config.mqtt_password
                    use_tls = self.config.mqtt_use_tls
                    tls_insecure = self.config.mqtt_tls_insecure
            
            # Validate broker and port
            if not broker or (isinstance(broker, str) and broker.strip() == ""):
                print("ERROR: MQTT broker not configured")
                print("       Set CONFIG_SERVICE_URL and CONFIG_SERVICE_API_KEY to fetch from Config Service")
                return False
            
            # Validate port
            if port is None:
                print("ERROR: MQTT port not configured")
                return False
            
            # Convert port to int if needed
            try:
                port = int(port)
            except (ValueError, TypeError):
                print(f"ERROR: Invalid port type: {type(port).__name__}")
                return False
            
            # Validate port range
            if port <= 0 or port > 65535:
                print(f"ERROR: Invalid port: {port} (must be 1-65535)")
                return False
            
            # Create client
            self.client = mqtt.Client()
            
            # Set credentials
            if username and password:
                self.client.username_pw_set(username, password)
            
            # Configure TLS
            if use_tls:
                if tls_insecure:
                    self.client.tls_set(cert_reqs=ssl.CERT_NONE)
                    self.client.tls_insecure_set(True)
                else:
                    # Secure TLS - would need CA certificate path
                    self.client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
            
            # Set callbacks
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message
            
            # Connect
            print(f"Connecting to MQTT broker: {broker}:{port}")
            self.client.connect(broker, port, 60)
            self.client.loop_start()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.is_connected:
                # Subscribe to status topic
                status_topic = f"aroc/robot/{self.config.robot_id}/status/navigation"
                self.client.subscribe(status_topic, qos=1)
                print(f"Subscribed to: {status_topic}")
                return True
            else:
                print(f"ERROR: Failed to connect to MQTT broker {broker}:{port}")
                print(f"  - Username: {'set' if username else 'not set'}")
                print(f"  - TLS: {'enabled' if use_tls else 'disabled'}")
                return False
                
        except Exception as e:
            print(f"ERROR: Exception connecting to MQTT: {e}")
            return False
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT on_connect callback"""
        if rc == 0:
            with self.lock:
                self.is_connected = True
            print("MQTT: Connected successfully")
        else:
            with self.lock:
                self.is_connected = False
            error_messages = {
                1: "incorrect protocol version",
                2: "invalid client identifier",
                3: "server unavailable",
                4: "bad username or password",
                5: "not authorised"
            }
            error_msg = error_messages.get(rc, f"unknown error code {rc}")
            print(f"MQTT: Connection failed - {error_msg} (code {rc})")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT on_disconnect callback"""
        with self.lock:
            self.is_connected = False
        print("Disconnected from MQTT broker")
    
    def _on_message(self, client, userdata, msg):
        """MQTT on_message callback"""
        try:
            payload = json.loads(msg.payload.decode())
            with self.lock:
                self.last_status = payload
                self.last_status_time = datetime.now()
            
            # Call status callback if set
            if self.on_status_callback:
                self.on_status_callback(payload)
                
        except Exception as e:
            print(f"ERROR: Failed to parse MQTT message: {e}")
    
    def send_navigate_command(self, target_id: str, priority: str = "normal") -> Optional[str]:
        """
        Send navigate command via MQTT.
        
        Args:
            target_id: Target position ID (position_A, position_B, etc.)
            priority: Command priority (normal, high, emergency)
        
        Returns:
            Command ID if sent successfully, None otherwise.
        """
        if not self.is_connected or not self.client:
            print("ERROR: MQTT client not connected")
            return None
        
        try:
            command_id = str(uuid.uuid4())
            command = {
                "command_id": command_id,
                "timestamp": datetime.now().isoformat() + "Z",
                "target_id": target_id,
                "priority": priority
            }
            
            topic = f"aroc/robot/{self.config.robot_id}/commands/navigateTo"
            result = self.client.publish(topic, json.dumps(command), qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"Sent navigate command: {command_id} -> {target_id}")
                return command_id
            else:
                print(f"ERROR: Failed to publish command: {result.rc}")
                return None
                
        except Exception as e:
            print(f"ERROR: Exception sending navigate command: {e}")
            return None
    
    def send_cancel_command(self, reason: Optional[str] = None) -> Optional[str]:
        """
        Send cancel command via MQTT.
        
        Args:
            reason: Optional cancel reason
        
        Returns:
            Command ID if sent successfully, None otherwise.
        """
        if not self.is_connected or not self.client:
            print("ERROR: MQTT client not connected")
            return None
        
        try:
            command_id = str(uuid.uuid4())
            command = {
                "command_id": command_id,
                "timestamp": datetime.now().isoformat() + "Z"
            }
            if reason:
                command["reason"] = reason
            
            topic = f"aroc/robot/{self.config.robot_id}/commands/cancel"
            result = self.client.publish(topic, json.dumps(command), qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"Sent cancel command: {command_id}")
                return command_id
            else:
                print(f"ERROR: Failed to publish cancel command: {result.rc}")
                return None
                
        except Exception as e:
            print(f"ERROR: Exception sending cancel command: {e}")
            return None
    
    def get_last_status(self) -> Optional[Dict[str, Any]]:
        """Get last received navigation status"""
        with self.lock:
            return self.last_status.copy() if self.last_status else None
    
    def get_status_info(self) -> Dict[str, Any]:
        """Get MQTT connection status information"""
        with self.lock:
            return {
                'connected': self.is_connected,
                'broker': self.config.mqtt_broker,
                'port': self.config.mqtt_port,
                'last_status': self.last_status.copy() if self.last_status else None,
                'last_status_time': self.last_status_time.isoformat() if self.last_status_time else None
            }
    
    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            with self.lock:
                self.is_connected = False

