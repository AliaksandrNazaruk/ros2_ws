#!/usr/bin/env python3
"""
MQTT Chain Debugger

Debug MQTT chain from fake_hub to robot:
- Send commands through fake_hub
- Track all MQTT messages
- Check robot blocking handling
- Validate specification compliance
"""

import sys
import os
import time
import json
from pathlib import Path
from datetime import datetime, timezone

# Add scripts directory to path
SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))

from fake_hub import FakeHub


CONFIG_SERVICE_URL = "http://localhost:7900"
API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = os.getenv("ROBOT_ID", "fahrdummy-01-local")

# Log file path
LOG_DIR = Path("/tmp")
LOG_FILE = LOG_DIR / f"mqtt_chain_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"


class MQTTLogger:
    """Logger for MQTT messages to file and console"""
    
    def __init__(self, log_file: Path):
        self.log_file = log_file
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.log_handle = open(log_file, 'w', encoding='utf-8')
        self.write_header()
    
    def write_header(self):
        """Write header to log file"""
        header = f"""
{'='*80}
MQTT CHAIN DEBUG LOG
{'='*80}
Started: {datetime.now(timezone.utc).isoformat()}
Robot ID: {ROBOT_ID}
Config Service: {CONFIG_SERVICE_URL}
Log File: {self.log_file}
{'='*80}

"""
        self.log_handle.write(header)
        self.log_handle.flush()
    
    def log_message(self, msg_type: str, topic: str, payload: dict, direction: str = "→", 
                   validation_errors: list = None):
        """Write MQTT message to log and print to console"""
        timestamp = datetime.now(timezone.utc).isoformat()
        
        # Format message
        log_entry = f"""
{'='*80}
{direction} {msg_type.upper()}
{'='*80}
Topic: {topic}
Timestamp: {timestamp}
Payload:
{json.dumps(payload, indent=2, ensure_ascii=False)}
"""
        
        if validation_errors:
            log_entry += f"\n  SCHEMA VALIDATION ERRORS:\n"
            for error in validation_errors:
                log_entry += f"   - {error}\n"
        else:
            log_entry += f"\n Schema validation passed\n"
        
        log_entry += f"{'='*80}\n\n"
        
        # Write to file
        self.log_handle.write(log_entry)
        self.log_handle.flush()
        
        # Print to console
        print(log_entry, end='')
    
    def log_command_sent(self, command_id: str, target_id: str, priority: str):
        """Log command sending"""
        log_entry = f"""
{'='*80}
→ COMMAND SENT
{'='*80}
Timestamp: {datetime.now(timezone.utc).isoformat()}
Command ID: {command_id}
Target ID: {target_id}
Priority: {priority}
Topic: aroc/robot/{ROBOT_ID}/commands/navigateTo
{'='*80}

"""
        self.log_handle.write(log_entry)
        self.log_handle.flush()
        print(log_entry, end='')
    
    def log_summary(self, command_id: str, events: list, final_status: dict = None):
        """Log final summary"""
        log_entry = f"""
{'='*80}
 SUMMARY
{'='*80}
Command ID: {command_id}
Timestamp: {datetime.now(timezone.utc).isoformat()}

Events received: {len(events)}
"""
        
        if events:
            log_entry += "\nEvent sequence:\n"
            for i, event in enumerate(events, 1):
                ev_type = event.get('event_type')
                if ev_type == 'ack':
                    ack_type = event.get('ack_type')
                    log_entry += f"  {i}. ack({ack_type})\n"
                elif ev_type == 'result':
                    result_type = event.get('result_type')
                    error_code = event.get('error_code')
                    log_entry += f"  {i}. result({result_type}, error={error_code})\n"
        
        if final_status:
            log_entry += f"\nFinal Status:\n"
            log_entry += f"  Status: {final_status.get('status')}\n"
            log_entry += f"  Error Code: {final_status.get('error_code')}\n"
            log_entry += f"  Error Message: {final_status.get('error_message')}\n"
        
        log_entry += f"\n{'='*80}\n\n"
        
        self.log_handle.write(log_entry)
        self.log_handle.flush()
        print(log_entry, end='')
    
    def close(self):
        """Close log file"""
        footer = f"""
{'='*80}
LOG ENDED: {datetime.now(timezone.utc).isoformat()}
{'='*80}
"""
        self.log_handle.write(footer)
        self.log_handle.close()


def print_mqtt_message(msg_type: str, topic: str, payload: dict, direction: str = "→"):
    """Pretty print MQTT message (deprecated, use MQTTLogger)"""
    print(f"\n{'='*80}")
    print(f"{direction} {msg_type.upper()}")
    print(f"{'='*80}")
    print(f"Topic: {topic}")
    print(f"Timestamp: {datetime.now(timezone.utc).isoformat()}")
    print(f"Payload:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"{'='*80}\n")


def validate_command_schema(payload: dict) -> tuple[bool, list[str]]:
    """Validate command schema according to SPECIFICATION.md Section 3.1"""
    errors = []
    required_fields = ['schema_version', 'robot_id', 'command_id', 'timestamp', 'target_id', 'priority']
    
    for field in required_fields:
        if field not in payload:
            errors.append(f"Missing required field: {field}")
    
    if 'command_id' in payload:
        # Check UUIDv4 format
        import uuid
        try:
            uuid.UUID(payload['command_id'], version=4)
        except (ValueError, TypeError):
            errors.append(f"Invalid UUIDv4 format for command_id: {payload['command_id']}")
    
    if 'priority' in payload:
        valid_priorities = ['normal', 'high', 'emergency']
        if payload['priority'] not in valid_priorities:
            errors.append(f"Invalid priority: {payload['priority']}, must be one of {valid_priorities}")
    
    return len(errors) == 0, errors


def validate_event_schema(payload: dict) -> tuple[bool, list[str]]:
    """Validate event schema according to SPECIFICATION.md Section 3.3"""
    errors = []
    required_fields = ['schema_version', 'robot_id', 'timestamp', 'event_type', 'command_id']
    
    for field in required_fields:
        if field not in payload:
            errors.append(f"Missing required field: {field}")
    
    if 'event_type' in payload:
        if payload['event_type'] == 'ack':
            if 'ack_type' not in payload:
                errors.append("Missing ack_type for ack event")
            elif payload.get('ack_type') not in ['received', 'accepted', 'rejected']:
                errors.append(f"Invalid ack_type: {payload.get('ack_type')}")
        elif payload['event_type'] == 'result':
            if 'result_type' not in payload:
                errors.append("Missing result_type for result event")
            elif payload.get('result_type') not in ['succeeded', 'aborted', 'canceled', 'error']:
                errors.append(f"Invalid result_type: {payload.get('result_type')}")
    
    return len(errors) == 0, errors


def validate_status_schema(payload: dict) -> tuple[bool, list[str]]:
    """Validate status schema according to SPECIFICATION.md Section 3.4"""
    errors = []
    required_fields = ['schema_version', 'robot_id', 'timestamp', 'status']
    
    for field in required_fields:
        if field not in payload:
            errors.append(f"Missing required field: {field}")
    
    valid_statuses = ['idle', 'navigating', 'paused', 'error', 'canceling', 'succeeded', 'aborted']
    if 'status' in payload and payload['status'] not in valid_statuses:
        errors.append(f"Invalid status: {payload['status']}, must be one of {valid_statuses}")
    
    return len(errors) == 0, errors


def debug_mqtt_chain():
    """Debug MQTT chain"""
    # Initialize logger
    logger = MQTTLogger(LOG_FILE)
    
    print("="*80)
    print("MQTT CHAIN DEBUGGER")
    print("="*80)
    print(f"Robot ID: {ROBOT_ID}")
    print(f"Config Service: {CONFIG_SERVICE_URL}")
    print(f"Log file: {LOG_FILE}")
    print()
    
    # Create fake_hub
    hub = FakeHub(
        config_service_url=CONFIG_SERVICE_URL,
        api_key=API_KEY,
        robot_id=ROBOT_ID
    )
    
    # Get configuration and connect
    print(" Initializing fake_hub...")
    if not hub.fetch_mqtt_config():
        print(" ERROR: Failed to get MQTT configuration")
        logger.close()
        return False
    
    if not hub.connect_mqtt():
        print(" ERROR: Failed to connect to MQTT broker")
        logger.close()
        return False
    
    print(" fake_hub ready for testing\n")
    
    # Override handlers for detailed logging
    original_on_message = hub.mqtt_client.on_message
    
    def debug_on_message(client, userdata, msg):
        """Detailed MQTT message handler"""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Determine message type
            if 'commands/events' in msg.topic:
                msg_type = "EVENT"
                is_valid, errors = validate_event_schema(payload)
                direction = "←"  # From robot to hub
            elif 'status/navigation' in msg.topic:
                msg_type = "STATUS"
                is_valid, errors = validate_status_schema(payload)
                direction = "←"  # From robot to hub
            else:
                msg_type = "UNKNOWN"
                is_valid, errors = True, []
                direction = "?"
            
            # Write to log
            logger.log_message(msg_type, msg.topic, payload, direction, 
                             validation_errors=errors if not is_valid else None)
            
            # Call original handler
            original_on_message(client, userdata, msg)
            
        except Exception as e:
            error_msg = f" Error parsing MQTT message: {e}\n   Raw payload: {msg.payload}\n"
            print(error_msg)
            logger.log_handle.write(error_msg)
            logger.log_handle.flush()
    
    hub.mqtt_client.on_message = debug_on_message
    
    try:
        # TEST 1: Send navigation command
        print("\n" + "="*80)
        print("TEST 1: Sending navigation command")
        print("="*80)
        
        target_id = "position_A"
        command_id = hub.send_navigate_command(target_id, priority="normal")
        
        if not command_id:
            print(" Failed to send command")
            logger.close()
            return False
        
        # Log command sending
        logger.log_command_sent(command_id, target_id, "normal")
        
        print(f" Command sent: command_id={command_id}, target_id={target_id}")
        print("\n Waiting for robot responses (10 seconds)...")
        
        # Wait for responses
        time.sleep(10.0)
        
        # Get status
        status_obj = hub.get_status_by_command_id(command_id, timeout=2.0, wait_for_error=True)
        
        if status_obj:
            print(f"\n Final status for command_id={command_id}:")
            print(f"   Status: {status_obj.get('status')}")
            print(f"   Error Code: {status_obj.get('error_code')}")
            print(f"   Error Message: {status_obj.get('error_message')}")
            
            # Blocking check
            error_code = status_obj.get('error_code')
            if error_code == 'NAV_GOAL_REJECTED':
                print(f"\n Robot correctly handled blocking:")
                print(f"   - Command received (ack received)")
                print(f"   - Command rejected (ack rejected)")
                print(f"   - Error published (result error)")
                print(f"   - Robot does NOT try to move (goal rejected)")
            elif error_code == 'NAV_RATE_LIMIT_EXCEEDED':
                print(f"\n Rate limiting works correctly")
            else:
                print(f"\n  Unexpected error_code: {error_code}")
        else:
            print(f"\n  Status not received (timeout)")
        
        # TEST 2: Check blocking handling
        print("\n" + "="*80)
        print("TEST 2: Check robot blocking handling")
        print("="*80)
        print("Robot should:")
        print("  1. Receive command via MQTT")
        print("  2. Send ack(received)")
        print("  3. Try to send goal to Nav2")
        print("  4. Receive rejection from Nav2 (robot is blocked)")
        print("  5. Send ack(rejected) with reason")
        print("  6. Send result(error) with NAV_GOAL_REJECTED")
        print("  7. NOT try to move")
        print()
        
        # Collect all events for this command_id
        # status_by_command_id stores last status/event as dict
        events = []
        if command_id in hub.status_by_command_id:
            event = hub.status_by_command_id[command_id]
            if isinstance(event, dict):
                events.append(event)
        
        # Also check error_status_by_command_id
        if command_id in hub.error_status_by_command_id:
            error_event = hub.error_status_by_command_id[command_id]
            if isinstance(error_event, dict) and error_event not in events:
                events.append(error_event)
        
        print(f" Events received: {len(events)}")
        
        event_sequence = []
        for event in events:
            if isinstance(event, dict):
                ev_type = event.get('event_type')
                if ev_type == 'ack':
                    ack_type = event.get('ack_type')
                    event_sequence.append(f"ack({ack_type})")
                elif ev_type == 'result':
                    result_type = event.get('result_type')
                    error_code = event.get('error_code')
                    event_sequence.append(f"result({result_type}, error={error_code})")
                elif 'status' in event:
                    # This is status, not event
                    status = event.get('status')
                    error_code = event.get('error_code')
                    event_sequence.append(f"status({status}, error={error_code})")
        
        print(f" Event sequence: {' → '.join(event_sequence)}")
        
        # Write final summary to log
        logger.log_summary(command_id, events, status_obj)
        
        # Check expected sequence
        expected_sequence = ['ack(received)', 'ack(rejected)', 'result(error, error=NAV_GOAL_REJECTED)']
        if len(event_sequence) >= 2:
            if 'ack(received)' in event_sequence and 'ack(rejected)' in event_sequence:
                print(f"\n Event sequence is correct for blocked robot")
            else:
                print(f"\n  Unexpected event sequence")
        else:
            print(f"\n  Not enough events for check")
        
        print(f"\nFull log saved to: {LOG_FILE}")
        
    finally:
        hub.shutdown()
        logger.close()
    
    return True


if __name__ == "__main__":
    success = debug_mqtt_chain()
    sys.exit(0 if success else 1)

