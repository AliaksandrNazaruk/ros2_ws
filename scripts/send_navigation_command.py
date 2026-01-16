#!/usr/bin/env python3
"""
Send navigation command to robot

Simple script to send a navigation command and monitor the result.
"""

import sys
import os
import time
from pathlib import Path

# Add scripts directory to path
SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))

from fake_hub import FakeHub


CONFIG_SERVICE_URL = os.getenv("CONFIG_SERVICE_URL", "http://localhost:7900")
API_KEY = os.getenv("CONFIG_SERVICE_API_KEY", "")
ROBOT_ID = os.getenv("ROBOT_ID", "fahrdummy-01-local")


def send_navigation_command(target_id: str):
    """Send navigation command and monitor result"""
    if not API_KEY:
        print("❌ ERROR: CONFIG_SERVICE_API_KEY is not set")
        return False

    print("="*80)
    print(f"Sending navigation command to: {target_id}")
    print("="*80)
    print()
    
    # Create fake_hub
    hub = FakeHub(
        config_service_url=CONFIG_SERVICE_URL,
        api_key=API_KEY,
        robot_id=ROBOT_ID
    )
    
    # Initialize
    print("Connecting to MQTT...")
    if not hub.fetch_mqtt_config():
        print("❌ ERROR: Failed to fetch MQTT configuration")
        return False
    
    if not hub.connect_mqtt():
        print("❌ ERROR: Failed to connect to MQTT broker")
        return False
    
    print("✅ Connected to MQTT\n")
    
    try:
        # Send command
        print(f"Sending command: target_id={target_id}")
        command_id = hub.send_navigate_command(target_id, priority="normal")
        
        if not command_id:
            print("❌ Failed to send command")
            return False
        
        print(f"✅ Command sent: command_id={command_id}\n")
        
        # Wait for authoritative result event
        print("Waiting for commands/events terminal result (30 seconds)...")
        print("="*80)
        result_event = hub.wait_for_result_event(command_id, timeout=30.0)

        if not result_event or result_event.get("event_type") != "result":
            last_type = result_event.get("event_type") if result_event else "none"
            print(f"\n⏱️  Timeout reached (last event: {last_type})")
            return False

        result_status = result_event.get("result_status")
        error_code = result_event.get("error_code")
        error_message = result_event.get("error_message")

        print("\n📣 Result event received:")
        print(f"   Result: {result_status}")
        if error_code:
            print(f"   Error: {error_code}")
        if error_message:
            print(f"   Error message: {error_message}")

        if result_status == "succeeded":
            print("\n✅ Navigation completed successfully!")
            return True

        print("\n❌ Navigation failed")
        return False
        
    finally:
        hub.shutdown()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: send_navigation_command.py <target_id>")
        print("\nAvailable positions:")
        print("  - position_A")
        print("  - position_B")
        print("  - position_C")
        print("  - position_D")
        print("  - position_E")
        sys.exit(1)
    
    target_id = sys.argv[1]
    success = send_navigation_command(target_id)
    sys.exit(0 if success else 1)
