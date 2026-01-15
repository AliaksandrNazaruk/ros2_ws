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


CONFIG_SERVICE_URL = "http://localhost:7900"
API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = os.getenv("ROBOT_ID", "fahrdummy-01-local")


def send_navigation_command(target_id: str):
    """Send navigation command and monitor result"""
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
        
        # Monitor for 30 seconds
        print("Monitoring robot response (30 seconds)...")
        print("="*80)
        
        start_time = time.time()
        timeout = 30.0
        last_status = None
        
        while (time.time() - start_time) < timeout:
            # Get current status
            status_obj = hub.get_status_by_command_id(command_id, timeout=1.0, wait_for_error=False)
            
            if status_obj:
                status_val = status_obj.get('status', 'unknown')
                error_code = status_obj.get('error_code')
                progress = status_obj.get('progress_percent', 0)
                target_id_status = status_obj.get('target_id')
                
                # Print if status changed
                if status_obj != last_status:
                    print(f"\n📊 Status Update:")
                    print(f"   Status: {status_val}")
                    print(f"   Target ID: {target_id_status}")
                    print(f"   Progress: {progress}%")
                    if error_code:
                        print(f"   Error: {error_code}")
                    last_status = status_obj.copy()
                    
                    # Check if navigation completed
                    if status_val == 'succeeded':
                        print(f"\n✅ Navigation completed successfully!")
                        return True
                    elif status_val == 'error' and error_code:
                        if error_code == 'NAV_GOAL_ABORTED':
                            print(f"\n⚠️  Navigation aborted (may be due to no map or unreachable goal)")
                        else:
                            print(f"\n❌ Navigation failed: {error_code}")
                        return False
            
            time.sleep(1.0)
            print(".", end="", flush=True)
        
        print(f"\n\n⏱️  Timeout reached")
        
        # Final status
        final_status = hub.get_status_by_command_id(command_id, timeout=2.0, wait_for_error=True)
        if final_status:
            status_val = final_status.get('status', 'unknown')
            error_code = final_status.get('error_code')
            print(f"\n📊 Final Status:")
            print(f"   Status: {status_val}")
            if error_code:
                print(f"   Error: {error_code}")
        
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
