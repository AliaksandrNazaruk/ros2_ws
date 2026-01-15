#!/usr/bin/env python3
"""
Verify that map origin correctly accounts for offsetX and offsetY from Symovo API
"""

import sys
import os
import yaml
import requests
import json

def verify_map_origin(map_yaml_path, symovo_endpoint='https://192.168.1.100'):
    """Verify map origin matches offsetX and offsetY from Symovo API"""
    
    print("="*70)
    print("🔍 Verifying Map Origin")
    print("="*70)
    print()
    
    # Get map info from Symovo API
    print("1. Getting map info from Symovo API...")
    try:
        response = requests.get(f"{symovo_endpoint}/v0/map", verify=False, timeout=10)
        response.raise_for_status()
        maps = response.json()
        
        if not isinstance(maps, list) or len(maps) == 0:
            print("   ❌ No maps found in API response")
            return False
        
        map_info = maps[0]
        api_offset_x = map_info.get('offsetX', 0.0)
        api_offset_y = map_info.get('offsetY', 0.0)
        
        print(f"   ✅ API offsetX: {api_offset_x}")
        print(f"   ✅ API offsetY: {api_offset_y}")
        print()
    except Exception as e:
        print(f"   ❌ Failed to get map info from API: {e}")
        return False
    
    # Check map YAML file
    print("2. Checking map YAML file...")
    if not os.path.exists(map_yaml_path):
        print(f"   ❌ Map YAML file not found: {map_yaml_path}")
        return False
    
    try:
        with open(map_yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        yaml_origin = map_data.get('origin', [0.0, 0.0, 0.0])
        yaml_offset_x = yaml_origin[0] if isinstance(yaml_origin, list) and len(yaml_origin) > 0 else 0.0
        yaml_offset_y = yaml_origin[1] if isinstance(yaml_origin, list) and len(yaml_origin) > 1 else 0.0
        
        print(f"   ✅ YAML origin X: {yaml_offset_x}")
        print(f"   ✅ YAML origin Y: {yaml_offset_y}")
        print()
    except Exception as e:
        print(f"   ❌ Failed to read map YAML: {e}")
        return False
    
    # Compare
    print("3. Comparing offsets...")
    x_match = abs(api_offset_x - yaml_offset_x) < 0.001
    y_match = abs(api_offset_y - yaml_offset_y) < 0.001
    
    if x_match and y_match:
        print("   ✅ Map origin correctly matches Symovo API offsets!")
        print()
        print("="*70)
        print("✅ VERIFICATION PASSED")
        print("="*70)
        return True
    else:
        print("   ❌ Map origin does NOT match Symovo API offsets!")
        print(f"      X difference: {abs(api_offset_x - yaml_offset_x):.6f}")
        print(f"      Y difference: {abs(api_offset_y - yaml_offset_y):.6f}")
        print()
        print("="*70)
        print("❌ VERIFICATION FAILED")
        print("="*70)
        print()
        print("💡 Solution: Reload map using load_symovo_map.py")
        print(f"   python3 scripts/load_symovo_map.py {symovo_endpoint} <amr_id> <output_dir>")
        return False


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: verify_map_origin.py <map_yaml_path> [symovo_endpoint]")
        print("Example: verify_map_origin.py maps/symovo_map/map.yaml https://192.168.1.100")
        sys.exit(1)
    
    map_yaml_path = sys.argv[1]
    symovo_endpoint = sys.argv[2] if len(sys.argv) > 2 else 'https://192.168.1.100'
    
    success = verify_map_origin(map_yaml_path, symovo_endpoint)
    sys.exit(0 if success else 1)
