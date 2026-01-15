#!/usr/bin/env python3
"""
Load map from Symovo API and convert to Nav2 format.

Usage:
    python3 scripts/load_symovo_map.py <endpoint> <amr_id> <output_dir>
    
Example:
    python3 scripts/load_symovo_map.py https://192.168.1.100 15 /tmp/map
"""

import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'aehub_navigation', 'src'))

from aehub_navigation.symovo_map_loader import SymovoMapLoader

def main():
    if len(sys.argv) < 4:
        print("Usage: load_symovo_map.py <endpoint> <amr_id> <output_dir>")
        print("Example: load_symovo_map.py https://192.168.1.100 15 /tmp/map")
        sys.exit(1)
    
    endpoint = sys.argv[1]
    amr_id = int(sys.argv[2])
    output_dir = sys.argv[3]
    
    print(f"Loading map from Symovo API...")
    print(f"  Endpoint: {endpoint}")
    print(f"  AMR ID: {amr_id}")
    print(f"  Output: {output_dir}")
    print()
    
    loader = SymovoMapLoader(endpoint, amr_id, tls_verify=False)
    yaml_path = loader.load_map(output_dir)
    
    if yaml_path:
        print(f"\n✅ Map loaded successfully!")
        print(f"   YAML file: {yaml_path}")
        print(f"\nTo use with map_server:")
        print(f"   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path}")
        sys.exit(0)
    else:
        print("\n❌ Failed to load map")
        sys.exit(1)

if __name__ == '__main__':
    main()
