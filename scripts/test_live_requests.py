#!/usr/bin/env python3
"""
Live Testing Script - Real requests to Symovo API and FastAPI
Tests actual functionality with real data
"""

import requests
import json
import time
import sys
from datetime import datetime
import urllib3

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

FASTAPI_URL = "http://localhost:8000"
SYMOVO_ENDPOINT = "https://192.168.1.100"
AMR_ID = 15

def print_section(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def print_result(name, success, data=None, error=None):
    if success:
        print(f" {name}")
        if data and isinstance(data, dict) and len(str(data)) < 500:
            print(f"   {json.dumps(data, indent=2)}")
    else:
        print(f" {name}")
        if error:
            print(f"   Error: {error}")
        elif data:
            print(f"   Response: {data}")

def test_symovo_agv():
    """Test GET /v0/agv - Get AMR status"""
    print_section("Symovo API - AMR Status")
    
    try:
        response = requests.get(f"{SYMOVO_ENDPOINT}/v0/agv", verify=False, timeout=5)
        if response.status_code == 200:
            data = response.json()
            amr = next((a for a in data if a.get('id') == AMR_ID), None)
            if amr:
                print_result("GET /v0/agv", True, {
                    'amr_id': amr['id'],
                    'name': amr.get('name'),
                    'pose': amr.get('pose'),
                    'velocity': amr.get('velocity'),
                    'state': amr.get('state'),
                    'battery': amr.get('battery_level', 0) * 100
                })
                return amr
            else:
                print_result("GET /v0/agv", False, error=f"AMR {AMR_ID} not found")
        else:
            print_result("GET /v0/agv", False, error=f"HTTP {response.status_code}")
    except Exception as e:
        print_result("GET /v0/agv", False, error=str(e))
    return None

def test_symovo_maps():
    """Test GET /v0/map - Get maps"""
    print_section("Symovo API - Maps")
    
    try:
        response = requests.get(f"{SYMOVO_ENDPOINT}/v0/map", verify=False, timeout=5)
        if response.status_code == 200:
            data = response.json()
            if isinstance(data, list) and len(data) > 0:
                map_info = data[0]
                print_result("GET /v0/map", True, {
                    'map_id': map_info.get('id'),
                    'name': map_info.get('name'),
                    'resolution': map_info.get('resolution'),
                    'size': map_info.get('size')
                })
                return map_info
            else:
                print_result("GET /v0/map", True, {'maps': 0})
        else:
            print_result("GET /v0/map", False, error=f"HTTP {response.status_code}")
    except Exception as e:
        print_result("GET /v0/map", False, error=str(e))
    return None

def test_symovo_move_command():
    """Test PUT /v0/amr/{id}/move/speed - Send movement command"""
    print_section("Symovo API - Movement Command")
    
    # Send stop command (safe test)
    command = {
        'speed': 0.0,
        'angular_speed': 0.0,
        'duration': 0.1
    }
    
    try:
        response = requests.put(
            f"{SYMOVO_ENDPOINT}/v0/amr/{AMR_ID}/move/speed",
            json=command,
            verify=False,
            timeout=5
        )
        if response.status_code in [200, 202]:
            print_result("PUT /v0/amr/{id}/move/speed (stop)", True, command)
            return True
        else:
            print_result("PUT /v0/amr/{id}/move/speed", False, 
                        error=f"HTTP {response.status_code}: {response.text[:100]}")
    except Exception as e:
        print_result("PUT /v0/amr/{id}/move/speed", False, error=str(e))
    return False

def test_fastapi_health():
    """Test FastAPI health"""
    print_section("FastAPI - Health Check")
    
    try:
        response = requests.get(f"{FASTAPI_URL}/api/monitor/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print_result("GET /api/monitor/health", True, data)
            return data
        else:
            print_result("GET /api/monitor/health", False, error=f"HTTP {response.status_code}")
    except Exception as e:
        print_result("GET /api/monitor/health", False, error=str(e))
    return None

def test_fastapi_positions():
    """Test FastAPI positions"""
    print_section("FastAPI - Positions")
    
    try:
        response = requests.get(f"{FASTAPI_URL}/api/positions", timeout=5)
        if response.status_code == 200:
            data = response.json()
            positions = data.get('positions', [])
            print_result("GET /api/positions", True, {
                'count': len(positions),
                'positions': [{'id': p['position_id'], 'x': p['x'], 'y': p['y']} 
                             for p in positions[:5]]
            })
            return positions
        else:
            print_result("GET /api/positions", False, error=f"HTTP {response.status_code}")
    except Exception as e:
        print_result("GET /api/positions", False, error=str(e))
    return None

def test_fastapi_navigate_command():
    """Test sending navigation command via FastAPI"""
    print_section("FastAPI - Navigation Command")
    
    # Get first position
    positions = test_fastapi_positions()
    if not positions or len(positions) == 0:
        print("  No positions available for navigation test")
        return False
    
    target_id = positions[0]['position_id']
    
    command = {
        'target_id': target_id,
        'priority': 'normal'
    }
    
    try:
        response = requests.post(
            f"{FASTAPI_URL}/api/test/navigate",
            json=command,
            timeout=5
        )
        if response.status_code in [200, 202]:
            data = response.json()
            print_result("POST /api/test/navigate", True, data)
            return True
        else:
            print_result("POST /api/test/navigate", False,
                        error=f"HTTP {response.status_code}: {response.text[:200]}")
    except Exception as e:
        print_result("POST /api/test/navigate", False, error=str(e))
    return False

def test_fastapi_ros2_nodes():
    """Test FastAPI ROS2 nodes"""
    print_section("FastAPI - ROS2 Nodes")
    
    try:
        response = requests.get(f"{FASTAPI_URL}/api/monitor/nodes", timeout=5)
        if response.status_code == 200:
            data = response.json()
            nav_nodes = [n for n in data if any(keyword in n.get('name', '').lower() 
                          for keyword in ['nav', 'navigation', 'aehub', 'symovo'])]
            print_result("GET /api/monitor/nodes", True, {
                'total_nodes': len(data),
                'navigation_nodes': len(nav_nodes),
                'nav_node_names': [n['name'] for n in nav_nodes[:10]]
            })
            return nav_nodes
        else:
            print_result("GET /api/monitor/nodes", False, error=f"HTTP {response.status_code}")
    except Exception as e:
        print_result("GET /api/monitor/nodes", False, error=str(e))
    return None

def main():
    print("\n" + "=" * 60)
    print("  LIVE TESTING - Real Requests to Symovo & FastAPI")
    print("=" * 60)
    print(f"Time: {datetime.now().isoformat()}")
    print(f"FastAPI: {FASTAPI_URL}")
    print(f"Symovo: {SYMOVO_ENDPOINT}")
    print(f"AMR ID: {AMR_ID}")
    
    results = {
        'symovo': {'passed': 0, 'failed': 0},
        'fastapi': {'passed': 0, 'failed': 0}
    }
    
    # Symovo API Tests
    amr_data = test_symovo_agv()
    if amr_data:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    map_data = test_symovo_maps()
    if map_data:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # Note: Movement command test is optional (may fail if AMR not ready)
    move_result = test_symovo_move_command()
    if move_result:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # FastAPI Tests
    health = test_fastapi_health()
    if health:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    positions = test_fastapi_positions()
    if positions:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    nodes = test_fastapi_ros2_nodes()
    if nodes is not None:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Navigation command test (optional)
    nav_result = test_fastapi_navigate_command()
    if nav_result:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Summary
    print_section("Test Summary")
    print(f"Symovo API:   {results['symovo']['passed']} passed,  {results['symovo']['failed']} failed")
    print(f"FastAPI:      {results['fastapi']['passed']} passed,  {results['fastapi']['failed']} failed")
    
    total_passed = results['symovo']['passed'] + results['fastapi']['passed']
    total_failed = results['symovo']['failed'] + results['fastapi']['failed']
    print(f"\nTotal:        {total_passed} passed,  {total_failed} failed")
    
    if amr_data:
        print(f"\n AMR {AMR_ID} Status:")
        print(f"   Position: x={amr_data['pose']['x']:.3f}, y={amr_data['pose']['y']:.3f}, "
              f"theta={amr_data['pose']['theta']:.3f}")
        print(f"   State: {amr_data.get('state', 'unknown')}")
        print(f"   Battery: {amr_data.get('battery_level', 0) * 100:.1f}%")
    
    return 0 if total_failed == 0 else 1

if __name__ == '__main__':
    sys.exit(main())

