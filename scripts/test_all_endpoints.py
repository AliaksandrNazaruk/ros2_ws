#!/usr/bin/env python3
"""
Comprehensive endpoint testing script
Tests FastAPI and Symovo API endpoints
"""

import requests
import json
import sys
from typing import Dict, Any

FASTAPI_URL = "http://localhost:8000"
SYMOVO_ENDPOINT = "https://192.168.1.100"
AMR_ID = 15

def test_endpoint(method: str, url: str, expected_status: int = 200, 
                 json_data: Dict = None, description: str = "") -> tuple[bool, Any]:
    """Test an endpoint"""
    try:
        if method.upper() == 'GET':
            response = requests.get(url, timeout=5.0, verify=False)
        elif method.upper() == 'POST':
            response = requests.post(url, json=json_data, timeout=5.0, verify=False)
        elif method.upper() == 'PUT':
            response = requests.put(url, json=json_data, timeout=5.0, verify=False)
        else:
            return False, f"Unsupported method: {method}"
        
        success = response.status_code == expected_status
        try:
            data = response.json() if response.text else None
        except:
            data = response.text[:200] if response.text else None
        
        return success, {
            'status': response.status_code,
            'data': data,
            'expected': expected_status
        }
    except Exception as e:
        return False, {'error': str(e)}

def print_result(name: str, success: bool, result: Any):
    """Print test result"""
    if success:
        print(f" {name}: OK")
        if isinstance(result, dict) and 'data' in result:
            if isinstance(result['data'], dict) and len(result['data']) < 10:
                print(f"   Data: {json.dumps(result['data'], indent=2)}")
    else:
        print(f" {name}: FAILED")
        print(f"   {result}")

def main():
    print("=" * 60)
    print("Comprehensive Endpoint Testing")
    print("=" * 60)
    print()
    
    # Disable SSL warnings
    import urllib3
    urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
    
    results = {
        'fastapi': {'passed': 0, 'failed': 0},
        'symovo': {'passed': 0, 'failed': 0}
    }
    
    # FastAPI Tests
    print("🔵 FastAPI Endpoints:")
    print("-" * 60)
    
    # Health
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/monitor/health")
    print_result("GET /api/monitor/health", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Status
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/monitor/status")
    print_result("GET /api/monitor/status", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Nodes
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/monitor/nodes")
    print_result("GET /api/monitor/nodes", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Topics
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/monitor/topics")
    print_result("GET /api/monitor/topics", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Positions
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/positions")
    print_result("GET /api/positions", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    # Process Status
    success, result = test_endpoint('GET', f"{FASTAPI_URL}/api/process/status")
    print_result("GET /api/process/status", success, result)
    if success:
        results['fastapi']['passed'] += 1
    else:
        results['fastapi']['failed'] += 1
    
    print()
    
    # Symovo API Tests
    print("🟢 Symovo API Endpoints:")
    print("-" * 60)
    
    # AGV List
    success, result = test_endpoint('GET', f"{SYMOVO_ENDPOINT}/v0/agv")
    print_result("GET /v0/agv", success, result)
    if success:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # AMR Pose
    success, result = test_endpoint('GET', f"{SYMOVO_ENDPOINT}/v0/amr/{AMR_ID}/pose", expected_status=200)
    print_result(f"GET /v0/amr/{AMR_ID}/pose", success, result)
    if success:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # Move Speed (stop command)
    success, result = test_endpoint('PUT', f"{SYMOVO_ENDPOINT}/v0/amr/{AMR_ID}/move/speed",
                                   json_data={'speed': 0.0, 'angular_speed': 0.0, 'duration': 0.1},
                                   expected_status=202)
    print_result(f"PUT /v0/amr/{AMR_ID}/move/speed (stop)", success, result)
    if success:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # Scan PNG
    success, result = test_endpoint('GET', f"{SYMOVO_ENDPOINT}/v0/amr/{AMR_ID}/scan.png", expected_status=200)
    print_result(f"GET /v0/amr/{AMR_ID}/scan.png", success, result)
    if success:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    # Maps
    success, result = test_endpoint('GET', f"{SYMOVO_ENDPOINT}/v0/map")
    print_result("GET /v0/map", success, result)
    if success:
        results['symovo']['passed'] += 1
    else:
        results['symovo']['failed'] += 1
    
    print()
    print("=" * 60)
    print("Summary:")
    print("=" * 60)
    print(f"FastAPI:  {results['fastapi']['passed']} passed,  {results['fastapi']['failed']} failed")
    print(f"Symovo:   {results['symovo']['passed']} passed,  {results['symovo']['failed']} failed")
    print()
    
    total_passed = results['fastapi']['passed'] + results['symovo']['passed']
    total_failed = results['fastapi']['failed'] + results['symovo']['failed']
    print(f"Total:  {total_passed} passed,  {total_failed} failed")
    
    return 0 if total_failed == 0 else 1

if __name__ == '__main__':
    sys.exit(main())

