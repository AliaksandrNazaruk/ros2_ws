#!/usr/bin/env python3
"""
Test script for robot_service positions API

Tests the robot_service API endpoints for position management.
"""

import requests
import json
import sys
from typing import Dict, Any, List

BASE_URL = "http://localhost:8110"
API_BASE = BASE_URL  # API paths don't have /api/v1/robot prefix


def test_list_positions() -> bool:
    """Test GET /robot_positions/list"""
    print("🧪 Testing GET /robot_positions/list")
    
    try:
        response = requests.get(f"{API_BASE}/robot_positions/list", timeout=5)
        print(f"   Status: {response.status_code}")
        
        if response.status_code == 200:
            try:
                data = response.json()
                print(f"   ✅ Success! Response: {json.dumps(data, indent=2)}")
                return True
            except json.JSONDecodeError:
                print(f"   ⚠️  Response is not JSON: {response.text[:200]}")
                return False
        else:
            print(f"   ❌ Error: {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False
            
    except requests.exceptions.ConnectionError:
        print(f"   ❌ Connection error: Cannot connect to {BASE_URL}")
        print(f"   Make sure robot_service is running on port 8110")
        return False
    except Exception as e:
        print(f"   ❌ Error: {str(e)}")
        return False


def test_get_position(position_id: str) -> bool:
    """Test getting a specific position"""
    print(f"\n🧪 Testing GET position: {position_id}")
    
    try:
        # First, get list to find a position
        response = requests.get(f"{API_BASE}/robot_positions/list", timeout=5)
        if response.status_code != 200:
            print(f"   ❌ Cannot get positions list: {response.status_code}")
            return False
        
        positions = response.json()
        if isinstance(positions, list) and len(positions) > 0:
            # Try to find the position
            found = False
            for pos in positions:
                pos_id = pos.get('position_id') or pos.get('id') or pos.get('name', '')
                if pos_id == position_id:
                    print(f"   ✅ Found position: {json.dumps(pos, indent=2)}")
                    found = True
                    break
            
            if not found:
                print(f"   ⚠️  Position '{position_id}' not found in list")
                print(f"   Available positions: {[p.get('position_id') or p.get('id') or p.get('name', '') for p in positions]}")
            
            return found
        else:
            print(f"   ⚠️  No positions found or unexpected format")
            return False
            
    except Exception as e:
        print(f"   ❌ Error: {str(e)}")
        return False


def test_save_position(position_id: str, x: float, y: float, theta: float, description: str = "") -> bool:
    """Test POST /robot_positions/save"""
    print(f"\n🧪 Testing POST /robot_positions/save")
    print(f"   Position: {position_id} at ({x}, {y}, {theta})")
    
    try:
        payload = {
            "position_id": position_id,
            "x": x,
            "y": y,
            "theta": theta,
            "description": description
        }
        
        response = requests.post(
            f"{API_BASE}/robot_positions/save",
            json=payload,
            timeout=5
        )
        
        print(f"   Status: {response.status_code}")
        
        if response.status_code in [200, 201]:
            try:
                data = response.json()
                print(f"   ✅ Success! Response: {json.dumps(data, indent=2)}")
                return True
            except json.JSONDecodeError:
                print(f"   ⚠️  Response is not JSON: {response.text[:200]}")
                return response.status_code == 201
        else:
            print(f"   ❌ Error: {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False
            
    except Exception as e:
        print(f"   ❌ Error: {str(e)}")
        return False


def test_delete_position(position_id: str) -> bool:
    """Test POST /robot_positions/delete"""
    print(f"\n🧪 Testing POST /robot_positions/delete")
    print(f"   Position ID: {position_id}")
    
    try:
        response = requests.post(
            f"{API_BASE}/robot_positions/delete",
            params={"position_id": position_id},
            timeout=5
        )
        
        print(f"   Status: {response.status_code}")
        
        if response.status_code in [200, 201]:
            try:
                data = response.json()
                print(f"   ✅ Success! Response: {json.dumps(data, indent=2)}")
                return True
            except json.JSONDecodeError:
                print(f"   ⚠️  Response is not JSON: {response.text[:200]}")
                return True
        else:
            print(f"   ❌ Error: {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False
            
    except Exception as e:
        print(f"   ❌ Error: {str(e)}")
        return False


def main():
    """Main test function"""
    print("=" * 60)
    print("🚀 Testing robot_service Positions API")
    print("=" * 60)
    print(f"Base URL: {BASE_URL}\n")
    
    results = []
    
    # Test 1: List positions
    results.append(("List positions", test_list_positions()))
    
    # Test 2: Save a test position
    test_pos_id = "test_position_nav2"
    results.append(("Save test position", test_save_position(
        test_pos_id, 1.0, 2.0, 0.0, "Test position from nav2_test_server"
    )))
    
    # Test 3: List positions again (should include new one)
    results.append(("List positions (after save)", test_list_positions()))
    
    # Test 4: Get specific position
    results.append(("Get test position", test_get_position(test_pos_id)))
    
    # Test 5: Delete test position
    results.append(("Delete test position", test_delete_position(test_pos_id)))
    
    # Test 6: List positions again (should not include deleted one)
    results.append(("List positions (after delete)", test_list_positions()))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed!")
        sys.exit(0)
    else:
        print("⚠️  Some tests failed")
        sys.exit(1)


if __name__ == "__main__":
    main()

