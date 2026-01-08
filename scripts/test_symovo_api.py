#!/usr/bin/env python3

"""
Symovo API Testing Script

Tests all Symovo API endpoints used by the navigation system.
Validates request/response formats, error handling, and data parsing.
"""

import requests
import json
import sys
import time
from typing import Optional, Dict, Any
import urllib3

# Disable SSL warnings for testing
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class SymovoAPITester:
    """Test suite for Symovo API endpoints"""
    
    def __init__(self, endpoint: str, amr_id: int, tls_verify: bool = False):
        """
        Initialize API tester.
        
        Args:
            endpoint: Symovo API endpoint (e.g., "https://192.168.1.100")
            amr_id: AMR ID to test with
            tls_verify: Whether to verify TLS certificates
        """
        self.endpoint = endpoint.rstrip('/')
        self.amr_id = amr_id
        self.tls_verify = tls_verify
        self.session = requests.Session()
        self.session.verify = tls_verify
        
        self.test_results = {
            'passed': [],
            'failed': [],
            'warnings': []
        }
    
    def log(self, level: str, message: str):
        """Log message with level"""
        prefix = {
            'info': 'ℹ ',
            'success': '',
            'error': '',
            'warning': ' ',
            'debug': ''
        }.get(level, '')
        print(f"{prefix} [{level.upper()}] {message}")
    
    def test_endpoint(self, method: str, path: str, expected_status: int = 200,
                     payload: Optional[Dict] = None, description: str = "") -> bool:
        """
        Test an API endpoint.
        
        Returns:
            True if test passed, False otherwise
        """
        url = f"{self.endpoint}{path}"
        test_name = f"{method} {path}"
        if description:
            test_name += f" - {description}"
        
        try:
            if method.upper() == 'GET':
                response = self.session.get(url, timeout=5.0)
            elif method.upper() == 'PUT':
                response = self.session.put(
                    url,
                    json=payload if payload else {},
                    headers={'Content-Type': 'application/json'},
                    timeout=5.0
                )
            elif method.upper() == 'POST':
                response = self.session.post(
                    url,
                    json=payload if payload else {},
                    headers={'Content-Type': 'application/json'},
                    timeout=5.0
                )
            else:
                self.log('error', f"Unsupported method: {method}")
                return False
            
            if response.status_code == expected_status:
                self.log('success', f"{test_name}: OK (HTTP {response.status_code})")
                self.test_results['passed'].append(test_name)
                return True
            else:
                self.log('error', 
                    f"{test_name}: FAILED - Expected {expected_status}, got {response.status_code}")
                if response.text:
                    self.log('debug', f"Response: {response.text[:200]}")
                self.test_results['failed'].append((test_name, response.status_code, expected_status))
                return False
                
        except requests.exceptions.Timeout:
            self.log('error', f"{test_name}: TIMEOUT")
            self.test_results['failed'].append((test_name, 'TIMEOUT', expected_status))
            return False
        except requests.exceptions.ConnectionError as e:
            self.log('error', f"{test_name}: CONNECTION ERROR - {e}")
            self.test_results['failed'].append((test_name, 'CONNECTION_ERROR', expected_status))
            return False
        except Exception as e:
            self.log('error', f"{test_name}: EXCEPTION - {e}")
            self.test_results['failed'].append((test_name, str(e), expected_status))
            return False
    
    def test_get_agv_list(self) -> bool:
        """Test GET /v0/agv - Get list of AGV objects"""
        url = f"{self.endpoint}/v0/agv"
        
        try:
            response = self.session.get(url, timeout=5.0)
            
            if response.status_code == 200:
                data = response.json()
                
                # Validate response format
                if not isinstance(data, list):
                    self.log('error', "GET /v0/agv: Response is not an array")
                    self.test_results['failed'].append(("GET /v0/agv", "Invalid format", 200))
                    return False
                
                # Find our AMR
                amr_found = False
                for amr in data:
                    if isinstance(amr, dict) and amr.get('id') == self.amr_id:
                        amr_found = True
                        
                        # Validate structure
                        has_pose = 'pose' in amr
                        has_velocity = 'velocity' in amr
                        
                        if has_pose:
                            pose = amr['pose']
                            required_pose_fields = ['x', 'y', 'theta']
                            missing = [f for f in required_pose_fields if f not in pose]
                            if missing:
                                self.log('warning', 
                                    f"GET /v0/agv: AMR {self.amr_id} pose missing fields: {missing}")
                                self.test_results['warnings'].append(
                                    f"AMR {self.amr_id} pose missing: {missing}")
                        
                        if has_velocity:
                            velocity = amr['velocity']
                            required_vel_fields = ['x', 'y', 'theta']
                            missing = [f for f in required_vel_fields if f not in velocity]
                            if missing:
                                self.log('warning',
                                    f"GET /v0/agv: AMR {self.amr_id} velocity missing fields: {missing}")
                                self.test_results['warnings'].append(
                                    f"AMR {self.amr_id} velocity missing: {missing}")
                        
                        self.log('success', 
                            f"GET /v0/agv: Found AMR {self.amr_id}, pose={has_pose}, velocity={has_velocity}")
                        break
                
                if not amr_found:
                    self.log('warning', f"GET /v0/agv: AMR {self.amr_id} not found in response")
                    self.test_results['warnings'].append(f"AMR {self.amr_id} not found")
                
                self.test_results['passed'].append("GET /v0/agv")
                return True
            else:
                self.log('error', f"GET /v0/agv: HTTP {response.status_code}")
                self.test_results['failed'].append(("GET /v0/agv", response.status_code, 200))
                return False
                
        except Exception as e:
            self.log('error', f"GET /v0/agv: Exception - {e}")
            self.test_results['failed'].append(("GET /v0/agv", str(e), 200))
            return False
    
    def test_move_speed_command(self) -> bool:
        """Test PUT /v0/amr/{id}/move/speed - Send movement command"""
        path = f"/v0/amr/{self.amr_id}/move/speed"
        payload = {
            "speed": 0.0,  # Stop command for safety
            "angular_speed": 0.0,
            "duration": 0.05
        }
        
        # Test with stop command (safe)
        result = self.test_endpoint('PUT', path, expected_status=202, payload=payload,
                                   description="Stop command")
        
        if result:
            self.log('info', "Move command accepted (stop command for safety)")
        
        return result
    
    def test_scan_png(self) -> bool:
        """Test GET /v0/amr/{id}/scan.png - Get laser scan as PNG"""
        path = f"/v0/amr/{self.amr_id}/scan.png"
        url = f"{self.endpoint}{path}"
        
        try:
            response = self.session.get(url, timeout=5.0)
            
            if response.status_code == 200:
                # Check content type
                content_type = response.headers.get('Content-Type', '')
                if 'image' in content_type.lower() or 'png' in content_type.lower():
                    size = len(response.content)
                    self.log('success', 
                        f"GET {path}: OK - PNG image received ({size} bytes)")
                    self.test_results['passed'].append(f"GET {path}")
                    return True
                else:
                    self.log('warning', 
                        f"GET {path}: Unexpected Content-Type: {content_type}")
                    self.test_results['warnings'].append(f"{path} Content-Type: {content_type}")
                    # Still count as passed if we got data
                    if len(response.content) > 0:
                        self.test_results['passed'].append(f"GET {path}")
                        return True
                    return False
            elif response.status_code == 404:
                self.log('warning', f"GET {path}: AMR {self.amr_id} not found (404)")
                self.test_results['warnings'].append(f"{path} - AMR not found")
                return False
            elif response.status_code == 503:
                self.log('warning', f"GET {path}: AMR not ready (503) - may be normal")
                self.test_results['warnings'].append(f"{path} - AMR not ready")
                return False
            else:
                self.log('error', f"GET {path}: HTTP {response.status_code}")
                self.test_results['failed'].append((f"GET {path}", response.status_code, 200))
                return False
                
        except Exception as e:
            self.log('error', f"GET {path}: Exception - {e}")
            self.test_results['failed'].append((f"GET {path}", str(e), 200))
            return False
    
    def test_amr_pose(self) -> bool:
        """Test GET /v0/amr/{id}/pose - Get AMR pose"""
        path = f"/v0/amr/{self.amr_id}/pose"
        return self.test_endpoint('GET', path, expected_status=200, 
                                 description="Get AMR pose")
    
    def run_all_tests(self):
        """Run all API tests"""
        self.log('info', f"Starting Symovo API tests")
        self.log('info', f"Endpoint: {self.endpoint}")
        self.log('info', f"AMR ID: {self.amr_id}")
        self.log('info', f"TLS Verify: {self.tls_verify}")
        print()
        
        # Test endpoints
        self.log('info', "Testing GET /v0/agv (AGV list)...")
        self.test_get_agv_list()
        print()
        
        self.log('info', f"Testing GET /v0/amr/{self.amr_id}/pose...")
        self.test_amr_pose()
        print()
        
        self.log('info', f"Testing PUT /v0/amr/{self.amr_id}/move/speed (stop command)...")
        self.test_move_speed_command()
        print()
        
        self.log('info', f"Testing GET /v0/amr/{self.amr_id}/scan.png...")
        self.test_scan_png()
        print()
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary"""
        print("=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        
        total = len(self.test_results['passed']) + len(self.test_results['failed'])
        passed = len(self.test_results['passed'])
        failed = len(self.test_results['failed'])
        warnings = len(self.test_results['warnings'])
        
        print(f"Total tests: {total}")
        print(f" Passed: {passed}")
        print(f" Failed: {failed}")
        print(f"  Warnings: {warnings}")
        print()
        
        if self.test_results['failed']:
            print("FAILED TESTS:")
            for test in self.test_results['failed']:
                if isinstance(test, tuple):
                    print(f"  - {test[0]}: Expected {test[2]}, got {test[1]}")
                else:
                    print(f"  - {test}")
            print()
        
        if self.test_results['warnings']:
            print("WARNINGS:")
            for warning in self.test_results['warnings']:
                print(f"  - {warning}")
            print()
        
        if failed == 0:
            self.log('success', "All tests passed!")
            return 0
        else:
            self.log('error', f"{failed} test(s) failed")
            return 1


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Symovo API endpoints')
    parser.add_argument('--endpoint', type=str, default='https://192.168.1.100',
                       help='Symovo API endpoint')
    parser.add_argument('--amr-id', type=int, default=15,
                       help='AMR ID to test with')
    parser.add_argument('--tls-verify', action='store_true',
                       help='Verify TLS certificates (default: False)')
    
    args = parser.parse_args()
    
    tester = SymovoAPITester(
        endpoint=args.endpoint,
        amr_id=args.amr_id,
        tls_verify=args.tls_verify
    )
    
    exit_code = tester.run_all_tests()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

