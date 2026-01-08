#!/usr/bin/env python3
"""
Comprehensive Test Runner

Runs all tests for the navigation node:
- Unit tests (pytest)
- Integration tests (pytest)
- Fake Hub end-to-end tests (test_chain_extended.py)

Generates coverage report and test summary.
"""

import sys
import os
import subprocess
import argparse
import time
from pathlib import Path
from typing import List, Tuple, Optional
import requests


# Add scripts directory to path
SCRIPT_DIR = Path(__file__).parent
WORKSPACE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))


class TestRunner:
    """Comprehensive test runner for navigation node"""
    
    def __init__(self, workspace_dir: Path, verbose: bool = False, coverage: bool = False):
        """
        Initialize test runner.
        
        Args:
            workspace_dir: ROS2 workspace directory
            verbose: Enable verbose output
            coverage: Generate coverage report
        """
        self.workspace_dir = Path(workspace_dir)
        self.verbose = verbose
        self.coverage = coverage
        self.results = {
            'unit': {'passed': 0, 'failed': 0, 'total': 0},
            'integration': {'passed': 0, 'failed': 0, 'total': 0},
            'fake_hub': {'passed': 0, 'failed': 0, 'total': 0}
        }
    
    def check_prerequisites(self) -> Tuple[bool, List[str]]:
        """
        Check prerequisites before running tests.
        
        Returns:
            Tuple of (all_ok, missing_items)
        """
        missing = []
        
        # Check Config Service
        try:
            response = requests.get('http://localhost:7900/api/v1/config/broker', timeout=2.0)
            if response.status_code != 200:
                missing.append('Config Service (http://localhost:7900) - not responding correctly')
        except requests.exceptions.RequestException:
            missing.append('Config Service (http://localhost:7900) - not available')
        
        # Check pytest
        try:
            import pytest
        except ImportError:
            missing.append('pytest - install with: pip install pytest pytest-cov')
        
        # Check ROS2
        try:
            import rclpy
        except ImportError:
            missing.append('rclpy - ROS2 Python package not found')
        
        return len(missing) == 0, missing
    
    def run_unit_tests(self) -> bool:
        """
        Run unit tests using pytest.
        
        Returns:
            True if all tests passed, False otherwise
        """
        print("\n" + "="*70)
        print(" UNIT TESTS")
        print("="*70)
        
        test_dir = self.workspace_dir / 'src' / 'aehub_navigation' / 'test' / 'unit'
        
        if not test_dir.exists():
            print(f" Unit test directory not found: {test_dir}")
            return False
        
        cmd = ['python3', '-m', 'pytest', str(test_dir), '-v']
        
        if self.coverage:
            cmd.extend(['--cov=aehub_navigation', '--cov-report=term-missing', '--cov-report=html'])
        
        if not self.verbose:
            cmd.append('-q')
        
        print(f"Running: {' '.join(cmd)}")
        print()
        
        try:
            result = subprocess.run(cmd, cwd=self.workspace_dir, capture_output=not self.verbose)
            
            if result.returncode == 0:
                print(" Unit tests passed")
                return True
            else:
                print(" Unit tests failed")
                if not self.verbose and result.stdout:
                    print(result.stdout.decode('utf-8', errors='ignore'))
                if result.stderr:
                    print(result.stderr.decode('utf-8', errors='ignore'))
                return False
        except Exception as e:
            print(f" Error running unit tests: {e}")
            return False
    
    def run_integration_tests(self) -> bool:
        """
        Run integration tests using pytest.
        
        Returns:
            True if all tests passed, False otherwise
        """
        print("\n" + "="*70)
        print(" INTEGRATION TESTS")
        print("="*70)
        
        test_dir = self.workspace_dir / 'src' / 'aehub_navigation' / 'test' / 'integration'
        
        if not test_dir.exists():
            print(f" Integration test directory not found: {test_dir}")
            return False
        
        cmd = ['python3', '-m', 'pytest', str(test_dir), '-v', '--tb=short']
        
        if self.coverage:
            cmd.extend(['--cov=aehub_navigation', '--cov-append', '--cov-report=term-missing'])
        
        if not self.verbose:
            cmd.append('-q')
        
        print(f"Running: {' '.join(cmd)}")
        print()
        print("  Note: Integration tests require ROS2 context and may take longer")
        print()
        
        try:
            result = subprocess.run(cmd, cwd=self.workspace_dir, capture_output=not self.verbose)
            
            if result.returncode == 0:
                print(" Integration tests passed")
                return True
            else:
                print(" Integration tests failed")
                if not self.verbose and result.stdout:
                    print(result.stdout.decode('utf-8', errors='ignore'))
                if result.stderr:
                    print(result.stderr.decode('utf-8', errors='ignore'))
                return False
        except Exception as e:
            print(f" Error running integration tests: {e}")
            return False
    
    def run_fake_hub_tests(self) -> bool:
        """
        Run fake hub end-to-end tests.
        
        Returns:
            True if all tests passed, False otherwise
        """
        print("\n" + "="*70)
        print(" FAKE HUB END-TO-END TESTS")
        print("="*70)
        
        test_script = self.workspace_dir / 'scripts' / 'test_chain_extended.py'
        
        if not test_script.exists():
            print(f" Fake hub test script not found: {test_script}")
            return False
        
        cmd = ['python3', str(test_script)]
        
        if self.verbose:
            cmd.append('--verbose')
        
        print(f"Running: {' '.join(cmd)}")
        print()
        print("  Note: Fake hub tests require Config Service and MQTT broker")
        print()
        
        try:
            result = subprocess.run(cmd, cwd=self.workspace_dir, capture_output=not self.verbose)
            
            if result.returncode == 0:
                print(" Fake hub tests passed")
                return True
            else:
                print(" Fake hub tests failed")
                if not self.verbose and result.stdout:
                    print(result.stdout.decode('utf-8', errors='ignore'))
                if result.stderr:
                    print(result.stderr.decode('utf-8', errors='ignore'))
                return False
        except Exception as e:
            print(f" Error running fake hub tests: {e}")
            return False
    
    def run_all(self, skip_fake_hub: bool = False) -> bool:
        """
        Run all tests.
        
        Args:
            skip_fake_hub: Skip fake hub tests (useful when Config Service is not available)
        
        Returns:
            True if all tests passed, False otherwise
        """
        print("="*70)
        print(" COMPREHENSIVE TEST RUNNER")
        print("="*70)
        print(f"Workspace: {self.workspace_dir}")
        print(f"Coverage: {'Enabled' if self.coverage else 'Disabled'}")
        print()
        
        # Check prerequisites
        print(" Checking prerequisites...")
        all_ok, missing = self.check_prerequisites()
        
        if not all_ok:
            print("  Missing prerequisites:")
            for item in missing:
                print(f"   - {item}")
            print()
            if 'Config Service' in str(missing):
                print(" Tip: Config Service is required for fake_hub tests.")
                print("   You can skip them with --skip-fake-hub")
                print()
                if not skip_fake_hub:
                    response = input("Continue without fake_hub tests? (y/n): ")
                    if response.lower() == 'y':
                        skip_fake_hub = True
                    else:
                        return False
        else:
            print(" All prerequisites met")
            print()
        
        start_time = time.time()
        results = []
        
        # Run unit tests
        results.append(('Unit Tests', self.run_unit_tests()))
        
        # Run integration tests
        results.append(('Integration Tests', self.run_integration_tests()))
        
        # Run fake hub tests (if not skipped)
        if not skip_fake_hub:
            results.append(('Fake Hub Tests', self.run_fake_hub_tests()))
        else:
            print("\n" + "="*70)
            print("⏭  FAKE HUB TESTS SKIPPED")
            print("="*70)
            print("Skipped due to missing prerequisites or user request")
            results.append(('Fake Hub Tests', None))  # None = skipped
        
        elapsed_time = time.time() - start_time
        
        # Print summary
        print("\n" + "="*70)
        print(" TEST SUMMARY")
        print("="*70)
        
        passed = sum(1 for _, result in results if result is True)
        failed = sum(1 for _, result in results if result is False)
        skipped = sum(1 for _, result in results if result is None)
        total = len(results)
        
        for test_name, result in results:
            if result is True:
                status = " PASSED"
            elif result is False:
                status = " FAILED"
            else:
                status = "⏭  SKIPPED"
            print(f"  {test_name:.<40} {status}")
        
        print()
        print(f"Total: {total} | Passed: {passed} | Failed: {failed} | Skipped: {skipped}")
        print(f"Time: {elapsed_time:.2f}s")
        print()
        
        if self.coverage:
            print(" Coverage report generated in htmlcov/index.html")
            print()
        
        if failed == 0:
            print(" ALL TESTS PASSED!")
            return True
        else:
            print(f"  {failed} test suite(s) failed")
            return False


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Run all tests for navigation node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all tests
  python3 scripts/run_all_tests.py
  
  # Run with coverage
  python3 scripts/run_all_tests.py --coverage
  
  # Run verbose
  python3 scripts/run_all_tests.py --verbose
  
  # Skip fake hub tests
  python3 scripts/run_all_tests.py --skip-fake-hub
        """
    )
    
    parser.add_argument(
        '--workspace',
        type=str,
        default=str(Path(__file__).parent.parent),
        help='ROS2 workspace directory (default: parent of scripts/)'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )
    
    parser.add_argument(
        '--coverage', '-c',
        action='store_true',
        help='Generate coverage report'
    )
    
    parser.add_argument(
        '--skip-fake-hub',
        action='store_true',
        help='Skip fake hub end-to-end tests'
    )
    
    parser.add_argument(
        '--unit-only',
        action='store_true',
        help='Run only unit tests'
    )
    
    parser.add_argument(
        '--integration-only',
        action='store_true',
        help='Run only integration tests'
    )
    
    parser.add_argument(
        '--fake-hub-only',
        action='store_true',
        help='Run only fake hub tests'
    )
    
    args = parser.parse_args()
    
    runner = TestRunner(
        workspace_dir=Path(args.workspace),
        verbose=args.verbose,
        coverage=args.coverage
    )
    
    if args.unit_only:
        success = runner.run_unit_tests()
    elif args.integration_only:
        success = runner.run_integration_tests()
    elif args.fake_hub_only:
        success = runner.run_fake_hub_tests()
    else:
        success = runner.run_all(skip_fake_hub=args.skip_fake_hub)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

