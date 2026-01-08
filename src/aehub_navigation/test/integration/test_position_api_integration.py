#!/usr/bin/env python3
"""
Integration tests for Position API

Tests the FastAPI endpoints for position management:
- GET /api/positions
- GET /api/positions/{position_id}
- POST /api/positions
- DELETE /api/positions/{position_id}
"""

import pytest
import requests
import json
import os
import sys
import time
import subprocess
import signal

# Add path to test server
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../scripts/nav2_test_server'))


class TestPositionAPI:
    """Integration tests for Position API"""
    
    @pytest.fixture(scope="class")
    def api_server(self):
        """Start API server for testing"""
        # This would start the actual server
        # For now, assume server is running or skip if not available
        base_url = "http://localhost:8000"
        
        # Check if server is running
        try:
            response = requests.get(f"{base_url}/health", timeout=2)
            if response.status_code == 200:
                yield base_url
            else:
                pytest.skip("API server not available")
        except requests.exceptions.RequestException:
            pytest.skip("API server not running")
    
    def test_get_all_positions(self, api_server):
        """Test GET /api/positions"""
        response = requests.get(f"{api_server}/api/positions")
        
        assert response.status_code == 200
        data = response.json()
        assert 'positions' in data
        assert 'count' in data
        assert isinstance(data['positions'], list)
        assert data['count'] == len(data['positions'])
    
    def test_get_single_position(self, api_server):
        """Test GET /api/positions/{position_id}"""
        # First, get all positions to find an existing one
        all_positions = requests.get(f"{api_server}/api/positions").json()
        
        if all_positions['count'] > 0:
            position_id = all_positions['positions'][0]['position_id']
            
            response = requests.get(f"{api_server}/api/positions/{position_id}")
            assert response.status_code == 200
            
            data = response.json()
            assert data['position_id'] == position_id
            assert 'x' in data
            assert 'y' in data
            assert 'theta' in data
        else:
            pytest.skip("No positions available for testing")
    
    def test_create_position(self, api_server):
        """Test POST /api/positions"""
        import uuid
        position_id = f"test_position_{uuid.uuid4().hex[:8]}"
        
        payload = {
            'position_id': position_id,
            'x': 10.0,
            'y': 20.0,
            'theta': 1.57,
            'description': 'Test position from integration test'
        }
        
        response = requests.post(
            f"{api_server}/api/positions",
            json=payload
        )
        
        assert response.status_code == 201
        data = response.json()
        assert data['position_id'] == position_id
        assert data['x'] == 10.0
        assert data['y'] == 20.0
        
        # Cleanup: delete the test position
        requests.delete(f"{api_server}/api/positions/{position_id}")
    
    def test_create_position_validation(self, api_server):
        """Test POST /api/positions with invalid data"""
        # Test with invalid position_id (contains invalid characters)
        invalid_payload = {
            'position_id': 'invalid-position!@#',
            'x': 10.0,
            'y': 20.0,
            'theta': 0.0
        }
        
        response = requests.post(
            f"{api_server}/api/positions",
            json=invalid_payload
        )
        
        assert response.status_code == 422  # Validation error
        
        # Test with out-of-range coordinates
        invalid_payload2 = {
            'position_id': 'test_position',
            'x': 2000.0,  # Out of range
            'y': 20.0,
            'theta': 0.0
        }
        
        response = requests.post(
            f"{api_server}/api/positions",
            json=invalid_payload2
        )
        
        assert response.status_code == 422  # Validation error
    
    def test_delete_position(self, api_server):
        """Test DELETE /api/positions/{position_id}"""
        import uuid
        position_id = f"test_delete_{uuid.uuid4().hex[:8]}"
        
        # First create a position
        payload = {
            'position_id': position_id,
            'x': 5.0,
            'y': 5.0,
            'theta': 0.0,
            'description': 'Position to be deleted'
        }
        
        create_response = requests.post(
            f"{api_server}/api/positions",
            json=payload
        )
        assert create_response.status_code == 201
        
        # Now delete it
        delete_response = requests.delete(f"{api_server}/api/positions/{position_id}")
        assert delete_response.status_code == 200
        
        data = delete_response.json()
        assert data['success'] is True
        assert data['position_id'] == position_id
        
        # Verify it's deleted
        get_response = requests.get(f"{api_server}/api/positions/{position_id}")
        assert get_response.status_code == 404
    
    def test_delete_nonexistent_position(self, api_server):
        """Test DELETE /api/positions/{position_id} with non-existent ID"""
        response = requests.delete(f"{api_server}/api/positions/nonexistent_position_12345")
        assert response.status_code == 404


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

