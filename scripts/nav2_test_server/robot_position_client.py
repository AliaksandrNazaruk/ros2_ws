"""
Robot Position Client

Client for interacting with robot_service API for position management.
This replaces the local file-based PositionRegistry with API-based storage.
"""

import httpx
from typing import Optional, List, Dict, Any
from datetime import datetime


class RobotPositionClient:
    """Client for robot_service position API"""
    
    def __init__(self, base_url: str = "http://localhost:8110", timeout: float = 10.0):
        """
        Initialize the client
        
        Args:
            base_url: Base URL of the robot_service API
            timeout: Request timeout in seconds
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.client = httpx.AsyncClient(timeout=timeout)
    
    async def close(self):
        """Close the HTTP client"""
        await self.client.aclose()
    
    async def list_positions(self) -> List[Dict[str, Any]]:
        """
        Get list of all positions
        
        Returns:
            List of position dictionaries with keys: position_id, x, y, theta, description
        """
        try:
            response = await self.client.get(
                f"{self.base_url}/robot_positions/list"
            )
            response.raise_for_status()
            
            # API returns a string, need to parse it
            # Based on the API docs, it seems to return JSON as string
            data = response.json()
            
            # Handle different response formats
            if isinstance(data, str):
                import json
                data = json.loads(data)
            
            # If data is a list, normalize and return it
            if isinstance(data, list):
                return [self._normalize_position(pos) for pos in data]
            
            # If data is a dict with positions key
            if isinstance(data, dict) and 'positions' in data:
                positions = data['positions']
                if isinstance(positions, list):
                    return [self._normalize_position(pos) for pos in positions]
                return []
            
            # If data is a dict with other structure, try to extract positions
            if isinstance(data, dict):
                # Try common keys
                for key in ['data', 'items', 'results']:
                    if key in data and isinstance(data[key], list):
                        return [self._normalize_position(pos) for pos in data[key]]
            
            # Fallback: return empty list
            return []
            
        except httpx.HTTPStatusError as e:
            raise Exception(f"HTTP error getting positions: {e.response.status_code} - {e.response.text}")
        except Exception as e:
            raise Exception(f"Error getting positions: {str(e)}")
    
    def _normalize_position(self, pos: Dict[str, Any]) -> Dict[str, Any]:
        """
        Normalize position data from API format to our expected format.
        
        API format:
        {
            "id": "...",
            "name": "...",
            "params": {
                "location": {"x_m": ..., "y_m": ..., "theta_deg": ..., "map_id": ...},
                "position_id": "...",
                "x": ...,
                "y": ...,
                "theta": ...,
                "description": "..."
            }
        }
        
        Our format:
        {
            "position_id": "...",
            "x": ...,
            "y": ...,
            "theta": ...,
            "description": "..."
        }
        """
        normalized = {}
        
        # Try to get position_id from params or use name/id
        params = pos.get('params', {})
        normalized['position_id'] = (
            params.get('position_id') or 
            pos.get('name') or 
            pos.get('id') or 
            str(pos.get('id', ''))
        )
        
        # Get coordinates - prefer our format (x, y, theta), fallback to location (x_m, y_m, theta_deg)
        if 'x' in params and 'y' in params and 'theta' in params:
            normalized['x'] = float(params['x'])
            normalized['y'] = float(params['y'])
            normalized['theta'] = float(params['theta'])
        elif 'location' in params:
            location = params['location']
            # Convert from meters and degrees to our format (meters and radians)
            import math
            normalized['x'] = float(location.get('x_m', 0.0))
            normalized['y'] = float(location.get('y_m', 0.0))
            theta_deg = float(location.get('theta_deg', 0.0))
            normalized['theta'] = math.radians(theta_deg)
        else:
            normalized['x'] = 0.0
            normalized['y'] = 0.0
            normalized['theta'] = 0.0
        
        # Get description
        normalized['description'] = str(params.get('description', '') or pos.get('name', '') or '')
        
        # Also preserve original data for reference
        normalized['_original'] = pos
        
        return normalized
    
    async def get_position(self, position_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific position by ID
        
        Args:
            position_id: Position identifier (can be position_id, name, or id)
            
        Returns:
            Position dictionary or None if not found
        """
        positions = await self.list_positions()
        for pos in positions:
            # Check multiple possible fields
            pos_id = pos.get('position_id') or pos.get('_original', {}).get('name') or pos.get('_original', {}).get('id')
            if str(pos_id) == str(position_id):
                return pos
        return None
    
    async def save_position(
        self,
        position_id: str,
        x: float,
        y: float,
        theta: float,
        description: str = ""
    ) -> bool:
        """
        Save or update a position
        
        Args:
            position_id: Position identifier
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
            description: Position description
            
        Returns:
            True if successful, False otherwise
        """
        try:
            payload = {
                "position_id": position_id,
                "x": x,
                "y": y,
                "theta": theta,
                "description": description
            }
            
            response = await self.client.post(
                f"{self.base_url}/robot_positions/save",
                json=payload
            )
            response.raise_for_status()
            return True
            
        except httpx.HTTPStatusError as e:
            if e.response.status_code == 422:
                error_detail = e.response.json() if e.response.headers.get('content-type', '').startswith('application/json') else {}
                raise ValueError(f"Validation error: {error_detail.get('detail', e.response.text)}")
            raise Exception(f"HTTP error saving position: {e.response.status_code} - {e.response.text}")
        except Exception as e:
            raise Exception(f"Error saving position: {str(e)}")
    
    async def delete_position(self, position_id: str) -> bool:
        """
        Delete a position
        
        Args:
            position_id: Position identifier
            
        Returns:
            True if successful, False otherwise
        """
        try:
            response = await self.client.post(
                f"{self.base_url}/robot_positions/delete",
                params={"position_id": position_id}
            )
            response.raise_for_status()
            return True
            
        except httpx.HTTPStatusError as e:
            if e.response.status_code == 404:
                raise ValueError(f"Position '{position_id}' not found")
            raise Exception(f"HTTP error deleting position: {e.response.status_code} - {e.response.text}")
        except Exception as e:
            raise Exception(f"Error deleting position: {str(e)}")
    
    async def run_position(self, position_id: str) -> Dict[str, Any]:
        """
        Run navigation to a position
        
        Args:
            position_id: Position identifier
            
        Returns:
            Response dictionary with task_id, success, result, etc.
        """
        try:
            response = await self.client.post(
                f"{self.base_url}/robot_positions/run",
                params={"position_id": position_id}
            )
            response.raise_for_status()
            
            data = response.json()
            if isinstance(data, str):
                import json
                data = json.loads(data)
            
            return data
            
        except httpx.HTTPStatusError as e:
            if e.response.status_code == 422:
                error_detail = e.response.json() if e.response.headers.get('content-type', '').startswith('application/json') else {}
                raise ValueError(f"Validation error: {error_detail.get('detail', e.response.text)}")
            raise Exception(f"HTTP error running position: {e.response.status_code} - {e.response.text}")
        except Exception as e:
            raise Exception(f"Error running position: {str(e)}")
    
    def has_position(self, position_id: str) -> bool:
        """
        Check if position exists (synchronous wrapper for async method)
        
        Note: This is a blocking call. For async code, use get_position() instead.
        """
        import asyncio
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # If loop is running, we can't use it - return False and log warning
                import logging
                logging.warning("has_position called from async context - use get_position() instead")
                return False
            position = loop.run_until_complete(self.get_position(position_id))
            return position is not None
        except RuntimeError:
            # No event loop - create one
            position = asyncio.run(self.get_position(position_id))
            return position is not None

