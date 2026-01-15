#!/usr/bin/env python3

# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Position Registry (Python wrapper)

Loads positions from YAML and provides access to them.
"""

import yaml
import os
import fcntl
import shutil
import tempfile
import threading
from geometry_msgs.msg import PoseStamped
import math


class PositionRegistry:
    def __init__(self, allowed_base_dir=None):
        """
        Initialize PositionRegistry.
        
        Args:
            allowed_base_dir: Base directory for path validation (prevents path traversal).
                            If None, uses package config directory.
        """
        self.positions = {}
        self.map_frame_id = "map"
        self._lock = threading.Lock()  # Lock for thread-safe access to positions dict
        self._file_lock = threading.Lock()  # Lock for file operations
        self.allowed_base_dir = allowed_base_dir
    
    def _validate_path(self, yaml_path: str) -> str:
        """
        Validate and normalize path, preventing path traversal attacks.
        
        Args:
            yaml_path: Path to validate
            
        Returns:
            Normalized absolute path
            
        Raises:
            ValueError: If path traversal is detected
        """
        # Normalize path
        yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
        
        # If allowed_base_dir is set, validate path is within it
        if self.allowed_base_dir:
            allowed_base = os.path.abspath(self.allowed_base_dir)
            if not yaml_path.startswith(allowed_base):
                raise ValueError(f"Path traversal detected: {yaml_path} is outside allowed directory {allowed_base}")
        
        return yaml_path
    
    def loadFromYAML(self, yaml_path: str) -> bool:
        """Load positions from YAML file with file locking"""
        with self._file_lock:
            try:
                # If absolute path is provided, use it directly
                if not os.path.isabs(yaml_path):
                    # Resolve relative path
                    from ament_index_python.packages import get_package_share_directory
                    try:
                        pkg_dir = get_package_share_directory('aehub_navigation')
                        yaml_path = os.path.join(pkg_dir, 'config', 'positions.yaml')
                    except Exception:
                        # Fallback to workspace config
                        yaml_path = os.path.join(
                            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                            '..', '..', '..', 'config', 'positions.yaml'
                        )
                
                # Validate and normalize path
                yaml_path = self._validate_path(yaml_path)
                
                if not os.path.exists(yaml_path):
                    print(f"Error: Positions file not found: {yaml_path}")
                    return False
                
                # Open file with exclusive lock (prevents concurrent reads during write)
                with open(yaml_path, 'r') as f:
                    try:
                        # Acquire shared lock (allows concurrent reads, blocks writes)
                        fcntl.flock(f.fileno(), fcntl.LOCK_SH)
                        config = yaml.safe_load(f)
                    finally:
                        # Release lock
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
            
                if not config or 'positions' not in config:
                    print(f"Error: Invalid YAML structure in {yaml_path}")
                    return False
                
                # Update positions dict with lock
                with self._lock:
                    self.positions.clear()
                    
                    for position_id, pos_data in config['positions'].items():
                        if not isinstance(pos_data, dict):
                            print(f"Warning: Invalid position data for {position_id}, skipping")
                            continue
                        
                        try:
                            self.positions[position_id] = {
                                'x': float(pos_data['x']),
                                'y': float(pos_data['y']),
                                'theta': float(pos_data['theta']),
                                'description': str(pos_data.get('description', ''))
                            }
                        except (KeyError, ValueError, TypeError) as e:
                            print(f"Warning: Invalid position data for {position_id}: {e}, skipping")
                            continue
                    
                    # Allow any number of positions (dynamic positions)
                    if len(self.positions) == 0:
                        print(f"Warning: No valid positions loaded from {yaml_path}")
                        return False
                    
                    print(f"Successfully loaded {len(self.positions)} positions from {yaml_path}")
                    return True
            except ValueError as e:
                # Path validation error
                print(f"Error: {e}")
                return False
            except Exception as e:
                print(f"Error loading positions from {yaml_path}: {e}")
                import traceback
                traceback.print_exc()
                return False
    
    def getPosition(self, position_id: str) -> PoseStamped:
        """Get position by ID (thread-safe)"""
        with self._lock:
            if position_id not in self.positions:
                return None
            
            pos = self.positions[position_id].copy()  # Copy to avoid race conditions
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame_id
        
        pose.pose.position.x = pos['x']
        pose.pose.position.y = pos['y']
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta = pos['theta']
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose
    
    def hasPosition(self, position_id: str) -> bool:
        """Check if position exists (thread-safe)"""
        with self._lock:
            return position_id in self.positions
    
    def getAllPositionIds(self):
        """Get all position IDs (thread-safe)"""
        with self._lock:
            return list(self.positions.keys())
    
    def getPositionCount(self) -> int:
        """Get number of positions (thread-safe)"""
        with self._lock:
            return len(self.positions)
    
    def saveToYAML(self, yaml_path: str) -> bool:
        """Save positions to YAML file with atomic write and file locking"""
        with self._file_lock:
            try:
                # Validate and normalize path
                yaml_path = self._validate_path(yaml_path)
                
                # Create directory if it doesn't exist
                os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                
                # Copy positions dict with lock to avoid race conditions
                with self._lock:
                    positions_copy = self.positions.copy()
                    position_count = len(positions_copy)
                
                config = {
                    'positions': {}
                }
                
                for position_id, pos_data in positions_copy.items():
                    config['positions'][position_id] = {
                        'x': pos_data['x'],
                        'y': pos_data['y'],
                        'theta': pos_data['theta']
                    }
                    if pos_data.get('description'):
                        config['positions'][position_id]['description'] = pos_data['description']
                
                # Atomic write: write to temp file, then rename
                # This ensures file is never in corrupted state
                temp_fd, temp_path = tempfile.mkstemp(
                    suffix='.yaml',
                    dir=os.path.dirname(yaml_path),
                    text=True
                )
                
                try:
                    # Write to temp file with exclusive lock
                    with os.fdopen(temp_fd, 'w') as f:
                        fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                        yaml.dump(config, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
                        f.flush()
                        os.fsync(f.fileno())  # Ensure data is written to disk
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                    
                    # Atomic rename (works on most filesystems)
                    shutil.move(temp_path, yaml_path)
                    
                    print(f"Successfully saved {position_count} positions to {yaml_path}")
                    return True
                except Exception as e:
                    # Clean up temp file on error
                    try:
                        if os.path.exists(temp_path):
                            os.unlink(temp_path)
                    except Exception:
                        pass
                    raise e
            except ValueError as e:
                # Path validation error
                print(f"Error: {e}")
                return False
            except Exception as e:
                print(f"Error saving positions to {yaml_path}: {e}")
                import traceback
                traceback.print_exc()
                return False
    
    def addPosition(self, position_id: str, x: float, y: float, theta: float, description: str = "") -> bool:
        """Add or update a position (thread-safe)"""
        if not position_id or not isinstance(position_id, str):
            print(f"Error: Invalid position_id: {position_id}")
            return False
        
        try:
            with self._lock:
                self.positions[position_id] = {
                    'x': float(x),
                    'y': float(y),
                    'theta': float(theta),
                    'description': str(description) if description else ''
                }
            print(f"Position '{position_id}' added/updated: x={x}, y={y}, theta={theta}")
            return True
        except (ValueError, TypeError) as e:
            print(f"Error adding position '{position_id}': {e}")
            return False
    
    def removePosition(self, position_id: str) -> bool:
        """Remove a position (thread-safe)"""
        with self._lock:
            if position_id not in self.positions:
                print(f"Warning: Position '{position_id}' not found for removal")
                return False
            
            del self.positions[position_id]
        print(f"Position '{position_id}' removed successfully")
        return True

