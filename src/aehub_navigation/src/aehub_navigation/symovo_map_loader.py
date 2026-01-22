"""
Symovo Map Loader

Fetches map from Symovo API and converts it to Nav2 format (PGM + YAML).
"""

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

import json
import logging
import os
import requests
from typing import Optional, Dict, Any
from PIL import Image
import numpy as np

logger = logging.getLogger(__name__)

# region agent log
def _agent_log(*, run_id: str, hypothesis_id: str, location: str, message: str, data: dict):
    try:
        import json as _json
        import time as _time

        os.makedirs("/home/boris/ros2_ws/.cursor", exist_ok=True)
        with open("/home/boris/ros2_ws/.cursor/debug.log", "a", encoding="utf-8") as f:
            f.write(
                _json.dumps(
                    {
                        "sessionId": "debug-session",
                        "runId": run_id,
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "timestamp": int(_time.time() * 1000),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
    except Exception:
        pass


# endregion agent log


class SymovoMapLoader:
    """Loads map from Symovo API and converts to Nav2 format."""
    
    def __init__(self, endpoint: str, amr_id: int, tls_verify: bool = False):
        """
        Initialize Symovo Map Loader.
        
        Args:
            endpoint: Symovo API endpoint (e.g., "https://192.168.1.100")
            amr_id: AMR ID to get map for
            tls_verify: Whether to verify TLS certificates
        """
        self.endpoint = endpoint.rstrip('/')
        self.amr_id = amr_id
        self.tls_verify = tls_verify
        self.map_data: Optional[Dict[str, Any]] = None
        
    def fetch_map_info(self, *, map_id: Optional[int] = None) -> Optional[Dict[str, Any]]:
        """
        Fetch map information from Symovo API.
        
        Returns:
            Map information dict or None on error
        """
        try:
            url = f"{self.endpoint}/v0/map"
            logger.info(f"Fetching map info from {url}")
            # region agent log
            _agent_log(
                run_id="pre-fix",
                hypothesis_id="H6",
                location="symovo_map_loader.py:fetch_map_info",
                message="GET /v0/map",
                data={"url": url, "map_id_override": map_id, "tls_verify": self.tls_verify},
            )
            # endregion agent log
            
            response = requests.get(url, verify=self.tls_verify, timeout=10)
            response.raise_for_status()
            
            maps = response.json()
            if not isinstance(maps, list) or len(maps) == 0:
                logger.error("No maps found in API response")
                return None
            
            if map_id is not None:
                map_info = next((m for m in maps if m.get("id") == map_id), None)
                if not map_info:
                    logger.error(f"Map id={map_id} not found in /v0/map list")
                    # region agent log
                    _agent_log(
                        run_id="pre-fix",
                        hypothesis_id="H6",
                        location="symovo_map_loader.py:fetch_map_info",
                        message="Requested map_id not found in /v0/map list",
                        data={"map_id": map_id, "map_ids": [m.get("id") for m in maps[:10]]},
                    )
                    # endregion agent log
                    return None
            else:
                # Fallback: first map
                map_info = maps[0]
            logger.info(f"Found map: id={map_info.get('id')}, name={map_info.get('name')}")
            # region agent log
            _agent_log(
                run_id="pre-fix",
                hypothesis_id="H6",
                location="symovo_map_loader.py:fetch_map_info",
                message="Selected map",
                data={
                    "selected_id": map_info.get("id"),
                    "name": map_info.get("name"),
                    "offsetX": map_info.get("offsetX"),
                    "offsetY": map_info.get("offsetY"),
                    "resolution": map_info.get("resolution"),
                },
            )
            # endregion agent log
            
            self.map_data = map_info
            return map_info
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to fetch map info: {e}")
            return None
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse map info JSON: {e}")
            return None
    
    def fetch_map_image(self) -> Optional[np.ndarray]:
        """
        Fetch map image from Symovo API endpoint /v0/map/{id}/full.png.
        
        Returns:
            Image as numpy array (grayscale) or None on error
        """
        if not self.map_data:
            logger.error("Map data not loaded")
            return None
        
        try:
            map_id = self.map_data.get('id')
            if not map_id:
                logger.error("No map ID in map data")
                return None
            
            # Fetch PNG image from API
            url = f"{self.endpoint}/v0/map/{map_id}/full.png"
            logger.info(f"Fetching map image from {url}")
            # region agent log
            _agent_log(
                run_id="pre-fix",
                hypothesis_id="H7",
                location="symovo_map_loader.py:fetch_map_image",
                message="GET /v0/map/{id}/full.png",
                data={"url": url, "map_id": map_id, "tls_verify": self.tls_verify},
            )
            # endregion agent log
            
            response = requests.get(url, verify=self.tls_verify, timeout=30)
            response.raise_for_status()
            # region agent log
            _agent_log(
                run_id="pre-fix",
                hypothesis_id="H7",
                location="symovo_map_loader.py:fetch_map_image",
                message="Received map image bytes",
                data={"status_code": response.status_code, "bytes": len(response.content)},
            )
            # endregion agent log
            
            # Load image from response bytes
            from io import BytesIO
            image = Image.open(BytesIO(response.content))
            
            # Convert to grayscale if needed
            if image.mode != 'L':
                image = image.convert('L')
            
            # Convert to numpy array
            img_array = np.array(image)
            logger.info(f"Fetched image: shape={img_array.shape}, dtype={img_array.dtype}")
            
            return img_array
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to fetch map image: {e}")
            return None
        except Exception as e:
            logger.error(f"Failed to process map image: {e}")
            return None
    
    def convert_to_pgm(self, img_array: np.ndarray, output_path: str) -> bool:
        """
        Convert image array to PGM format.
        
        Args:
            img_array: Image as numpy array (grayscale, 0-255)
            output_path: Path to save PGM file
            
        Returns:
            True on success, False on error
        """
        try:
            height, width = img_array.shape
            
            # PGM format: 0=black/occupied, 255=white/free
            # Symovo API provides: white=free (255), black=occupied (0)
            # Nav2 expects: white=free, black=occupied in PGM
            # So NO inversion needed - Symovo format matches Nav2 format
            # Just use the image array as-is
            
            pgm_array = img_array
            
            # Write PGM file
            with open(output_path, 'wb') as f:
                # PGM header
                f.write(f"P5\n{width} {height}\n255\n".encode())
                # Image data
                f.write(pgm_array.tobytes())
            
            logger.info(f"Saved PGM file: {output_path} ({width}x{height})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to convert to PGM: {e}")
            return False
    
    def create_yaml(self, pgm_path: str, output_path: str, *, write_absolute_image_path: bool) -> bool:
        """
        Create YAML file for Nav2 map_server.
        
        Args:
            pgm_path: Path to PGM file
            output_path: Path to save YAML file
            
        Returns:
            True on success, False on error
        """
        if not self.map_data:
            logger.error("Map data not loaded")
            return False
        
        try:
            # Get map metadata
            resolution = self.map_data.get('resolution', 0.05)
            offset_x = self.map_data.get('offsetX', 0.0)
            offset_y = self.map_data.get('offsetY', 0.0)
            size = self.map_data.get('size', [100, 100])
            
            # Prefer relative image path for portability.
            image_path = os.path.abspath(pgm_path) if write_absolute_image_path else os.path.basename(pgm_path)
            
            # Create YAML content
            yaml_content = f"""image: {image_path}
resolution: {resolution}
origin: [{offset_x}, {offset_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
            
            with open(output_path, 'w') as f:
                f.write(yaml_content)
            
            logger.info(f"Saved YAML file: {output_path}")
            logger.info(f"  resolution: {resolution}")
            logger.info(f"  origin: [{offset_x}, {offset_y}, 0.0]")
            logger.info(f"  size: {size}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to create YAML: {e}")
            return False
    
    def _select_map_id_from_robot_pose(self) -> Optional[int]:
        """
        Try to select active map id from robot pose.
        Prefer /v0/amr (documented), fallback to /v0/agv (observed on some deployments).
        Expected schema: item.pose.map_id (best-effort; returns None if not available).
        """
        for path in ("/v0/amr", "/v0/agv"):
            try:
                url = f"{self.endpoint}{path}"
                # region agent log
                _agent_log(
                    run_id="pre-fix",
                    hypothesis_id="H8",
                    location="symovo_map_loader.py:_select_map_id_from_robot_pose",
                    message="GET robot list to infer pose.map_id",
                    data={"url": url, "amr_id": self.amr_id, "tls_verify": self.tls_verify},
                )
                # endregion agent log
                response = requests.get(url, verify=self.tls_verify, timeout=10)
                response.raise_for_status()
                arr = response.json()
                if not isinstance(arr, list):
                    continue
                for item in arr:
                    if item.get("id") == self.amr_id:
                        pose = item.get("pose") or {}
                        map_id = pose.get("map_id")
                        if map_id is None:
                            # region agent log
                            _agent_log(
                                run_id="pre-fix",
                                hypothesis_id="H8",
                                location="symovo_map_loader.py:_select_map_id_from_robot_pose",
                                message="pose.map_id missing for amr_id",
                                data={"path": path, "pose_keys": list(pose.keys())[:20]},
                            )
                            # endregion agent log
                            return None
                        # region agent log
                        _agent_log(
                            run_id="pre-fix",
                            hypothesis_id="H8",
                            location="symovo_map_loader.py:_select_map_id_from_robot_pose",
                            message="Selected map_id from robot pose",
                            data={"path": path, "map_id": int(map_id)},
                        )
                        # endregion agent log
                        return int(map_id)
            except Exception:
                continue
        return None

    def load_map(
        self,
        output_dir: str,
        *,
        map_id: Optional[int] = None,
        select_map_mode: str = "robot_pose",
        write_absolute_image_path: bool = False,
    ) -> Optional[str]:
        """
        Load map from Symovo API and save to Nav2 format.
        
        Args:
            output_dir: Directory to save map files
            
        Returns:
            Path to YAML file or None on error
        """
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        selected_map_id: Optional[int] = map_id
        if selected_map_id is None and select_map_mode == "robot_pose":
            selected_map_id = self._select_map_id_from_robot_pose()
            if selected_map_id is not None:
                logger.info(f"Selected map_id from robot pose: {selected_map_id}")
            else:
                logger.warning("Failed to detect map_id from robot pose; falling back to first map")
        # region agent log
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H9",
            location="symovo_map_loader.py:load_map",
            message="load_map selected map id",
            data={"selected_map_id": selected_map_id, "select_map_mode": select_map_mode},
        )
        # endregion agent log

        # Fetch map info
        if not self.fetch_map_info(map_id=selected_map_id):
            return None
        
        # Fetch image from API
        img_array = self.fetch_map_image()
        if img_array is None:
            return None
        
        # Save PGM
        pgm_path = os.path.join(output_dir, 'map.pgm')
        if not self.convert_to_pgm(img_array, pgm_path):
            return None
        
        # Save YAML
        yaml_path = os.path.join(output_dir, 'map.yaml')
        if not self.create_yaml(pgm_path, yaml_path, write_absolute_image_path=write_absolute_image_path):
            return None
        
        logger.info(f"Map loaded successfully: {yaml_path}")
        return yaml_path


def main():
    """Test function."""
    import sys
    
    if len(sys.argv) < 4:
        print("Usage: symovo_map_loader.py <endpoint> <amr_id> <output_dir>")
        sys.exit(1)
    
    endpoint = sys.argv[1]
    amr_id = int(sys.argv[2])
    output_dir = sys.argv[3]
    
    loader = SymovoMapLoader(endpoint, amr_id, tls_verify=False)
    yaml_path = loader.load_map(output_dir)
    
    if yaml_path:
        print(f"✅ Map loaded: {yaml_path}")
        sys.exit(0)
    else:
        print("❌ Failed to load map")
        sys.exit(1)


if __name__ == '__main__':
    main()
