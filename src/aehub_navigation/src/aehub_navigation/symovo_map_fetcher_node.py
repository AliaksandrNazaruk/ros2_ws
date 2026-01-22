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
One-shot Symovo map fetcher.

Best-practice goal:
- keep Nav2 standard `nav2_map_server` as the publisher of `/map`
- fetch/convert map into a standard `map.yaml` + `map.pgm` on disk BEFORE starting map_server

This node does exactly one fetch->convert->write and then exits with:
- exit code 0: success
- exit code 2: transient/IO error (fetch failed or write failed)

It is intended to be used from launch via OnProcessExit to gate map_server startup.
"""

import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from aehub_navigation.symovo_map_loader import SymovoMapLoader


# region agent log
def _agent_log(*, run_id: str, hypothesis_id: str, location: str, message: str, data: dict):
    try:
        import json
        import time

        os.makedirs("/home/boris/ros2_ws/.cursor", exist_ok=True)
        with open("/home/boris/ros2_ws/.cursor/debug.log", "a", encoding="utf-8") as f:
            f.write(
                json.dumps(
                    {
                        "sessionId": "debug-session",
                        "runId": run_id,
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "timestamp": int(time.time() * 1000),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
    except Exception:
        pass


# endregion agent log


class SymovoMapFetcherNode(Node):
    def __init__(self) -> None:
        super().__init__("symovo_map_fetcher")

        self.declare_parameter("symovo_endpoint", "https://192.168.1.100")
        self.declare_parameter("amr_id", 15)
        self.declare_parameter("tls_verify", False)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("map_id", 0)  # optional override (0 means auto)
        self.declare_parameter("select_map_mode", "robot_pose")  # robot_pose | first
        self.declare_parameter("write_absolute_image_path", False)

        self.endpoint = str(self.get_parameter("symovo_endpoint").value).rstrip("/")
        self.amr_id = int(self.get_parameter("amr_id").value)
        self.tls_verify = bool(self.get_parameter("tls_verify").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.map_id_override = int(self.get_parameter("map_id").value)
        self.select_map_mode = str(self.get_parameter("select_map_mode").value)
        self.write_absolute_image_path = bool(self.get_parameter("write_absolute_image_path").value)

        if not self.output_dir:
            raise RuntimeError("output_dir parameter is required")

        # region agent log
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H4",
            location="symovo_map_fetcher_node.py:__init__",
            message="Initialized map fetcher node (parameters)",
            data={
                "endpoint": self.endpoint,
                "amr_id": self.amr_id,
                "tls_verify": self.tls_verify,
                "output_dir": self.output_dir,
                "map_id_override": self.map_id_override,
                "select_map_mode": self.select_map_mode,
                "write_absolute_image_path": self.write_absolute_image_path,
            },
        )
        # endregion agent log

        self.get_logger().info(
            f"symovo_map_fetcher: endpoint={self.endpoint}, amr_id={self.amr_id}, "
            f"tls_verify={self.tls_verify}, output_dir={self.output_dir}"
        )

    def run_once(self) -> bool:
        os.makedirs(self.output_dir, exist_ok=True)

        loader = SymovoMapLoader(self.endpoint, self.amr_id, tls_verify=self.tls_verify)
        # region agent log
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H5",
            location="symovo_map_fetcher_node.py:run_once",
            message="Calling SymovoMapLoader.load_map",
            data={"output_dir": self.output_dir, "select_map_mode": self.select_map_mode},
        )
        # endregion agent log
        yaml_path: Optional[str] = loader.load_map(
            self.output_dir,
            map_id=self.map_id_override if self.map_id_override > 0 else None,
            select_map_mode=self.select_map_mode,
            write_absolute_image_path=self.write_absolute_image_path,
        )
        if not yaml_path:
            # region agent log
            _agent_log(
                run_id="pre-fix",
                hypothesis_id="H5",
                location="symovo_map_fetcher_node.py:run_once",
                message="SymovoMapLoader.load_map returned empty",
                data={},
            )
            # endregion agent log
            return False

        # region agent log
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H5",
            location="symovo_map_fetcher_node.py:run_once",
            message="Map ready",
            data={"yaml_path": yaml_path},
        )
        # endregion agent log
        self.get_logger().info(f"Map ready: {yaml_path}")
        return True


def main() -> int:
    rclpy.init()
    try:
        node = SymovoMapFetcherNode()
        ok = node.run_once()
        node.destroy_node()
        return 0 if ok else 2
    except KeyboardInterrupt:
        return 130
    except Exception as e:
        # Keep error visible in ros2 launch output
        print(f"[symovo_map_fetcher] ERROR: {e}", file=sys.stderr)
        return 2
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

