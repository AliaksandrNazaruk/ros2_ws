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

"""aehub_navigation.mqtt_status_publisher

Publishes navigation status to:
  aroc/robot/{ROBOT_ID}/status/navigation

The MQTT client guide (mqtt.txt) defines status values:
  - idle
  - navigating
  - arrived
  - error

The internal state machine has more granular states (canceling/aborted/...).
This publisher maps them to the externally documented set.

NOTE: This class does not manage its own MQTT connection. It uses the shared
MQTTConnectionManager provided by NavigationIntegratedNode.
"""

from __future__ import annotations

import json
import traceback
from datetime import datetime
from typing import Optional, Dict, Any

from aehub_navigation.navigation_state_manager import NavigationState
from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager


class MQTTStatusPublisher:
    def __init__(self):
        self.mqtt_manager: Optional[MQTTConnectionManager] = None
        self.robot_id: Optional[str] = None
        self.status_topic: Optional[str] = None

        # Mutable status snapshot (updated by NavigationIntegratedNode)
        self._snapshot: Dict[str, Any] = {
            "state": NavigationState.IDLE,
            "active_target_id": None,
            "active_command_id": None,
            "progress_percent": 0,
            "eta_seconds": 0,
            "error_code": None,
            "error_message": None,
            "current_position": {"x": 0.0, "y": 0.0, "theta": 0.0},
        }

    def set_mqtt_manager(self, mqtt_manager: MQTTConnectionManager) -> None:
        self.mqtt_manager = mqtt_manager

    def set_robot_id(self, robot_id: str) -> None:
        self.robot_id = robot_id
        self.status_topic = f"aroc/robot/{robot_id}/status/navigation"

    def updateStatus(
        self,
        state: NavigationState,
        target_id: Optional[str] = None,
        progress_percent: int = 0,
        eta_seconds: int = 0,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
        current_position: Optional[dict] = None,
        command_id: Optional[str] = None,
    ) -> None:
        self._snapshot["state"] = state
        # Canonical v2.0 naming
        self._snapshot["active_target_id"] = target_id
        self._snapshot["active_command_id"] = command_id
        self._snapshot["progress_percent"] = int(progress_percent or 0)
        self._snapshot["eta_seconds"] = int(eta_seconds or 0)
        self._snapshot["error_code"] = error_code
        self._snapshot["error_message"] = error_message
        if current_position is not None:
            self._snapshot["current_position"] = dict(current_position)

    def publishStatus(self) -> None:
        if not self.mqtt_manager or not self.status_topic or not self.robot_id:
            return

        state: NavigationState = self._snapshot.get("state") or NavigationState.IDLE
        status_value = self._map_state_to_public_status(state)

        payload: Dict[str, Any] = {
            # Keep schema_version for forward-compatibility; clients may ignore.
            "schema_version": "2.0",
            "robot_id": self.robot_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "status": status_value,
            "active_target_id": self._snapshot.get("active_target_id"),
            "active_command_id": self._snapshot.get("active_command_id"),
            # Optional extras (clients should treat as optional)
            "progress_percent": self._snapshot.get("progress_percent", 0),
            "eta_seconds": self._snapshot.get("eta_seconds", 0),
            "current_position": {
                "x": float((self._snapshot.get("current_position") or {}).get("x", 0.0)),
                "y": float((self._snapshot.get("current_position") or {}).get("y", 0.0)),
                "theta": float((self._snapshot.get("current_position") or {}).get("theta", 0.0)),
            },
            "error_code": self._snapshot.get("error_code"),
            "error_message": self._snapshot.get("error_message"),
        }

        try:
            self.mqtt_manager.publish(self.status_topic, json.dumps(payload), qos=1)
        except Exception:
            # Never throw from status publication.
            if getattr(self.mqtt_manager, "logger", None):
                self.mqtt_manager.logger.error(
                    f"Failed to publish navigation status to {self.status_topic}:\n{traceback.format_exc()}"
                )

    @staticmethod
    def _map_state_to_public_status(state: NavigationState) -> str:
        """Map internal state to externally documented status values."""
        if state in (NavigationState.NAVIGATING,):
            return "navigating"
        if state in (NavigationState.SUCCEEDED,):
            return "arrived"
        if state in (NavigationState.ERROR, NavigationState.ABORTED):
            return "error"
        # PAUSED/CANCELING/IDLE => idle (public spec has no paused/canceling)
        return "idle"
