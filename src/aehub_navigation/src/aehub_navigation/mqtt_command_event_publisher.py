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
MQTT Command Event Publisher

Publishes per-command events to:
- aroc/robot/{ROBOT_ID}/commands/events

Event types:
- ack: received | accepted | rejected
- result: succeeded | aborted | canceled | error
"""

import json
from datetime import datetime
from typing import Optional, Dict, Any

from aehub_navigation.mqtt_connection_manager import MQTTConnectionManager


class MQTTCommandEventPublisher:
    def __init__(self):
        self.mqtt_manager: Optional[MQTTConnectionManager] = None
        self.robot_id: Optional[str] = None
        self.events_topic: Optional[str] = None

    def set_mqtt_manager(self, mqtt_manager: MQTTConnectionManager):
        self.mqtt_manager = mqtt_manager

    def set_robot_id(self, robot_id: str):
        self.robot_id = robot_id
        self.events_topic = f'aroc/robot/{robot_id}/commands/events'

    def _publish_event(self, event: Dict[str, Any]) -> bool:
        if not self.mqtt_manager or not self.events_topic or not self.robot_id:
            return False
        try:
            event['timestamp'] = datetime.utcnow().isoformat() + 'Z'
            return self.mqtt_manager.publish(self.events_topic, json.dumps(event), qos=1)
        except Exception:
            return False

    def publish_ack(self, command_id: str, target_id: Optional[str], ack_status: str, reason: Optional[str] = None):
        """
        ack_status: received | accepted | rejected
        """
        payload = {
            'schema_version': '2.0',
            'robot_id': self.robot_id,
            'event_type': 'ack',
            'ack_status': ack_status,
            'command_id': command_id,
            'target_id': target_id,
        }
        if reason:
            payload['reason'] = reason
        self._publish_event(payload)

    def publish_result(self, command_id: str, target_id: Optional[str], result_status: str,
                       error_code: Optional[str] = None, error_message: Optional[str] = None):
        """
        result_status: succeeded | aborted | canceled | error
        """
        payload = {
            'schema_version': '2.0',
            'robot_id': self.robot_id,
            'event_type': 'result',
            'result_status': result_status,
            'command_id': command_id,
            'target_id': target_id,
            'error_code': error_code,
            'error_message': error_message,
        }
        self._publish_event(payload)


