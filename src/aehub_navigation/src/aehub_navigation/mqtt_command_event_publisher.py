#!/usr/bin/env python3

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

    def publish_ack(self, command_id: str, target_id: Optional[str], ack_type: str, reason: Optional[str] = None):
        """
        ack_type: received | accepted | rejected
        """
        payload = {
            'schema_version': '1.0',
            'robot_id': self.robot_id,
            'event_type': 'ack',
            'ack_type': ack_type,
            'command_id': command_id,
            'target_id': target_id,
        }
        if reason:
            payload['reason'] = reason
        self._publish_event(payload)

    def publish_result(self, command_id: str, target_id: Optional[str], result_type: str,
                       error_code: Optional[str] = None, error_message: Optional[str] = None):
        """
        result_type: succeeded | aborted | canceled | error
        """
        payload = {
            'schema_version': '1.0',
            'robot_id': self.robot_id,
            'event_type': 'result',
            'result_type': result_type,
            'command_id': command_id,
            'target_id': target_id,
            'error_code': error_code,
            'error_message': error_message,
        }
        self._publish_event(payload)


