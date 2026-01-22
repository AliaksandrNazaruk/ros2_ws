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

"""aehub_navigation.command_validator

This module validates incoming MQTT navigation commands.

Canonical v2.0 note
-------------------
`schema_version` and `robot_id` are required and must match the expected major
version and configured robot_id.
- For navigateTo: requires either `target_id` (must exist in PositionRegistry)
  OR a direct pose: `x` and `y` (and optional `theta`).
- For cancel: requires only `command_id` and `timestamp`.

Deduplication
-------------
Validation should *not* permanently mark a command as processed. The caller
(NavigationIntegratedNode) decides when a command becomes terminal.
This class provides a lightweight TTL-based processed-id cache for idempotency.
"""

from __future__ import annotations

import re
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple


_UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[1-5][0-9a-fA-F]{3}-[89abAB][0-9a-fA-F]{3}-[0-9a-fA-F]{12}$"
)


class CommandValidator:
    """Validate and deduplicate commands."""

    def __init__(self, max_processed_ids: int = 1000, ttl_seconds: int = 3600):
        self._max_processed_ids = int(max_processed_ids)
        self._ttl_seconds = int(ttl_seconds)

        # command_id -> first_seen_epoch_s
        self._processed: Dict[str, float] = {}
        self._lock = threading.Lock()

    # ---------------------------------------------------------------------
    # Public: processed-id cache (idempotency)
    # ---------------------------------------------------------------------

    def is_duplicate(self, command_id: str, now_s: Optional[float] = None) -> bool:
        if not command_id:
            return False
        if now_s is None:
            now_s = time.time()
        self._prune(now_s)
        with self._lock:
            return command_id in self._processed

    def mark_processed(self, command_id: str, now_s: Optional[float] = None) -> None:
        if not command_id:
            return
        if now_s is None:
            now_s = time.time()
        self._prune(now_s)
        with self._lock:
            self._processed[command_id] = now_s
            # Cap size (remove oldest)
            if len(self._processed) > self._max_processed_ids:
                items = sorted(self._processed.items(), key=lambda kv: kv[1])
                for k, _ in items[: max(1, len(items) - self._max_processed_ids)]:
                    self._processed.pop(k, None)

    # ---------------------------------------------------------------------
    # Public: validation API (backward compatible)
    # ---------------------------------------------------------------------

    def validate_command(
        self,
        command: Dict[str, Any],
        position_registry: Any,
        expected_robot_id: Optional[str] = None,
        command_type: Optional[str] = None,
    ) -> Tuple[bool, Optional[str]]:
        """Backward compatible wrapper.

        Returns:
            (ok, error_message)
        """
        ok, _code, msg = self.validate_command_detailed(
            command=command,
            position_registry=position_registry,
            expected_robot_id=expected_robot_id,
            command_type=command_type,
        )
        return ok, msg

    def validate_command_detailed(
        self,
        command: Dict[str, Any],
        position_registry: Any,
        expected_robot_id: Optional[str] = None,
        command_type: Optional[str] = None,
    ) -> Tuple[bool, str, Optional[str]]:
        """Validate a command.

        Args:
            command: Parsed JSON payload (dict)
            position_registry: PositionRegistry
            expected_robot_id: robot_id derived from topic / node config
            command_type: 'navigateTo' | 'cancel' | None (auto)

        Returns:
            (ok, error_code, error_message)
        """
        if not isinstance(command, dict):
            return False, "NAV_INVALID_COMMAND", "payload must be a JSON object"

        # Detect command type if not supplied
        ct = (command_type or command.get("type") or "").strip()
        if not ct:
            # Heuristic: navigate if target_id or x/y exist
            if "target_id" in command or ("x" in command and "y" in command):
                ct = "navigateTo"
            else:
                ct = "cancel"  # safest default

        # Common fields
        ok, code, msg = self._validate_common(command, expected_robot_id=expected_robot_id)
        if not ok:
            return ok, code, msg

        if ct == "navigateTo":
            return self._validate_navigate(command, position_registry)
        if ct == "cancel":
            return self._validate_cancel(command)

        # Unknown command type
        return False, "NAV_INVALID_COMMAND", f"unsupported command type: {ct}"

    # ---------------------------------------------------------------------
    # Internal validation
    # ---------------------------------------------------------------------

    def _validate_common(
        self,
        command: Dict[str, Any],
        expected_robot_id: Optional[str],
        require_timestamp: bool = True,
    ) -> Tuple[bool, str, Optional[str]]:
        # Canonical schema version check (MAJOR must match).
        schema_version = command.get("schema_version")
        if not isinstance(schema_version, str) or not schema_version.strip():
            return False, "INVALID_SCHEMA", "missing or invalid schema_version"
        major, _minor = self._parse_schema_version(schema_version.strip())
        if major is None:
            return False, "INVALID_SCHEMA", "schema_version must be MAJOR.MINOR"
        if major != 2:
            return False, "INVALID_SCHEMA", f"unsupported schema_version major={major}"

        command_id = command.get("command_id")
        if not isinstance(command_id, str) or not command_id:
            return False, "NAV_INVALID_COMMAND", "missing or invalid command_id"
        if not _UUID_RE.match(command_id):
            return False, "NAV_INVALID_COMMAND", "command_id must be a UUID"

        # Canonical v2.0 requires robot_id and validates it against expected_robot_id.
        rid = command.get("robot_id")
        if not isinstance(rid, str) or not rid.strip():
            return False, "NAV_INVALID_COMMAND", "missing or invalid robot_id"
        if expected_robot_id is not None and rid.strip() != expected_robot_id:
            return False, "ROBOT_ID_MISMATCH", "robot_id does not match configured robot_id"

        if require_timestamp:
            ts = command.get("timestamp")
            if not isinstance(ts, str) or not ts:
                return False, "NAV_INVALID_COMMAND", "missing or invalid timestamp"
            if not self._is_iso8601(ts):
                return False, "NAV_INVALID_COMMAND", "timestamp must be ISO8601/RFC3339"

        return True, "ok", None

    def _validate_navigate(self, command: Dict[str, Any], position_registry: Any) -> Tuple[bool, str, Optional[str]]:
        # Priority: optional in mqtt.txt, default to normal
        priority = command.get("priority", "normal")
        if priority is None:
            priority = "normal"
        if not isinstance(priority, str):
            return False, "NAV_INVALID_COMMAND", "priority must be a string"
        priority = priority.strip().lower()
        if priority not in {"normal", "high", "emergency"}:
            return False, "NAV_INVALID_COMMAND", "priority must be one of: normal, high, emergency"

        # Either target_id OR x/y pose
        target_id = command.get("target_id")
        has_pose = ("x" in command and "y" in command)

        if target_id is None and not has_pose:
            return False, "NAV_INVALID_COMMAND", "missing target_id (or x/y pose)"

        if target_id is not None:
            if not isinstance(target_id, str) or not target_id.strip():
                return False, "NAV_INVALID_COMMAND", "target_id must be a non-empty string"
            target_id = target_id.strip()
            # Conservative allowed set for MQTT topics and YAML keys
            if not re.match(r"^[A-Za-z0-9_-]+$", target_id):
                return False, "NAV_INVALID_COMMAND", "target_id contains invalid characters"

            if position_registry is not None and hasattr(position_registry, "hasPosition"):
                if not position_registry.hasPosition(target_id):
                    return False, "NAV_INVALID_TARGET", f"unknown target_id: {target_id}"

        if has_pose:
            try:
                float(command.get("x"))
                float(command.get("y"))
                if "theta" in command and command.get("theta") is not None:
                    float(command.get("theta"))
            except (TypeError, ValueError):
                return False, "NAV_INVALID_COMMAND", "x/y/theta must be numeric"

        # Optional ttl_seconds (canonical v2.0)
        if "ttl_seconds" in command and command.get("ttl_seconds") is not None:
            try:
                ttl = float(command.get("ttl_seconds"))
            except (TypeError, ValueError):
                return False, "NAV_INVALID_COMMAND", "ttl_seconds must be numeric"
            if ttl <= 0.0 or ttl > 86400.0:
                return False, "NAV_INVALID_COMMAND", "ttl_seconds out of allowed range"

        return True, "ok", None

    @staticmethod
    def _parse_schema_version(v: str) -> Tuple[Optional[int], Optional[int]]:
        try:
            parts = v.split(".")
            if len(parts) != 2:
                return None, None
            major = int(parts[0])
            minor = int(parts[1])
            if major < 0 or minor < 0:
                return None, None
            return major, minor
        except Exception:
            return None, None

    def _validate_cancel(self, command: Dict[str, Any]) -> Tuple[bool, str, Optional[str]]:
        # task_id and reason are optional and ignored by the navigation layer.
        if "reason" in command and command.get("reason") is not None:
            if not isinstance(command.get("reason"), str):
                return False, "NAV_INVALID_COMMAND", "reason must be a string"
        if "task_id" in command and command.get("task_id") is not None:
            if not isinstance(command.get("task_id"), str):
                return False, "NAV_INVALID_COMMAND", "task_id must be a string"
        return True, "ok", None

    @staticmethod
    def _is_iso8601(ts: str) -> bool:
        """Accept a pragmatic subset of RFC3339/ISO8601."""
        try:
            # datetime.fromisoformat does not accept 'Z' prior to py3.11;
            # normalize 'Z' to '+00:00'
            t = ts.strip()
            if t.endswith("Z"):
                t = t[:-1] + "+00:00"
            datetime.fromisoformat(t)
            return True
        except Exception:
            return False

    def _prune(self, now_s: float) -> None:
        cutoff = now_s - self._ttl_seconds
        with self._lock:
            if not self._processed:
                return
            expired = [cid for cid, ts in self._processed.items() if ts < cutoff]
            for cid in expired:
                self._processed.pop(cid, None)
