#!/usr/bin/env python3
#
# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class ExecutorState(Enum):
    READY = "ready"
    EXECUTING = "executing"
    DONE = "done"


@dataclass(frozen=True)
class ExecutorSnapshot:
    public_state: str
    active_command_id: Optional[str]


class NavigationExecutorFSM:
    """
    High-level executor FSM:
      READY -> EXECUTING -> DONE -> READY

    Notes:
    - DONE is an *edge state*: publish it once, then return to READY.
    - Deterministic: no sleeps, no background threads.
    """

    def __init__(self) -> None:
        self._state = ExecutorState.READY
        self._active_command_id: Optional[str] = None
        self._done_pending: bool = False

    def set_ready(self) -> None:
        self._state = ExecutorState.READY
        self._active_command_id = None
        self._done_pending = False

    def start(self, command_id: str) -> None:
        self._state = ExecutorState.EXECUTING
        self._active_command_id = command_id or None
        self._done_pending = False

    def mark_done(self) -> None:
        # Mark DONE and schedule an automatic return to READY on next snapshot.
        self._state = ExecutorState.DONE
        self._done_pending = True

    def snapshot(self) -> ExecutorSnapshot:
        snap = ExecutorSnapshot(
            public_state=self._state.value,
            active_command_id=self._active_command_id,
        )

        # DONE is published once, then immediately returns to READY.
        if self._done_pending:
            self.set_ready()

        return snap

