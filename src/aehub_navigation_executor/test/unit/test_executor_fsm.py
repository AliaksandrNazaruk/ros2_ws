#!/usr/bin/env python3
#
# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0

from aehub_navigation_executor.executor_fsm import NavigationExecutorFSM


def test_executor_fsm_ready_executing_done_ready():
    fsm = NavigationExecutorFSM()

    # Starts in READY
    s0 = fsm.snapshot()
    assert s0.public_state == "ready"

    # Start executing
    fsm.start("cmd-1")
    s1 = fsm.snapshot()
    assert s1.public_state == "executing"
    assert s1.active_command_id == "cmd-1"

    # Mark done -> DONE is published once
    fsm.mark_done()
    s2 = fsm.snapshot()
    assert s2.public_state == "done"

    # Next snapshot returns to READY automatically
    s3 = fsm.snapshot()
    assert s3.public_state == "ready"

