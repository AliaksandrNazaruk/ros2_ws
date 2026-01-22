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
Unit tests for command validator.

Traceability: SRS-CMD-01, SRS-CMD-02, SRS-CMD-03, SRS-DD-01, SRS-DD-02, SRS-DD-03, SRS-RL-01
"""

import pytest
import time
from aehub_navigation_executor.command_validator import CommandValidator, ValidationResult


# =========================
# SRS-CMD-01: Mandatory command_id
# =========================

def test_tc_cmd_01_missing_command_id_rejected():
    """TC-CMD-01: Missing command_id → rejected."""
    validator = CommandValidator()
    
    cmd = '{"timestamp": 100, "x": 1.0, "y": 2.0}'
    ok, reason, data, is_dup, outcome = validator.validate_navigate(cmd)
    assert not ok
    assert reason == "missing_fields"


def test_tc_cmd_02_non_uuid_command_id_accepted():
    """TC-CMD-02: Non-UUID command_id → accepted (any string is valid)."""
    validator = CommandValidator()
    
    # Note: SRS doesn't require UUID format, just presence
    cmd = '{"command_id": "test-123", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok, reason, data, is_dup, outcome = validator.validate_navigate(cmd)
    assert ok
    assert not is_dup


# =========================
# SRS-CMD-02: navigateTo payload validation
# =========================

def test_tc_cmd_04_target_id_or_coords_required():
    """TC-CMD-04: target_id OR (x,y[,theta]) required."""
    # With target_id (use separate validators to avoid rate limiting)
    validator1 = CommandValidator()
    cmd1 = '{"command_id": "test-1", "timestamp": 100, "target_id": "dock_A"}'
    ok1, reason1, data1, is_dup1, outcome1 = validator1.validate_navigate(cmd1)
    assert ok1
    assert not is_dup1
    
    # With x, y
    validator2 = CommandValidator()
    cmd2 = '{"command_id": "test-2", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator2.validate_navigate(cmd2)
    assert ok2
    assert not is_dup2
    
    # With x, y, theta
    validator3 = CommandValidator()
    cmd3 = '{"command_id": "test-3", "timestamp": 100, "x": 1.0, "y": 2.0, "theta": 1.57}'
    ok3, reason3, data3, is_dup3, outcome3 = validator3.validate_navigate(cmd3)
    assert ok3
    assert not is_dup3


def test_tc_cmd_05_both_missing_rejected():
    """TC-CMD-05: Both missing → rejected."""
    validator = CommandValidator()
    
    cmd = '{"command_id": "test-123", "timestamp": 100}'
    ok, reason, data, is_dup, outcome = validator.validate_navigate(cmd)
    assert not ok
    assert reason == "missing_pose"


# =========================
# SRS-CMD-03: cancel minimal payload
# =========================

def test_tc_cmd_07_cancel_with_only_command_id_valid():
    """TC-CMD-07: cancel with only command_id valid."""
    validator = CommandValidator()
    
    cmd = '{"command_id": "test-cancel", "timestamp": 100}'
    ok, reason, data, is_dup, outcome = validator.validate_cancel(cmd)
    assert ok
    assert not is_dup


# =========================
# SRS-DD-01: Idempotency per command_id
# =========================

def test_tc_dd_01_second_same_command_id_detected():
    """TC-DD-01: Second same command_id detected."""
    validator = CommandValidator()
    
    # First command
    cmd1 = '{"command_id": "test-123", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1)
    assert ok1
    assert not is_dup1
    
    # Store outcome
    validator.store_outcome("test-123", ack_data={"status": "accepted"})
    
    # Duplicate command
    cmd2 = '{"command_id": "test-123", "timestamp": 200, "x": 1.0, "y": 2.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_navigate(cmd2)
    assert not ok2
    assert is_dup2
    assert outcome2 is not None
    assert outcome2.command_id == "test-123"
    assert outcome2.ack_data is not None


# =========================
# SRS-DD-02: Cross-topic deduplication
# =========================

def test_tc_dd_04_same_command_id_across_topics():
    """TC-DD-04: Same command_id on navigateTo + cancel."""
    validator = CommandValidator()
    
    # First: navigateTo
    cmd1 = '{"command_id": "test-x", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1)
    assert ok1
    assert not is_dup1
    
    # Store outcome
    validator.store_outcome("test-x", ack_data={"status": "accepted"})
    
    # Second: cancel with same command_id (should be duplicate)
    cmd2 = '{"command_id": "test-x", "timestamp": 200}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_cancel(cmd2)
    assert not ok2
    assert is_dup2
    assert outcome2 is not None
    assert outcome2.command_id == "test-x"


# =========================
# SRS-DD-03: TTL eviction
# =========================

def test_tc_dd_06_command_id_expires_after_ttl():
    """TC-DD-06: command_id expires after TTL."""
    validator = CommandValidator(cache_ttl_sec=0.1)  # Very short TTL for testing
    
    # First command
    cmd1 = '{"command_id": "test-ttl", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1, rate_limit_sec=0.0)  # No rate limit for test
    assert ok1
    assert not is_dup1
    
    # Store outcome
    validator.store_outcome("test-ttl", ack_data={"status": "accepted"})
    
    # Immediately: should be duplicate
    cmd2 = '{"command_id": "test-ttl", "timestamp": 200, "x": 1.0, "y": 2.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_navigate(cmd2, rate_limit_sec=0.0)
    assert not ok2
    assert is_dup2
    
    # Wait for TTL to expire
    time.sleep(0.2)
    
    # Cleanup expired
    validator.cleanup_expired()
    
    # After TTL: should be valid (duplicate expired, but need to bypass rate limiter)
    # Rate limiter uses global time, so we bypass it for this test
    cmd3 = '{"command_id": "test-ttl", "timestamp": 300, "x": 1.0, "y": 2.0}'
    ok3, reason3, data3, is_dup3, outcome3 = validator.validate_navigate(cmd3, rate_limit_sec=0.0)  # No rate limit
    assert ok3
    assert not is_dup3


def test_tc_dd_07_reaccepted_after_ttl():
    """TC-DD-07: Re-accepted after TTL."""
    validator = CommandValidator(cache_ttl_sec=0.1)
    
    # First command (bypass rate limit for test)
    cmd1 = '{"command_id": "test-ttl2", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1, rate_limit_sec=0.0)
    validator.store_outcome("test-ttl2", ack_data={"status": "accepted"})
    
    # Wait for TTL
    time.sleep(0.2)
    validator.cleanup_expired()
    
    # Should be accepted again (bypass rate limit for test)
    cmd2 = '{"command_id": "test-ttl2", "timestamp": 300, "x": 1.0, "y": 2.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_navigate(cmd2, rate_limit_sec=0.0)
    assert ok2
    assert not is_dup2


# =========================
# SRS-RL-01: navigateTo rate limiting
# =========================

def test_tc_rl_01_rate_limiter_triggers():
    """TC-RL-01: Rate limiter triggers."""
    validator = CommandValidator()
    
    # First command
    cmd1 = '{"command_id": "test-1", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1, rate_limit_sec=1.0)
    assert ok1
    
    # Second command too soon
    cmd2 = '{"command_id": "test-2", "timestamp": 101, "x": 3.0, "y": 4.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_navigate(cmd2, rate_limit_sec=1.0)
    assert not ok2
    assert reason2 == "rate_limited"


def test_tc_rl_03_cancel_bypasses_limiter():
    """TC-RL-03: cancel bypasses limiter."""
    validator = CommandValidator()
    
    # Navigate (triggers rate limiter)
    cmd1 = '{"command_id": "test-1", "timestamp": 100, "x": 1.0, "y": 2.0}'
    ok1, reason1, data1, is_dup1, outcome1 = validator.validate_navigate(cmd1, rate_limit_sec=1.0)
    
    # Cancel immediately (should NOT be rate-limited)
    cmd2 = '{"command_id": "test-cancel", "timestamp": 101}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_cancel(cmd2)
    assert ok2
    assert not is_dup2


# =========================
# SRS-NEG-01: Malformed JSON
# =========================

def test_tc_neg_01_json_parse_error():
    """TC-NEG-01: JSON parse error."""
    validator = CommandValidator()
    
    # Invalid JSON
    cmd = '{invalid json}'
    ok, reason, data, is_dup, outcome = validator.validate_navigate(cmd)
    assert not ok
    assert reason == "invalid_json"
    
    # Missing timestamp
    cmd2 = '{"command_id": "test-123", "x": 1.0, "y": 2.0}'
    ok2, reason2, data2, is_dup2, outcome2 = validator.validate_navigate(cmd2)
    assert not ok2
    assert reason2 == "missing_fields"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
