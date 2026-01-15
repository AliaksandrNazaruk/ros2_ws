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
Unit tests for CommandRateLimiter
"""

import pytest
import time
import uuid

from aehub_navigation.command_rate_limiter import CommandRateLimiter


@pytest.fixture
def rate_limiter():
    """Create CommandRateLimiter instance"""
    return CommandRateLimiter(min_interval_seconds=0.1)


def test_check_rate_limit_allowed(rate_limiter):
    """Test rate limit allows commands with sufficient interval"""
    command_id = str(uuid.uuid4())
    current_time = time.time()
    
    is_allowed, error_msg = rate_limiter.check_rate_limit(command_id, current_time)
    assert is_allowed is True
    assert error_msg is None


def test_check_rate_limit_exceeded(rate_limiter):
    """Test rate limit rejects commands sent too frequently"""
    command_id = str(uuid.uuid4())
    current_time = time.time()
    
    # First command should be allowed
    is_allowed, error_msg = rate_limiter.check_rate_limit(command_id, current_time)
    assert is_allowed is True
    
    # Second command immediately after should be rejected
    is_allowed, error_msg = rate_limiter.check_rate_limit(command_id, current_time + 0.05)
    assert is_allowed is False
    assert "rate limit" in error_msg.lower() or "too frequently" in error_msg.lower()


def test_check_rate_limit_after_interval(rate_limiter):
    """Test rate limit allows commands after minimum interval"""
    command_id = str(uuid.uuid4())
    current_time = time.time()
    
    # First command
    is_allowed, _ = rate_limiter.check_rate_limit(command_id, current_time)
    assert is_allowed is True
    
    # Second command after interval should be allowed
    is_allowed, error_msg = rate_limiter.check_rate_limit(command_id, current_time + 0.15)
    assert is_allowed is True
    assert error_msg is None


def test_update_last_command_time(rate_limiter):
    """Test updating last command time"""
    command_id = str(uuid.uuid4())
    current_time = time.time()
    
    rate_limiter.update_last_command_time(command_id, current_time)
    last_time = rate_limiter.get_last_command_time(command_id)
    assert last_time == current_time


def test_get_last_command_time_nonexistent(rate_limiter):
    """Test getting last command time for nonexistent command"""
    command_id = str(uuid.uuid4())
    last_time = rate_limiter.get_last_command_time(command_id)
    assert last_time is None


def test_cleanup_old_entries(rate_limiter):
    """Test cleanup of old entries"""
    command_id1 = str(uuid.uuid4())
    command_id2 = str(uuid.uuid4())
    current_time = time.time()
    
    # Add old entry (beyond max_age)
    rate_limiter.update_last_command_time(command_id1, current_time - 4000)
    # Add recent entry
    rate_limiter.update_last_command_time(command_id2, current_time)
    
    # Cleanup should remove old entry
    rate_limiter.cleanup_old_entries(max_age_seconds=3600, current_time=current_time)
    
    assert rate_limiter.get_last_command_time(command_id1) is None
    assert rate_limiter.get_last_command_time(command_id2) == current_time


def test_clear(rate_limiter):
    """Test clearing all entries"""
    command_id = str(uuid.uuid4())
    current_time = time.time()
    
    rate_limiter.update_last_command_time(command_id, current_time)
    assert rate_limiter.get_last_command_time(command_id) == current_time
    
    rate_limiter.clear()
    assert rate_limiter.get_last_command_time(command_id) is None

