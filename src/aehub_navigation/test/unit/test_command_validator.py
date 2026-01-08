#!/usr/bin/env python3
"""
Unit tests for CommandValidator
"""

import pytest
import uuid
from datetime import datetime, timezone
from unittest.mock import Mock

from aehub_navigation.command_validator import CommandValidator
from aehub_navigation.position_registry import PositionRegistry


@pytest.fixture
def validator():
    """Create CommandValidator instance"""
    return CommandValidator(max_processed_ids=100, ttl_seconds=60)


@pytest.fixture
def position_registry():
    """Create PositionRegistry with test positions"""
    registry = PositionRegistry()
    # Add test position using addPosition method
    registry.addPosition('test_position', x=1.0, y=2.0, theta=0.0, description='Test position')
    return registry


@pytest.fixture
def valid_command():
    """Create a valid command"""
    return {
        'command_id': str(uuid.uuid4()),
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'target_id': 'test_position',
        'priority': 'normal'
    }


def test_validate_valid_command(validator, position_registry, valid_command):
    """Test validation of a valid command"""
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is True
    assert error_msg is None


def test_validate_invalid_type(validator, position_registry):
    """Test validation rejects non-dict commands"""
    is_valid, error_msg = validator.validate_command("not a dict", position_registry)
    assert is_valid is False
    assert "dictionary" in error_msg.lower()


def test_validate_missing_required_field(validator, position_registry, valid_command):
    """Test validation rejects commands with missing required fields"""
    del valid_command['command_id']
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "command_id" in error_msg.lower()


def test_validate_invalid_uuid(validator, position_registry, valid_command):
    """Test validation rejects invalid UUID format"""
    valid_command['command_id'] = 'not-a-uuid'
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "uuid" in error_msg.lower()


def test_validate_duplicate_command(validator, position_registry, valid_command):
    """Test validation rejects duplicate command_id"""
    # First command should be valid
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is True
    
    # Mark as processed
    validator.mark_as_processed(valid_command['command_id'])
    
    # Second command with same ID should be rejected
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "duplicate" in error_msg.lower()


def test_validate_invalid_timestamp(validator, position_registry, valid_command):
    """Test validation rejects invalid timestamp format"""
    valid_command['timestamp'] = 'not-iso8601'
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "iso-8601" in error_msg.lower()


def test_validate_invalid_target_id(validator, position_registry, valid_command):
    """Test validation rejects invalid target_id"""
    valid_command['target_id'] = 'nonexistent_position'
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "unknown" in error_msg.lower() or "not found" in error_msg.lower()


def test_validate_invalid_priority(validator, position_registry, valid_command):
    """Test validation rejects invalid priority"""
    valid_command['priority'] = 'low'  # Low priority is forbidden
    is_valid, error_msg = validator.validate_command(valid_command, position_registry)
    assert is_valid is False
    assert "priority" in error_msg.lower()


def test_mark_as_processed(validator):
    """Test marking command as processed"""
    command_id = str(uuid.uuid4())
    assert not validator.is_duplicate(command_id)
    
    validator.mark_as_processed(command_id)
    assert validator.is_duplicate(command_id)


def test_cleanup_expired(validator):
    """Test cleanup of expired command IDs"""
    command_id = str(uuid.uuid4())
    validator.mark_as_processed(command_id)
    assert validator.is_duplicate(command_id)
    
    # Create validator with very short TTL
    short_ttl_validator = CommandValidator(max_processed_ids=100, ttl_seconds=0.1)
    short_ttl_validator.mark_as_processed(command_id)
    assert short_ttl_validator.is_duplicate(command_id)
    
    # Wait for TTL to expire
    import time
    time.sleep(0.2)
    
    # Cleanup should remove expired IDs
    short_ttl_validator.cleanup_expired()
    assert not short_ttl_validator.is_duplicate(command_id)


def test_get_processed_count(validator):
    """Test getting processed command count"""
    assert validator.get_processed_count() == 0
    
    for i in range(5):
        validator.mark_as_processed(str(uuid.uuid4()))
    
    assert validator.get_processed_count() == 5


def test_clear(validator):
    """Test clearing all processed IDs"""
    for i in range(5):
        validator.mark_as_processed(str(uuid.uuid4()))
    
    assert validator.get_processed_count() == 5
    
    validator.clear()
    assert validator.get_processed_count() == 0

