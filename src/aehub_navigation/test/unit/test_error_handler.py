#!/usr/bin/env python3
"""
Unit tests for NavigationErrorHandler
"""

import pytest
from unittest.mock import Mock, MagicMock

from aehub_navigation.error_handler import NavigationErrorHandler


@pytest.fixture
def logger():
    """Create mock logger"""
    return Mock()


@pytest.fixture
def status_publisher():
    """Create mock status publisher"""
    return Mock()


@pytest.fixture
def error_handler(logger, status_publisher):
    """Create NavigationErrorHandler instance"""
    return NavigationErrorHandler(logger=logger, status_publisher=status_publisher)


def test_handle_error(logger, status_publisher, error_handler):
    """Test handling error with logging and publishing"""
    error_handler.handle_error(
        'NAV_INVALID_COMMAND',
        {'command_id': 'test-123', 'target_id': 'pos1'},
        'test-123'
    )
    
    # Should log error
    logger.error.assert_called_once()
    log_call = logger.error.call_args[0][0]
    assert 'NAV_INVALID_COMMAND' in log_call
    assert 'test-123' in log_call
    
    # Should publish error
    status_publisher.updateStatus.assert_called_once()
    status_publisher.publishStatus.assert_called_once()


def test_handle_error_without_publisher(logger):
    """Test handling error without status publisher"""
    error_handler = NavigationErrorHandler(logger=logger, status_publisher=None)
    
    error_handler.handle_error(
        'NAV_INVALID_COMMAND',
        {'command_id': 'test-123'},
        'test-123'
    )
    
    # Should log error
    logger.error.assert_called_once()
    
    # Should not crash without publisher


def test_log_error(logger, error_handler):
    """Test logging error"""
    error_handler.log_error(
        'NAV_INVALID_COMMAND',
        'Test error message',
        {'context': 'test'},
        'test-123'
    )
    
    logger.error.assert_called_once()
    log_call = logger.error.call_args[0][0]
    assert 'NAV_INVALID_COMMAND' in log_call
    assert 'Test error message' in log_call
    assert 'test-123' in log_call


def test_log_exception(logger, error_handler):
    """Test logging exception with traceback"""
    exception = ValueError('Test exception')
    
    error_handler.log_exception(
        exception,
        {'context': 'test'},
        'test-123'
    )
    
    logger.error.assert_called_once()
    # Check that exc_info was passed for traceback
    assert logger.error.call_args[1].get('exc_info') is True


def test_publish_error(status_publisher, error_handler):
    """Test publishing error to MQTT"""
    error_handler.publish_error(
        'NAV_INVALID_COMMAND',
        'Test error message',
        'test-123'
    )
    
    # Should update status with error
    status_publisher.updateStatus.assert_called_once()
    call_args = status_publisher.updateStatus.call_args
    assert call_args[1]['error_code'] == 'NAV_INVALID_COMMAND'
    assert call_args[1]['error_message'] == 'Test error message'
    assert call_args[1]['command_id'] == 'test-123'
    
    # Should publish status
    status_publisher.publishStatus.assert_called_once()


def test_get_error_code_description():
    """Test getting error code description"""
    desc = NavigationErrorHandler.get_error_code_description('NAV_INVALID_COMMAND')
    assert 'Invalid navigation command' in desc
    
    # Test unknown error code
    desc = NavigationErrorHandler.get_error_code_description('UNKNOWN_CODE')
    assert 'Unknown error code' in desc


def test_error_codes_defined():
    """Test that all standard error codes are defined"""
    assert 'NAV_INVALID_COMMAND' in NavigationErrorHandler.ERROR_CODES
    assert 'NAV_DUPLICATE_COMMAND' in NavigationErrorHandler.ERROR_CODES
    assert 'NAV_RATE_LIMIT_EXCEEDED' in NavigationErrorHandler.ERROR_CODES
    assert 'NAV_SERVER_UNAVAILABLE' in NavigationErrorHandler.ERROR_CODES
    assert 'NAV_GOAL_REJECTED' in NavigationErrorHandler.ERROR_CODES

