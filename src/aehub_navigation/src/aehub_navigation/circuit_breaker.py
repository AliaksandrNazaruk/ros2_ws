#!/usr/bin/env python3

"""
Circuit Breaker for Config Service

Prevents excessive requests when Config Service is unavailable.
Uses exponential backoff and opens circuit after consecutive failures.

Based on the Circuit Breaker pattern:
- CLOSED: Normal operation, requests allowed
- OPEN: Service unavailable, requests blocked (use cached config)
- HALF_OPEN: Testing if service recovered, allow one request
"""

import time
from enum import Enum
from typing import Optional, Callable
from threading import Lock


class CircuitState(Enum):
    """Circuit breaker states"""
    CLOSED = "closed"  # Normal operation
    OPEN = "open"  # Service unavailable, block requests
    HALF_OPEN = "half_open"  # Testing recovery


class CircuitBreaker:
    """
    Circuit breaker for external service calls.
    
    Prevents cascading failures by blocking requests when service is down.
    """
    
    def __init__(
        self,
        failure_threshold: int = 5,
        timeout: float = 60.0,
        expected_exception: type = Exception
    ):
        """
        Initialize circuit breaker.
        
        Args:
            failure_threshold: Number of consecutive failures before opening circuit
            timeout: Time in seconds before attempting to close circuit (half-open)
            expected_exception: Exception type that indicates failure
        """
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.expected_exception = expected_exception
        
        self.failure_count = 0
        self.last_failure_time: Optional[float] = None
        self.state = CircuitState.CLOSED
        self._lock = Lock()
    
    def call(self, func: Callable, *args, **kwargs):
        """
        Execute function with circuit breaker protection.
        
        Args:
            func: Function to call
            *args: Positional arguments for function
            **kwargs: Keyword arguments for function
            
        Returns:
            Function result
            
        Raises:
            CircuitBreakerOpenError: If circuit is open
            Exception: If function raises exception
        """
        with self._lock:
            # Check if circuit is open
            if self.state == CircuitState.OPEN:
                # Check if timeout has passed (try half-open)
                if self.last_failure_time and \
                   (time.time() - self.last_failure_time) >= self.timeout:
                    self.state = CircuitState.HALF_OPEN
                    self.failure_count = 0
                else:
                    raise CircuitBreakerOpenError(
                        f"Circuit breaker is OPEN. "
                        f"Last failure: {self.last_failure_time}"
                    )
        
        # Try to call function
        try:
            result = func(*args, **kwargs)
            # Success - reset failure count
            with self._lock:
                if self.state == CircuitState.HALF_OPEN:
                    # Success in half-open state - close circuit
                    self.state = CircuitState.CLOSED
                self.failure_count = 0
            return result
            
        except self.expected_exception as e:
            # Failure - increment count
            with self._lock:
                self.failure_count += 1
                self.last_failure_time = time.time()
                
                if self.failure_count >= self.failure_threshold:
                    # Open circuit
                    self.state = CircuitState.OPEN
                elif self.state == CircuitState.HALF_OPEN:
                    # Failure in half-open - open circuit again
                    self.state = CircuitState.OPEN
            
            # Re-raise exception
            raise
    
    def is_open(self) -> bool:
        """Check if circuit is open"""
        with self._lock:
            return self.state == CircuitState.OPEN
    
    def reset(self):
        """Manually reset circuit breaker to closed state"""
        with self._lock:
            self.state = CircuitState.CLOSED
            self.failure_count = 0
            self.last_failure_time = None
    
    def get_state(self) -> CircuitState:
        """Get current circuit state"""
        with self._lock:
            return self.state


class CircuitBreakerOpenError(Exception):
    """Exception raised when circuit breaker is open"""
    pass

