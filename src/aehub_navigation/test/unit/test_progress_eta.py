#!/usr/bin/env python3
"""
Unit tests for Progress and ETA calculation
"""

import pytest
from unittest.mock import Mock, MagicMock
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration


class TestProgressETACalculation:
    """Test suite for progress and ETA calculation"""
    
    def test_progress_calculation_basic(self):
        """Test basic progress calculation"""
        # Simulate feedback with distance_remaining
        initial_distance = 10.0  # meters
        distance_remaining = 5.0  # meters
        
        # Calculate progress
        progress_ratio = 1.0 - (distance_remaining / initial_distance)
        progress_percent = max(0, min(100, int(progress_ratio * 100)))
        
        assert progress_percent == 50  # 50% complete
    
    def test_progress_calculation_complete(self):
        """Test progress when goal is reached"""
        initial_distance = 10.0
        distance_remaining = 0.0
        
        progress_ratio = 1.0 - (distance_remaining / initial_distance)
        progress_percent = max(0, min(100, int(progress_ratio * 100)))
        
        assert progress_percent == 100
    
    def test_progress_calculation_small_distance(self):
        """Test progress with very small distances"""
        initial_distance = 0.1  # 10cm
        distance_remaining = 0.05  # 5cm
        
        # Should handle small distances gracefully
        if initial_distance > 0.01:
            progress_ratio = 1.0 - (distance_remaining / initial_distance)
            progress_percent = max(0, min(100, int(progress_ratio * 100)))
            assert progress_percent == 50
        else:
            # For very small distances, consider complete if < 10cm
            progress_percent = 100 if distance_remaining < 0.1 else 0
            assert progress_percent == 100
    
    def test_eta_calculation_from_velocity(self):
        """Test ETA calculation from average velocity"""
        distance_remaining = 5.0  # meters
        average_velocity = 0.5  # m/s
        
        eta_seconds = distance_remaining / average_velocity
        
        assert eta_seconds == 10.0  # 5m / 0.5 m/s = 10s
    
    def test_eta_validation(self):
        """Test ETA validation (non-negative)"""
        eta_values = [-5, 0, 10, 100]
        
        for eta in eta_values:
            validated_eta = max(0, int(eta))
            assert validated_eta >= 0
            assert isinstance(validated_eta, int)
    
    def test_progress_validation(self):
        """Test progress validation (0-100%)"""
        progress_values = [-10, 0, 50, 100, 150]
        
        for progress in progress_values:
            validated_progress = max(0, min(100, int(progress)))
            assert 0 <= validated_progress <= 100
    
    def test_velocity_calculation(self):
        """Test velocity calculation from distance change"""
        last_distance = 10.0
        current_distance = 8.0
        time_delta = 2.0  # seconds
        
        distance_delta = last_distance - current_distance
        velocity = distance_delta / time_delta
        
        assert velocity == 1.0  # 2m / 2s = 1 m/s
    
    def test_average_velocity_ema(self):
        """Test exponential moving average for velocity"""
        current_avg = 0.5  # m/s
        instant_velocity = 1.0  # m/s
        alpha = 0.3  # 30% weight to new value
        
        new_avg = 0.7 * current_avg + 0.3 * instant_velocity
        
        # Use approximate comparison due to floating point precision
        assert abs(new_avg - 0.65) < 0.0001  # 0.7 * 0.5 + 0.3 * 1.0 = 0.35 + 0.3 = 0.65
    
    def test_eta_fallback_when_nav2_not_provides(self):
        """Test ETA fallback when Nav2 doesn't provide estimated_time_remaining"""
        # Nav2 ETA not available
        nav2_eta = 0
        
        # Fallback calculation
        distance_remaining = 5.0
        average_velocity = 0.5
        
        if nav2_eta <= 0 and average_velocity > 0.01:
            eta_seconds = distance_remaining / average_velocity
        else:
            eta_seconds = nav2_eta
        
        assert eta_seconds == 10.0
    
    def test_edge_case_zero_initial_distance(self):
        """Test edge case when initial distance is very small"""
        initial_distance = 0.001  # 1mm
        distance_remaining = 0.0005  # 0.5mm
        
        # Should handle gracefully
        if initial_distance > 0.01:
            progress_percent = int((1.0 - distance_remaining / initial_distance) * 100)
        else:
            # For very small distances, use distance threshold
            progress_percent = 100 if distance_remaining < 0.1 else 0
        
        assert progress_percent == 100


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

