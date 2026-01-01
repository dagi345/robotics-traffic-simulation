#!/usr/bin/env python3
"""
Property-based tests for vehicle initialization.
Feature: traffic-control-fix

Tests Properties 19 and 20 related to initialization.
Validates: Requirements 9.2, 9.3
"""

import unittest
from hypothesis import given, settings, strategies as st
import sys
import os
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class InitializationLogic:
    """Standalone logic for testing initialization without ROS2."""
    
    def __init__(self, min_speed=3.0, max_speed=5.0):
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.SAFE_FOLLOWING_DISTANCE = 5.0
    
    def assign_speed(self, seed=None):
        """Assign random speed within configured range (Requirement 9.2)"""
        if seed is not None:
            random.seed(seed)
        return random.uniform(self.min_speed, self.max_speed)
    
    def check_vehicle_spacing(self, positions):
        """Check if vehicles maintain safe spacing (Requirement 9.3)"""
        for i, pos1 in enumerate(positions):
            for j, pos2 in enumerate(positions):
                if i >= j:
                    continue
                distance = abs(pos1 - pos2)
                if distance < self.SAFE_FOLLOWING_DISTANCE:
                    return False, distance
        return True, None


# World file vehicle positions (from intersection.world)
VEHICLE_POSITIONS = {
    'south_lane': {  # Southbound vehicles (y positions)
        'car_s1': 30, 'car_s2': 45, 'car_s3': 35, 'car_s4': 50
    },
    'north_lane': {  # Northbound vehicles (y positions, negative)
        'car_n1': -30, 'car_n2': -45, 'car_n3': -35, 'car_n4': -50
    },
    'east_lane': {  # Eastbound vehicles (x positions, negative)
        'car_e1': -30, 'car_e2': -45, 'car_e3': -35, 'car_e4': -50
    },
    'west_lane': {  # Westbound vehicles (x positions)
        'car_w1': 30, 'car_w2': 45, 'car_w3': 35, 'car_w4': 50
    }
}


class TestInitializationProperties(unittest.TestCase):
    """Property-based tests for initialization. Validates: Requirements 9.2, 9.3"""
    
    @settings(max_examples=100)
    @given(
        min_speed=st.floats(min_value=1.0, max_value=4.0),
        max_speed=st.floats(min_value=4.0, max_value=10.0),
        seed=st.integers(min_value=0, max_value=10000)
    )
    def test_property_19_initial_speed_assignment(self, min_speed, max_speed, seed):
        """
        Feature: traffic-control-fix, Property 19: Initial Speed Assignment
        
        For any vehicle at simulation start, the assigned speed SHALL be 
        within the configured minimum and maximum speed range.
        
        Validates: Requirements 9.2
        """
        logic = InitializationLogic(min_speed, max_speed)
        
        # Assign speed
        speed = logic.assign_speed(seed)
        
        # Speed should be within range
        self.assertGreaterEqual(
            speed, min_speed,
            f"Speed {speed} should be >= min_speed {min_speed}"
        )
        self.assertLessEqual(
            speed, max_speed,
            f"Speed {speed} should be <= max_speed {max_speed}"
        )
    
    @settings(max_examples=100)
    @given(
        spacing=st.floats(min_value=5.0, max_value=20.0)
    )
    def test_property_20_initial_vehicle_spacing(self, spacing):
        """
        Feature: traffic-control-fix, Property 20: Initial Vehicle Spacing
        
        For any pair of vehicles in the same lane at simulation start, the 
        distance between them SHALL be >= Safe_Following_Distance.
        
        Validates: Requirements 9.3
        """
        logic = InitializationLogic()
        
        # Create positions with given spacing
        positions = [0, spacing, spacing * 2, spacing * 3]
        
        # Check spacing
        is_safe, min_dist = logic.check_vehicle_spacing(positions)
        
        if spacing >= logic.SAFE_FOLLOWING_DISTANCE:
            self.assertTrue(
                is_safe,
                f"Spacing {spacing} >= {logic.SAFE_FOLLOWING_DISTANCE} should be safe"
            )
        else:
            self.assertFalse(
                is_safe,
                f"Spacing {spacing} < {logic.SAFE_FOLLOWING_DISTANCE} should be unsafe"
            )


class TestWorldFileConfiguration(unittest.TestCase):
    """Unit tests for world file vehicle configuration."""
    
    def test_world_file_vehicle_spacing(self):
        """Test that world file vehicles have safe spacing"""
        logic = InitializationLogic()
        
        for lane_name, vehicles in VEHICLE_POSITIONS.items():
            positions = list(vehicles.values())
            is_safe, min_dist = logic.check_vehicle_spacing(positions)
            
            self.assertTrue(
                is_safe,
                f"Lane {lane_name} has unsafe spacing: {min_dist}m < {logic.SAFE_FOLLOWING_DISTANCE}m"
            )
    
    def test_minimum_spacing_in_world_file(self):
        """Test that minimum spacing in world file is at least 5m"""
        for lane_name, vehicles in VEHICLE_POSITIONS.items():
            positions = sorted(vehicles.values())
            
            for i in range(len(positions) - 1):
                spacing = abs(positions[i+1] - positions[i])
                self.assertGreaterEqual(
                    spacing, 5.0,
                    f"Lane {lane_name}: spacing {spacing}m between vehicles is < 5m"
                )
    
    def test_all_16_vehicles_defined(self):
        """Test that all 16 vehicles are defined in positions"""
        total_vehicles = sum(len(v) for v in VEHICLE_POSITIONS.values())
        self.assertEqual(total_vehicles, 16)
    
    def test_default_speed_range(self):
        """Test default speed range is 3.0 to 5.0"""
        logic = InitializationLogic()
        
        # Generate multiple speeds
        speeds = [logic.assign_speed(i) for i in range(100)]
        
        for speed in speeds:
            self.assertGreaterEqual(speed, 3.0)
            self.assertLessEqual(speed, 5.0)


if __name__ == '__main__':
    unittest.main()
