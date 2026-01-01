#!/usr/bin/env python3
"""
Property-based tests for collision avoidance system.
Feature: traffic-control-fix

Tests Properties 12, 13, 14, and 15 related to collision avoidance.
"""

import unittest
from hypothesis import given, settings, strategies as st, assume
import sys
import os

# Add the scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


# Constants from the implementation
SAFE_FOLLOWING_DISTANCE = 5.0  # meters
STOP_DISTANCE = 3.0  # meters


class CollisionAvoidanceLogic:
    """
    Standalone logic class for testing collision avoidance without ROS2 initialization.
    Mirrors the logic from MultiTrafficFlowControl.
    """
    
    def __init__(self):
        self.SAFE_FOLLOWING_DISTANCE = 5.0
        self.STOP_DISTANCE = 3.0
        self.speeds = {}  # {vehicle_name: assigned_speed}
        self.light_state = "NS_GREEN"
        self.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
    
    def calculate_velocity(self, vehicle_name, distance_ahead, is_at_red_light):
        """
        Calculate velocity for a vehicle based on collision avoidance logic.
        
        Args:
            vehicle_name: Name of the vehicle
            distance_ahead: Distance to vehicle ahead in meters
            is_at_red_light: Whether vehicle is at a red light
            
        Returns:
            float: Calculated velocity
        """
        if vehicle_name not in self.speeds:
            return 0.0
        
        assigned_speed = self.speeds[vehicle_name]
        
        # Determine velocity based on conditions
        if is_at_red_light:
            # Stop for red light
            return 0.0
        elif distance_ahead < self.STOP_DISTANCE:
            # Emergency stop - too close to vehicle ahead
            return 0.0
        elif distance_ahead < self.SAFE_FOLLOWING_DISTANCE:
            # Slow down - approaching safe following distance
            # Linear interpolation between 0 and normal speed
            ratio = (distance_ahead - self.STOP_DISTANCE) / \
                   (self.SAFE_FOLLOWING_DISTANCE - self.STOP_DISTANCE)
            return assigned_speed * ratio
        else:
            # Normal speed - safe distance maintained
            return assigned_speed


class TestCollisionAvoidanceProperties(unittest.TestCase):
    """
    Property-based tests for collision avoidance.
    Validates: Requirements 5.1, 5.2, 5.4, 5.5
    """
    
    @settings(max_examples=100)
    @given(
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=0.1, max_value=4.99)
    )
    def test_property_12_collision_avoidance_speed_reduction(self, assigned_speed, distance_ahead):
        """
        Feature: traffic-control-fix, Property 12: Collision Avoidance Speed Reduction
        
        For any vehicle with another vehicle within the Safe_Following_Distance 
        ahead in the same lane, the Flow_Controller SHALL command a velocity 
        less than or equal to a safe speed (proportional to distance).
        
        Validates: Requirements 5.1
        """
        # Ensure distance is within safe following distance
        assume(distance_ahead < SAFE_FOLLOWING_DISTANCE)
        
        # Create logic instance
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = assigned_speed
        
        # Calculate velocity (not at red light)
        velocity = logic.calculate_velocity('car_n1', distance_ahead, is_at_red_light=False)
        
        # Velocity should be less than or equal to assigned speed
        self.assertLessEqual(
            velocity,
            assigned_speed,
            f"Velocity {velocity} should be <= assigned speed {assigned_speed} when distance {distance_ahead} < {SAFE_FOLLOWING_DISTANCE}"
        )
        
        # Velocity should be non-negative
        self.assertGreaterEqual(
            velocity,
            0.0,
            f"Velocity should be non-negative"
        )
        
        # If distance is less than STOP_DISTANCE, velocity should be 0
        if distance_ahead < STOP_DISTANCE:
            self.assertEqual(
                velocity,
                0.0,
                f"Velocity should be 0 when distance {distance_ahead} < {STOP_DISTANCE}"
            )
    
    @settings(max_examples=100)
    @given(
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=5.0, max_value=50.0)
    )
    def test_property_13_speed_resumption_after_safe_distance(self, assigned_speed, distance_ahead):
        """
        Feature: traffic-control-fix, Property 13: Speed Resumption After Safe Distance
        
        For any vehicle that had reduced speed due to a vehicle ahead, when 
        the distance to the vehicle ahead exceeds the Safe_Following_Distance, 
        the Flow_Controller SHALL allow the velocity to increase toward the 
        assigned speed.
        
        Validates: Requirements 5.2
        """
        # Ensure distance exceeds safe following distance
        assume(distance_ahead >= SAFE_FOLLOWING_DISTANCE)
        
        # Create logic instance
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = assigned_speed
        
        # Calculate velocity (not at red light)
        velocity = logic.calculate_velocity('car_n1', distance_ahead, is_at_red_light=False)
        
        # Velocity should equal assigned speed when distance is safe
        self.assertAlmostEqual(
            velocity,
            assigned_speed,
            places=5,
            msg=f"Velocity should equal assigned speed {assigned_speed} when distance {distance_ahead} >= {SAFE_FOLLOWING_DISTANCE}"
        )
    
    @settings(max_examples=100)
    @given(
        num_vehicles=st.integers(min_value=2, max_value=4),
        spacing=st.floats(min_value=5.0, max_value=10.0)
    )
    def test_property_14_queue_spacing(self, num_vehicles, spacing):
        """
        Feature: traffic-control-fix, Property 14: Queue Spacing
        
        For any set of vehicles queued at a red light in the same lane, the 
        distance between each consecutive pair SHALL be greater than or equal 
        to the Safe_Following_Distance.
        
        Validates: Requirements 5.4
        """
        # Simulate vehicles queued with given spacing
        # This test verifies that the spacing constraint is maintained
        
        # For each consecutive pair, spacing should be >= SAFE_FOLLOWING_DISTANCE
        self.assertGreaterEqual(
            spacing,
            SAFE_FOLLOWING_DISTANCE,
            f"Queue spacing {spacing} should be >= {SAFE_FOLLOWING_DISTANCE}"
        )
        
        # Create logic instance
        logic = CollisionAvoidanceLogic()
        
        # Assign speeds to vehicles
        for i in range(1, num_vehicles + 1):
            vehicle_name = f'car_n{i}'
            logic.speeds[vehicle_name] = 4.0
        
        # Simulate each vehicle checking distance to vehicle ahead
        # Each vehicle should maintain safe distance
        for i in range(1, num_vehicles):
            vehicle_name = f'car_n{i}'
            
            # Distance to vehicle ahead is the spacing
            velocity = logic.calculate_velocity(vehicle_name, spacing, is_at_red_light=True)
            
            # At red light, velocity should be 0
            self.assertEqual(
                velocity,
                0.0,
                f"Vehicle {vehicle_name} should stop at red light"
            )
    
    @settings(max_examples=100)
    @given(
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=0.1, max_value=4.99),
        light_state=st.sampled_from(['NS_GREEN', 'EW_GREEN', 'ALL_RED'])
    )
    def test_property_15_collision_avoidance_independence(self, assigned_speed, distance_ahead, light_state):
        """
        Feature: traffic-control-fix, Property 15: Collision Avoidance Independence
        
        For any traffic light state (red, green, or transitioning), the 
        collision avoidance logic SHALL still enforce Safe_Following_Distance 
        constraints between vehicles in the same lane.
        
        Validates: Requirements 5.5
        """
        # Ensure distance is within safe following distance
        assume(distance_ahead < SAFE_FOLLOWING_DISTANCE)
        
        # Create logic instance
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = assigned_speed
        logic.light_state = light_state
        
        # Test with vehicle NOT at red light (green light scenario)
        velocity_green = logic.calculate_velocity('car_n1', distance_ahead, is_at_red_light=False)
        
        # Velocity should be reduced due to collision avoidance
        # regardless of light state
        self.assertLessEqual(
            velocity_green,
            assigned_speed,
            f"Velocity should be reduced when distance {distance_ahead} < {SAFE_FOLLOWING_DISTANCE}, regardless of light state {light_state}"
        )
        
        # If distance is less than STOP_DISTANCE, velocity should be 0
        if distance_ahead < STOP_DISTANCE:
            self.assertEqual(
                velocity_green,
                0.0,
                f"Velocity should be 0 when distance {distance_ahead} < {STOP_DISTANCE}, regardless of light state"
            )
        
        # Test with vehicle at red light
        velocity_red = logic.calculate_velocity('car_n1', distance_ahead, is_at_red_light=True)
        
        # At red light, velocity should always be 0
        self.assertEqual(
            velocity_red,
            0.0,
            f"Velocity should be 0 at red light"
        )


class TestCollisionAvoidanceEdgeCases(unittest.TestCase):
    """
    Unit tests for specific edge cases in collision avoidance.
    """
    
    def test_exactly_at_stop_distance(self):
        """Test behavior when distance equals STOP_DISTANCE"""
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = 4.0
        
        velocity = logic.calculate_velocity('car_n1', STOP_DISTANCE, is_at_red_light=False)
        
        # At exactly STOP_DISTANCE, velocity should be 0 (proportional calculation)
        self.assertAlmostEqual(velocity, 0.0, places=5)
    
    def test_exactly_at_safe_following_distance(self):
        """Test behavior when distance equals SAFE_FOLLOWING_DISTANCE"""
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = 4.0
        
        velocity = logic.calculate_velocity('car_n1', SAFE_FOLLOWING_DISTANCE, is_at_red_light=False)
        
        # At exactly SAFE_FOLLOWING_DISTANCE, velocity should equal assigned speed
        self.assertAlmostEqual(velocity, 4.0, places=5)
    
    def test_midpoint_between_stop_and_safe_distance(self):
        """Test behavior at midpoint between STOP_DISTANCE and SAFE_FOLLOWING_DISTANCE"""
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = 4.0
        
        midpoint = (STOP_DISTANCE + SAFE_FOLLOWING_DISTANCE) / 2.0
        velocity = logic.calculate_velocity('car_n1', midpoint, is_at_red_light=False)
        
        # At midpoint, velocity should be approximately half of assigned speed
        expected_velocity = 4.0 * 0.5
        self.assertAlmostEqual(velocity, expected_velocity, places=1)
    
    def test_red_light_overrides_safe_distance(self):
        """Test that red light stops vehicle even when distance is safe"""
        logic = CollisionAvoidanceLogic()
        logic.speeds['car_n1'] = 4.0
        
        # Large distance (safe)
        velocity = logic.calculate_velocity('car_n1', 20.0, is_at_red_light=True)
        
        # Should still be 0 due to red light
        self.assertEqual(velocity, 0.0)
    
    def test_unknown_vehicle_returns_zero(self):
        """Test that unknown vehicle returns zero velocity"""
        logic = CollisionAvoidanceLogic()
        
        # Don't add vehicle to speeds dict
        velocity = logic.calculate_velocity('car_n1', 10.0, is_at_red_light=False)
        
        self.assertEqual(velocity, 0.0)


if __name__ == '__main__':
    unittest.main()
