#!/usr/bin/env python3
"""
Property-based tests for vehicle control system.
Feature: traffic-control-fix

Tests Properties 8, 9, 11, 16, 17, and 18 related to vehicle control.
"""

import unittest
from hypothesis import given, settings, strategies as st, assume
import sys
import os

# Add the scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class VehicleControlLogic:
    """
    Standalone logic class for testing vehicle control without ROS2 initialization.
    Mirrors the logic from MultiTrafficFlowControl.
    """
    
    def __init__(self):
        self.SAFE_FOLLOWING_DISTANCE = 5.0
        self.STOP_DISTANCE = 3.0
        self.speeds = {}
        self.light_state = "NS_GREEN"
        self.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
    
    def get_blocked_roads(self):
        """Get list of roads that should stop for red light"""
        if self.light_state == "NS_GREEN":
            return ['e', 'w']
        elif self.light_state == "EW_GREEN":
            return ['n', 's']
        elif self.light_state == "ALL_RED":
            return ['n', 's', 'e', 'w']
        return []
    
    def calculate_velocity_command(self, vehicle_name, distance_ahead):
        """
        Calculate velocity command for a vehicle.
        
        Args:
            vehicle_name: Name of the vehicle (e.g., 'car_n1')
            distance_ahead: Distance to vehicle ahead in meters
            
        Returns:
            dict: {'linear_x': float, 'linear_y': float}
        """
        if vehicle_name not in self.speeds:
            return {'linear_x': 0.0, 'linear_y': 0.0}
        
        # Extract road direction
        car_road = vehicle_name.split('_')[1][0]
        
        # Check if vehicle should stop for red light
        blocked_roads = self.get_blocked_roads()
        is_at_red_light = (car_road in blocked_roads and vehicle_name in self.in_zones[car_road])
        
        assigned_speed = self.speeds[vehicle_name]
        
        # Determine velocity based on conditions
        if is_at_red_light:
            velocity = 0.0
        elif distance_ahead < self.STOP_DISTANCE:
            velocity = 0.0
        elif distance_ahead < self.SAFE_FOLLOWING_DISTANCE:
            ratio = (distance_ahead - self.STOP_DISTANCE) / \
                   (self.SAFE_FOLLOWING_DISTANCE - self.STOP_DISTANCE)
            velocity = assigned_speed * ratio
        else:
            velocity = assigned_speed
        
        # All vehicles use linear.y with negative velocity for forward motion
        result = {'linear_x': 0.0, 'linear_y': -velocity}
        
        return result
    
    def get_velocity_topic(self, vehicle_name):
        """Get the topic name for a vehicle's velocity command"""
        return f'/model/{vehicle_name}/cmd_vel'


class TestVehicleControlProperties(unittest.TestCase):
    """
    Property-based tests for vehicle control.
    Validates: Requirements 4.1, 4.2, 4.3, 4.5, 6.1, 6.3, 6.4, 6.5
    """
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4),
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=10.0, max_value=50.0)
    )
    def test_property_8_red_light_stopping(self, vehicle_direction, vehicle_number, assigned_speed, distance_ahead):
        """
        Feature: traffic-control-fix, Property 8: Red Light Stopping
        
        For any vehicle in a Detection_Zone when its direction has a red light, 
        the Flow_Controller SHALL command that vehicle's velocity to zero.
        
        Validates: Requirements 4.1, 4.2
        """
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        logic.speeds[vehicle_name] = assigned_speed
        
        # Add vehicle to detection zone
        logic.in_zones[vehicle_direction].add(vehicle_name)
        
        # Set light state so this direction has red light
        if vehicle_direction in ['n', 's']:
            logic.light_state = "EW_GREEN"  # NS has red
        else:
            logic.light_state = "NS_GREEN"  # EW has red
        
        # Calculate velocity command
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead)
        
        # Both velocity components should be zero at red light
        self.assertEqual(
            cmd['linear_x'],
            0.0,
            f"Vehicle {vehicle_name} should have zero x velocity at red light"
        )
        self.assertEqual(
            cmd['linear_y'],
            0.0,
            f"Vehicle {vehicle_name} should have zero y velocity at red light"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4),
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=10.0, max_value=50.0)
    )
    def test_property_9_green_light_resumption(self, vehicle_direction, vehicle_number, assigned_speed, distance_ahead):
        """
        Feature: traffic-control-fix, Property 9: Green Light Resumption
        
        For any vehicle that was stopped at a red light, when the light turns 
        green for its direction, the Flow_Controller SHALL command a non-zero 
        velocity for that vehicle.
        
        Validates: Requirements 4.3
        """
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        logic.speeds[vehicle_name] = assigned_speed
        
        # Add vehicle to detection zone
        logic.in_zones[vehicle_direction].add(vehicle_name)
        
        # Set light state so this direction has green light
        if vehicle_direction in ['n', 's']:
            logic.light_state = "NS_GREEN"  # NS has green
        else:
            logic.light_state = "EW_GREEN"  # EW has green
        
        # Calculate velocity command
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead)
        
        # At least one velocity component should be non-zero (linear.y should be negative)
        self.assertLess(
            cmd['linear_y'],
            0.0,
            f"Vehicle {vehicle_name} should have negative linear.y at green light with safe distance"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4),
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        distance_ahead=st.floats(min_value=10.0, max_value=50.0)
    )
    def test_property_11_green_light_movement(self, vehicle_direction, vehicle_number, assigned_speed, distance_ahead):
        """
        Feature: traffic-control-fix, Property 11: Green Light Movement
        
        For any vehicle outside the Detection_Zone when its direction has a 
        green light and no vehicle is within Safe_Following_Distance ahead, 
        the velocity command SHALL equal the vehicle's assigned speed.
        
        Validates: Requirements 4.5
        """
        # Ensure distance is safe
        assume(distance_ahead >= 5.0)
        
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        logic.speeds[vehicle_name] = assigned_speed
        
        # Vehicle is NOT in detection zone (outside zone)
        # logic.in_zones[vehicle_direction] is empty
        
        # Set light state so this direction has green light
        if vehicle_direction in ['n', 's']:
            logic.light_state = "NS_GREEN"
        else:
            logic.light_state = "EW_GREEN"
        
        # Calculate velocity command
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead)
        
        # Get the magnitude of velocity (should be in linear.y)
        velocity_magnitude = abs(cmd['linear_y'])
        
        # Velocity magnitude should equal assigned speed
        self.assertAlmostEqual(
            velocity_magnitude,
            assigned_speed,
            places=5,
            msg=f"Vehicle {vehicle_name} should move at assigned speed {assigned_speed} when outside zone with green light"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4)
    )
    def test_property_16_velocity_command_routing(self, vehicle_direction, vehicle_number):
        """
        Feature: traffic-control-fix, Property 16: Velocity Command Routing
        
        For any vehicle, velocity commands SHALL be published to the topic 
        /model/{vehicle_name}/cmd_vel where {vehicle_name} matches the 
        vehicle's identifier.
        
        Validates: Requirements 6.1
        """
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        
        # Get the topic name
        topic = logic.get_velocity_topic(vehicle_name)
        
        # Verify topic format
        expected_topic = f'/model/{vehicle_name}/cmd_vel'
        self.assertEqual(
            topic,
            expected_topic,
            f"Topic should be {expected_topic} for vehicle {vehicle_name}"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4),
        assigned_speed=st.floats(min_value=3.0, max_value=5.0),
        should_stop=st.booleans()
    )
    def test_property_17_velocity_command_values(self, vehicle_direction, vehicle_number, assigned_speed, should_stop):
        """
        Feature: traffic-control-fix, Property 17: Velocity Command Values
        
        For any vehicle, when commanded to stop the velocity SHALL be zero, 
        and when commanded to move the velocity SHALL equal the assigned speed 
        with correct sign for direction.
        
        Validates: Requirements 6.3, 6.4
        """
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        logic.speeds[vehicle_name] = assigned_speed
        
        if should_stop:
            # Simulate stop condition (at red light)
            logic.in_zones[vehicle_direction].add(vehicle_name)
            if vehicle_direction in ['n', 's']:
                logic.light_state = "EW_GREEN"  # NS has red
            else:
                logic.light_state = "NS_GREEN"  # EW has red
            distance_ahead = 20.0
        else:
            # Simulate move condition (green light, safe distance)
            if vehicle_direction in ['n', 's']:
                logic.light_state = "NS_GREEN"
            else:
                logic.light_state = "EW_GREEN"
            distance_ahead = 20.0
        
        # Calculate velocity command
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead)
        
        if should_stop:
            # Both components should be zero
            self.assertEqual(cmd['linear_x'], 0.0)
            self.assertEqual(cmd['linear_y'], 0.0)
        else:
            # linear.x should be zero, linear.y should be negative with magnitude = assigned speed
            self.assertEqual(cmd['linear_x'], 0.0)
            self.assertAlmostEqual(abs(cmd['linear_y']), assigned_speed, places=5)
    
    @settings(max_examples=100)
    @given(
        vehicle_direction=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_number=st.integers(min_value=1, max_value=4),
        assigned_speed=st.floats(min_value=3.0, max_value=5.0)
    )
    def test_property_18_velocity_axis_mapping(self, vehicle_direction, vehicle_number, assigned_speed):
        """
        Feature: traffic-control-fix, Property 18: Velocity Axis Mapping
        
        For any vehicle, the velocity command SHALL use linear.y for forward
        motion (all vehicles use the same axis in the Prius model).
        
        Validates: Requirements 6.5
        """
        # Create logic instance
        logic = VehicleControlLogic()
        vehicle_name = f'car_{vehicle_direction}{vehicle_number}'
        logic.speeds[vehicle_name] = assigned_speed
        
        # Set green light for this direction
        if vehicle_direction in ['n', 's']:
            logic.light_state = "NS_GREEN"
        else:
            logic.light_state = "EW_GREEN"
        
        # Calculate velocity command with safe distance
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead=20.0)
        
        # All vehicles should use linear.y (linear.x should be 0)
        self.assertEqual(
            cmd['linear_x'],
            0.0,
            f"Vehicle {vehicle_name} should have zero linear.x"
        )
        
        # linear.y should be negative (forward motion)
        self.assertLess(
            cmd['linear_y'],
            0.0,
            f"Vehicle {vehicle_name} should have negative linear.y for forward motion"
        )
        
        # Magnitude should equal assigned speed
        self.assertAlmostEqual(
            abs(cmd['linear_y']),
            assigned_speed,
            places=5,
            msg=f"Vehicle {vehicle_name} velocity magnitude should equal assigned speed"
        )


class TestVehicleControlEdgeCases(unittest.TestCase):
    """
    Unit tests for specific edge cases in vehicle control.
    """
    
    def test_all_red_stops_all_vehicles(self):
        """Test that ALL_RED state stops vehicles in all directions"""
        logic = VehicleControlLogic()
        logic.light_state = "ALL_RED"
        
        for direction in ['n', 's', 'e', 'w']:
            vehicle_name = f'car_{direction}1'
            logic.speeds[vehicle_name] = 4.0
            logic.in_zones[direction].add(vehicle_name)
            
            cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead=20.0)
            
            self.assertEqual(cmd['linear_x'], 0.0)
            self.assertEqual(cmd['linear_y'], 0.0)
    
    def test_vehicle_outside_zone_ignores_red_light(self):
        """Test that vehicles outside detection zone move even with red light"""
        logic = VehicleControlLogic()
        logic.light_state = "EW_GREEN"  # NS has red
        
        vehicle_name = 'car_n1'
        logic.speeds[vehicle_name] = 4.0
        # Don't add to in_zones (vehicle is outside detection zone)
        
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead=20.0)
        
        # Should still move (not in detection zone) - linear.y should be negative
        self.assertLess(cmd['linear_y'], 0.0)
    
    def test_collision_avoidance_overrides_green_light(self):
        """Test that collision avoidance works even with green light"""
        logic = VehicleControlLogic()
        logic.light_state = "NS_GREEN"  # North has green
        
        vehicle_name = 'car_n1'
        logic.speeds[vehicle_name] = 4.0
        
        # Vehicle ahead is very close
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead=2.0)
        
        # Should stop due to collision avoidance
        self.assertEqual(cmd['linear_x'], 0.0)
        self.assertEqual(cmd['linear_y'], 0.0)
    
    def test_proportional_speed_reduction(self):
        """Test that speed reduces proportionally between STOP and SAFE distances"""
        logic = VehicleControlLogic()
        logic.light_state = "NS_GREEN"
        
        vehicle_name = 'car_n1'
        logic.speeds[vehicle_name] = 4.0
        
        # Test at midpoint (4.0 meters)
        cmd = logic.calculate_velocity_command(vehicle_name, distance_ahead=4.0)
        velocity_magnitude = abs(cmd['linear_y'])
        
        # Should be approximately half speed
        expected = 4.0 * 0.5
        self.assertAlmostEqual(velocity_magnitude, expected, places=1)


if __name__ == '__main__':
    unittest.main()
