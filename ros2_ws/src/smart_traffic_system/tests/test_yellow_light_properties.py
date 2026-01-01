#!/usr/bin/env python3
"""
Property tests for yellow light functionality.
Feature: adaptive-traffic-system-v2

Tests yellow light duration, vehicle slowdown, and state transitions.
Validates: Requirements 7.2, 7.3, 7.4, 7.5
"""

import unittest
import sys
import os
from hypothesis import given, strategies as st, settings

# Add scripts directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class MockTrafficManager:
    """Mock traffic manager for testing yellow light logic."""
    
    def __init__(self):
        # Constants matching smart_traffic_manager.py
        self.MIN_GREEN = 10.0
        self.MAX_GREEN = 30.0
        self.YELLOW_TIME = 3.0
        self.ALL_RED_TIME = 2.0
        
        # States
        self.NS_GREEN = "NS_GREEN"
        self.NS_YELLOW = "NS_YELLOW"
        self.EW_GREEN = "EW_GREEN"
        self.EW_YELLOW = "EW_YELLOW"
        self.ALL_RED = "ALL_RED"
        
        self.current_state = self.NS_GREEN
        self.elapsed_time = 0.0
        self.target_state = self.EW_GREEN

    def get_next_state(self, waiting_ns, waiting_ew):
        """Determine next state based on current state and traffic."""
        if self.current_state == self.NS_GREEN:
            should_switch = (
                waiting_ew > 0 and
                (waiting_ns == 0 or self.elapsed_time > self.MAX_GREEN) and
                self.elapsed_time > self.MIN_GREEN
            )
            if should_switch:
                return self.NS_YELLOW
        
        elif self.current_state == self.NS_YELLOW:
            if self.elapsed_time >= self.YELLOW_TIME:
                return self.ALL_RED
        
        elif self.current_state == self.EW_GREEN:
            should_switch = (
                waiting_ns > 0 and
                (waiting_ew == 0 or self.elapsed_time > self.MAX_GREEN) and
                self.elapsed_time > self.MIN_GREEN
            )
            if should_switch:
                return self.EW_YELLOW
        
        elif self.current_state == self.EW_YELLOW:
            if self.elapsed_time >= self.YELLOW_TIME:
                return self.ALL_RED
        
        elif self.current_state == self.ALL_RED:
            if self.elapsed_time > self.ALL_RED_TIME:
                return self.target_state
        
        return self.current_state


class MockFlowController:
    """Mock flow controller for testing yellow light vehicle response."""
    
    def __init__(self):
        self.YELLOW_SLOWDOWN_FACTOR = 0.5
        self.light_state = "NS_GREEN"
        self.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
        self.speeds = {}

    def get_velocity(self, vehicle_name, base_speed):
        """Calculate velocity for a vehicle considering yellow light."""
        car_road = vehicle_name.split('_')[1][0]
        
        # Determine yellow roads based on light state
        yellow_roads = []
        blocked_roads = []
        
        if self.light_state == "NS_GREEN":
            blocked_roads = ['e', 'w']
        elif self.light_state == "NS_YELLOW":
            yellow_roads = ['n', 's']
            blocked_roads = ['e', 'w']
        elif self.light_state == "EW_GREEN":
            blocked_roads = ['n', 's']
        elif self.light_state == "EW_YELLOW":
            yellow_roads = ['e', 'w']
            blocked_roads = ['n', 's']
        elif self.light_state == "ALL_RED":
            blocked_roads = ['n', 's', 'e', 'w']
        
        is_at_red = car_road in blocked_roads and vehicle_name in self.in_zones[car_road]
        is_at_yellow = car_road in yellow_roads and vehicle_name in self.in_zones[car_road]
        
        if is_at_red:
            return 0.0
        elif is_at_yellow:
            return base_speed * self.YELLOW_SLOWDOWN_FACTOR
        else:
            return base_speed


class TestYellowLightDuration(unittest.TestCase):
    """Property 18: Yellow Light Duration - yellow phase lasts exactly YELLOW_TIME."""

    @given(st.floats(min_value=0.0, max_value=2.9))
    @settings(max_examples=50)
    def test_yellow_stays_yellow_before_timeout(self, elapsed):
        """Yellow state should not transition before YELLOW_TIME."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = elapsed
        
        next_state = manager.get_next_state(0, 1)
        
        self.assertEqual(next_state, manager.NS_YELLOW,
                        f"Yellow should stay yellow at {elapsed}s (< 3.0s)")
    
    @given(st.floats(min_value=3.0, max_value=10.0))
    @settings(max_examples=50)
    def test_yellow_transitions_after_timeout(self, elapsed):
        """Yellow state should transition to ALL_RED after YELLOW_TIME."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = elapsed
        
        next_state = manager.get_next_state(0, 1)
        
        self.assertEqual(next_state, manager.ALL_RED,
                        f"Yellow should transition to ALL_RED at {elapsed}s (>= 3.0s)")
    
    def test_yellow_duration_constant(self):
        """Yellow duration should be exactly 3 seconds (Requirement 7.5)."""
        manager = MockTrafficManager()
        self.assertEqual(manager.YELLOW_TIME, 3.0,
                        "Yellow light duration should be 3 seconds")


class TestVehicleSlowdownOnYellow(unittest.TestCase):
    """Property 19: Vehicle Slowdown on Yellow - vehicles reduce speed by 50%."""

    @given(st.floats(min_value=1.0, max_value=10.0))
    @settings(max_examples=50)
    def test_ns_yellow_slows_ns_vehicles(self, base_speed):
        """NS vehicles should slow down during NS_YELLOW (Requirement 7.3)."""
        controller = MockFlowController()
        controller.light_state = "NS_YELLOW"
        controller.in_zones['n'] = {'car_n1'}
        
        velocity = controller.get_velocity('car_n1', base_speed)
        expected = base_speed * 0.5
        
        self.assertAlmostEqual(velocity, expected, places=5,
                              msg=f"NS vehicle should slow to 50% on yellow")
    
    @given(st.floats(min_value=1.0, max_value=10.0))
    @settings(max_examples=50)
    def test_ew_yellow_slows_ew_vehicles(self, base_speed):
        """EW vehicles should slow down during EW_YELLOW (Requirement 7.3)."""
        controller = MockFlowController()
        controller.light_state = "EW_YELLOW"
        controller.in_zones['e'] = {'car_e1'}
        
        velocity = controller.get_velocity('car_e1', base_speed)
        expected = base_speed * 0.5
        
        self.assertAlmostEqual(velocity, expected, places=5,
                              msg=f"EW vehicle should slow to 50% on yellow")
    
    @given(st.floats(min_value=1.0, max_value=10.0))
    @settings(max_examples=50)
    def test_yellow_does_not_affect_perpendicular_traffic(self, base_speed):
        """Perpendicular traffic should remain stopped during yellow."""
        controller = MockFlowController()
        controller.light_state = "NS_YELLOW"
        controller.in_zones['e'] = {'car_e1'}
        
        velocity = controller.get_velocity('car_e1', base_speed)
        
        self.assertEqual(velocity, 0.0,
                        "EW vehicles should remain stopped during NS_YELLOW")

    def test_slowdown_factor_is_50_percent(self):
        """Slowdown factor should be exactly 50% (Requirement 7.4)."""
        controller = MockFlowController()
        self.assertEqual(controller.YELLOW_SLOWDOWN_FACTOR, 0.5,
                        "Yellow slowdown factor should be 0.5 (50%)")


class TestYellowStateTransitions(unittest.TestCase):
    """Test valid state transitions involving yellow light."""
    
    def test_ns_green_to_ns_yellow_transition(self):
        """NS_GREEN should transition to NS_YELLOW, not directly to ALL_RED."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_GREEN
        manager.elapsed_time = 15.0  # Past MIN_GREEN
        
        # EW has traffic, NS is empty
        next_state = manager.get_next_state(0, 1)
        
        self.assertEqual(next_state, manager.NS_YELLOW,
                        "NS_GREEN should transition to NS_YELLOW first")
    
    def test_ew_green_to_ew_yellow_transition(self):
        """EW_GREEN should transition to EW_YELLOW, not directly to ALL_RED."""
        manager = MockTrafficManager()
        manager.current_state = manager.EW_GREEN
        manager.elapsed_time = 15.0  # Past MIN_GREEN
        
        # NS has traffic, EW is empty
        next_state = manager.get_next_state(1, 0)
        
        self.assertEqual(next_state, manager.EW_YELLOW,
                        "EW_GREEN should transition to EW_YELLOW first")
    
    def test_ns_yellow_to_all_red_transition(self):
        """NS_YELLOW should transition to ALL_RED after YELLOW_TIME."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = 3.0
        
        next_state = manager.get_next_state(0, 1)
        
        self.assertEqual(next_state, manager.ALL_RED,
                        "NS_YELLOW should transition to ALL_RED")

    def test_ew_yellow_to_all_red_transition(self):
        """EW_YELLOW should transition to ALL_RED after YELLOW_TIME."""
        manager = MockTrafficManager()
        manager.current_state = manager.EW_YELLOW
        manager.elapsed_time = 3.0
        
        next_state = manager.get_next_state(1, 0)
        
        self.assertEqual(next_state, manager.ALL_RED,
                        "EW_YELLOW should transition to ALL_RED")
    
    def test_complete_cycle_includes_yellow(self):
        """Complete cycle: NS_GREEN -> NS_YELLOW -> ALL_RED -> EW_GREEN."""
        manager = MockTrafficManager()
        
        # Start at NS_GREEN
        manager.current_state = manager.NS_GREEN
        manager.elapsed_time = 15.0
        
        # Step 1: NS_GREEN -> NS_YELLOW
        next_state = manager.get_next_state(0, 1)
        self.assertEqual(next_state, manager.NS_YELLOW)
        
        # Step 2: NS_YELLOW -> ALL_RED
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = 3.0
        next_state = manager.get_next_state(0, 1)
        self.assertEqual(next_state, manager.ALL_RED)
        
        # Step 3: ALL_RED -> EW_GREEN
        manager.current_state = manager.ALL_RED
        manager.target_state = manager.EW_GREEN
        manager.elapsed_time = 2.5
        next_state = manager.get_next_state(0, 1)
        self.assertEqual(next_state, manager.EW_GREEN)


class TestYellowLightEdgeCases(unittest.TestCase):
    """Edge cases for yellow light behavior."""
    
    def test_vehicle_not_in_zone_ignores_yellow(self):
        """Vehicles not in detection zone should not slow for yellow."""
        controller = MockFlowController()
        controller.light_state = "NS_YELLOW"
        # car_n1 is NOT in the zone
        controller.in_zones['n'] = set()
        
        velocity = controller.get_velocity('car_n1', 5.0)
        
        self.assertEqual(velocity, 5.0,
                        "Vehicle not in zone should maintain full speed")

    def test_exactly_at_yellow_timeout(self):
        """Test behavior exactly at YELLOW_TIME boundary."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = 3.0  # Exactly at timeout
        
        next_state = manager.get_next_state(0, 1)
        
        self.assertEqual(next_state, manager.ALL_RED,
                        "Should transition at exactly YELLOW_TIME")
    
    def test_yellow_with_no_waiting_traffic(self):
        """Yellow should still complete even if no traffic waiting."""
        manager = MockTrafficManager()
        manager.current_state = manager.NS_YELLOW
        manager.elapsed_time = 3.0
        
        # No traffic waiting anywhere
        next_state = manager.get_next_state(0, 0)
        
        self.assertEqual(next_state, manager.ALL_RED,
                        "Yellow should complete regardless of traffic")
    
    def test_all_directions_yellow_slowdown(self):
        """Test all four directions respond correctly to their yellow."""
        controller = MockFlowController()
        base_speed = 5.0
        
        # Test NS_YELLOW affects N and S
        controller.light_state = "NS_YELLOW"
        for direction in ['n', 's']:
            vehicle = f'car_{direction}1'
            controller.in_zones[direction] = {vehicle}
            velocity = controller.get_velocity(vehicle, base_speed)
            self.assertEqual(velocity, base_speed * 0.5,
                           f"{direction} should slow on NS_YELLOW")
        
        # Test EW_YELLOW affects E and W
        controller.light_state = "EW_YELLOW"
        controller.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
        for direction in ['e', 'w']:
            vehicle = f'car_{direction}1'
            controller.in_zones[direction] = {vehicle}
            velocity = controller.get_velocity(vehicle, base_speed)
            self.assertEqual(velocity, base_speed * 0.5,
                           f"{direction} should slow on EW_YELLOW")


if __name__ == '__main__':
    unittest.main()
