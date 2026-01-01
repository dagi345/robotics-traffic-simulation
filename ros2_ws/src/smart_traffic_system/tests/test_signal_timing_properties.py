#!/usr/bin/env python3
"""
Property-based tests for adaptive signal timing system.
Feature: traffic-control-fix

Tests Properties 5, 6, and 7 related to signal timing.
"""

import unittest
from hypothesis import given, settings, strategies as st, assume
import sys
import os

# Add the scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class SignalTimingLogic:
    """
    Standalone logic class for testing signal timing without ROS2 initialization.
    Mirrors the logic from SmartTrafficManager.
    """
    
    def __init__(self):
        self.MIN_GREEN = 10.0  # Seconds
        self.MAX_GREEN = 30.0
        self.ALL_RED_TIME = 2.0
        
        self.NS_GREEN = "NS_GREEN"
        self.EW_GREEN = "EW_GREEN"
        self.ALL_RED = "ALL_RED"
        
        self.current_state = self.NS_GREEN
        self.last_switch_time = 0.0
        self.target_state = None
        
        self.counts = {'n': 0, 's': 0, 'e': 0, 'w': 0}
    
    def update_counts(self, n, s, e, w):
        """Update vehicle counts for each direction"""
        self.counts = {'n': n, 's': s, 'e': e, 'w': w}
    
    def should_switch_state(self, elapsed_time):
        """
        Determine if state should switch based on current conditions.
        
        Args:
            elapsed_time: Time elapsed since last state change
            
        Returns:
            tuple: (should_switch, next_state, target_state)
        """
        waiting_ns = self.counts['n'] + self.counts['s']
        waiting_ew = self.counts['e'] + self.counts['w']
        
        next_state = self.current_state
        target = self.target_state
        
        if self.current_state == self.NS_GREEN:
            # Switch if EW has demand and (NS is empty or max time reached) and min time passed
            if (waiting_ew > 0 and (waiting_ns == 0 or elapsed_time > self.MAX_GREEN)) and elapsed_time > self.MIN_GREEN:
                next_state = self.ALL_RED
                target = self.EW_GREEN
        
        elif self.current_state == self.EW_GREEN:
            # Switch if NS has demand and (EW is empty or max time reached) and min time passed
            if (waiting_ns > 0 and (waiting_ew == 0 or elapsed_time > self.MAX_GREEN)) and elapsed_time > self.MIN_GREEN:
                next_state = self.ALL_RED
                target = self.NS_GREEN
        
        elif self.current_state == self.ALL_RED:
            if elapsed_time > self.ALL_RED_TIME:
                next_state = target
        
        should_switch = (next_state != self.current_state)
        return should_switch, next_state, target
    
    def transition_to(self, new_state, target=None):
        """Transition to a new state"""
        self.current_state = new_state
        if target is not None:
            self.target_state = target
        self.last_switch_time = 0.0


class TestSignalTimingProperties(unittest.TestCase):
    """
    Property-based tests for adaptive signal timing.
    Validates: Requirements 3.1, 3.2, 3.3, 3.4, 3.5
    """
    
    @settings(max_examples=100)
    @given(
        waiting_ew=st.integers(min_value=1, max_value=8),
        elapsed_time=st.floats(min_value=10.1, max_value=29.9)
    )
    def test_property_5_adaptive_signal_switching(self, waiting_ew, elapsed_time):
        """
        Feature: traffic-control-fix, Property 5: Adaptive Signal Switching
        
        For any traffic state where one direction has waiting vehicles and the 
        perpendicular direction is empty, after the minimum green time has 
        elapsed, the Traffic_Manager SHALL initiate a transition to give the 
        waiting direction a green signal.
        
        Validates: Requirements 3.1, 3.2
        """
        # Set up scenario: NS is empty (0 vehicles), EW has waiting vehicles
        waiting_ns = 0
        
        # Create logic instance in NS_GREEN state
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(0, 0, waiting_ew // 2, waiting_ew - waiting_ew // 2)
        
        # Check if should switch
        should_switch, next_state, target = logic.should_switch_state(elapsed_time)
        
        # Should initiate transition to ALL_RED (first step toward EW_GREEN)
        self.assertTrue(
            should_switch,
            f"Should switch when NS empty ({waiting_ns}), EW waiting ({waiting_ew}), elapsed {elapsed_time}s"
        )
        self.assertEqual(
            next_state,
            logic.ALL_RED,
            "Should transition to ALL_RED first"
        )
        self.assertEqual(
            target,
            logic.EW_GREEN,
            "Target should be EW_GREEN"
        )
    
    @settings(max_examples=100)
    @given(
        elapsed_time=st.floats(min_value=0.1, max_value=2.5)
    )
    def test_property_6_all_red_transition(self, elapsed_time):
        """
        Feature: traffic-control-fix, Property 6: ALL_RED Transition
        
        For any transition from NS_GREEN to EW_GREEN or from EW_GREEN to 
        NS_GREEN, the system SHALL pass through the ALL_RED state for at 
        least the configured safety interval duration.
        
        Validates: Requirements 3.3
        """
        # Create logic instance in ALL_RED state
        logic = SignalTimingLogic()
        logic.current_state = logic.ALL_RED
        logic.target_state = logic.EW_GREEN
        logic.update_counts(0, 0, 2, 2)
        
        # Check if should switch
        should_switch, next_state, _ = logic.should_switch_state(elapsed_time)
        
        if elapsed_time <= logic.ALL_RED_TIME:
            # Should NOT switch before ALL_RED_TIME
            self.assertFalse(
                should_switch,
                f"Should NOT switch from ALL_RED before {logic.ALL_RED_TIME}s (elapsed: {elapsed_time}s)"
            )
        else:
            # Should switch after ALL_RED_TIME
            self.assertTrue(
                should_switch,
                f"Should switch from ALL_RED after {logic.ALL_RED_TIME}s (elapsed: {elapsed_time}s)"
            )
            self.assertEqual(
                next_state,
                logic.EW_GREEN,
                "Should transition to target state after ALL_RED"
            )
    
    @settings(max_examples=100)
    @given(
        waiting_ns=st.integers(min_value=1, max_value=8),
        waiting_ew=st.integers(min_value=1, max_value=8),
        elapsed_time=st.floats(min_value=0.1, max_value=40.0)
    )
    def test_property_7_green_time_bounds(self, waiting_ns, waiting_ew, elapsed_time):
        """
        Feature: traffic-control-fix, Property 7: Green Time Bounds
        
        For any green state duration, it SHALL be greater than or equal to 
        the minimum green time and less than or equal to the maximum green 
        time (unless no perpendicular traffic exists).
        
        Validates: Requirements 3.4, 3.5
        """
        # Create logic instance in NS_GREEN state
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(waiting_ns // 2, waiting_ns - waiting_ns // 2, 
                           waiting_ew // 2, waiting_ew - waiting_ew // 2)
        
        # Check if should switch
        should_switch, _, _ = logic.should_switch_state(elapsed_time)
        
        if elapsed_time < logic.MIN_GREEN:
            # Should NOT switch before MIN_GREEN
            self.assertFalse(
                should_switch,
                f"Should NOT switch before MIN_GREEN ({logic.MIN_GREEN}s), elapsed: {elapsed_time}s"
            )
        
        if elapsed_time > logic.MAX_GREEN and waiting_ew > 0:
            # Should switch after MAX_GREEN if perpendicular traffic exists
            self.assertTrue(
                should_switch,
                f"Should switch after MAX_GREEN ({logic.MAX_GREEN}s) with perpendicular traffic, elapsed: {elapsed_time}s"
            )


class TestSignalTimingEdgeCases(unittest.TestCase):
    """
    Unit tests for specific edge cases in signal timing.
    """
    
    def test_exactly_at_min_green_time(self):
        """Test behavior at exactly MIN_GREEN time"""
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(0, 0, 2, 2)  # NS empty, EW waiting
        
        # At exactly MIN_GREEN, should not switch (needs to be > MIN_GREEN)
        should_switch, _, _ = logic.should_switch_state(10.0)
        self.assertFalse(should_switch)
        
        # Just after MIN_GREEN, should switch
        should_switch, next_state, target = logic.should_switch_state(10.1)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.ALL_RED)
        self.assertEqual(target, logic.EW_GREEN)
    
    def test_exactly_at_max_green_time(self):
        """Test behavior at exactly MAX_GREEN time"""
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(2, 2, 2, 2)  # Both directions have traffic
        
        # At exactly MAX_GREEN, should not switch (needs to be > MAX_GREEN)
        should_switch, _, _ = logic.should_switch_state(30.0)
        self.assertFalse(should_switch)
        
        # Just after MAX_GREEN, should switch
        should_switch, next_state, target = logic.should_switch_state(30.1)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.ALL_RED)
        self.assertEqual(target, logic.EW_GREEN)
    
    def test_exactly_at_all_red_time(self):
        """Test behavior at exactly ALL_RED_TIME"""
        logic = SignalTimingLogic()
        logic.current_state = logic.ALL_RED
        logic.target_state = logic.EW_GREEN
        logic.update_counts(0, 0, 2, 2)
        
        # At exactly ALL_RED_TIME, should not switch (needs to be > ALL_RED_TIME)
        should_switch, _, _ = logic.should_switch_state(2.0)
        self.assertFalse(should_switch)
        
        # Just after ALL_RED_TIME, should switch
        should_switch, next_state, _ = logic.should_switch_state(2.1)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.EW_GREEN)
    
    def test_no_perpendicular_traffic_extends_green(self):
        """Test that green can extend beyond MAX_GREEN if no perpendicular traffic"""
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(2, 2, 0, 0)  # NS has traffic, EW empty
        
        # Even after MAX_GREEN, should not switch if no perpendicular traffic
        should_switch, _, _ = logic.should_switch_state(35.0)
        self.assertFalse(should_switch)
    
    def test_both_directions_empty(self):
        """Test behavior when both directions are empty"""
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(0, 0, 0, 0)  # Both empty
        
        # Should not switch even after MIN_GREEN if no perpendicular traffic
        should_switch, _, _ = logic.should_switch_state(15.0)
        self.assertFalse(should_switch)
    
    def test_transition_sequence_ns_to_ew(self):
        """Test complete transition sequence from NS_GREEN to EW_GREEN"""
        logic = SignalTimingLogic()
        logic.current_state = logic.NS_GREEN
        logic.update_counts(0, 0, 2, 2)
        
        # Step 1: NS_GREEN -> ALL_RED
        should_switch, next_state, target = logic.should_switch_state(11.0)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.ALL_RED)
        self.assertEqual(target, logic.EW_GREEN)
        
        # Transition
        logic.transition_to(next_state, target)
        
        # Step 2: ALL_RED -> EW_GREEN
        should_switch, next_state, _ = logic.should_switch_state(2.5)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.EW_GREEN)
    
    def test_transition_sequence_ew_to_ns(self):
        """Test complete transition sequence from EW_GREEN to NS_GREEN"""
        logic = SignalTimingLogic()
        logic.current_state = logic.EW_GREEN
        logic.update_counts(2, 2, 0, 0)
        
        # Step 1: EW_GREEN -> ALL_RED
        should_switch, next_state, target = logic.should_switch_state(11.0)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.ALL_RED)
        self.assertEqual(target, logic.NS_GREEN)
        
        # Transition
        logic.transition_to(next_state, target)
        
        # Step 2: ALL_RED -> NS_GREEN
        should_switch, next_state, _ = logic.should_switch_state(2.5)
        self.assertTrue(should_switch)
        self.assertEqual(next_state, logic.NS_GREEN)


if __name__ == '__main__':
    unittest.main()
