#!/usr/bin/env python3
"""
Property-based tests for error handling system.
Feature: traffic-control-fix

Tests Properties 21, 22, 23, and 24 related to error handling.
"""

import unittest
from hypothesis import given, settings, strategies as st
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class ErrorHandlingLogic:
    """Standalone logic for testing error handling without ROS2."""
    
    def __init__(self):
        self.SENSOR_TIMEOUT = 1.0
        self.counts = {'n': 0, 's': 0, 'e': 0, 'w': 0}
        self.last_sensor_update = {'n': 0.0, 's': 0.0, 'e': 0.0, 'w': 0.0}
        self.sensor_failed = {'n': False, 's': False, 'e': False, 'w': False}
        self.state_transitions = []
        self.control_decisions = []
    
    def validate_sensor_data(self, msg):
        """Validate sensor data (Requirement 10.3)"""
        try:
            if not hasattr(msg, 'models'):
                return False
            _ = len(msg.models)
            for model in msg.models:
                if not hasattr(model, 'name'):
                    return False
            return True
        except:
            return False
    
    def process_sensor_data(self, msg, dir_key, current_time):
        """Process sensor data with validation"""
        if not self.validate_sensor_data(msg):
            return False
        
        self.counts[dir_key] = len(msg.models)
        self.last_sensor_update[dir_key] = current_time
        if self.sensor_failed[dir_key]:
            self.sensor_failed[dir_key] = False
        return True
    
    def check_sensor_failures(self, current_time):
        """Check for sensor failures (Requirement 10.1)"""
        failures = []
        for dir_key in ['n', 's', 'e', 'w']:
            elapsed = current_time - self.last_sensor_update[dir_key]
            if elapsed > self.SENSOR_TIMEOUT and not self.sensor_failed[dir_key]:
                self.sensor_failed[dir_key] = True
                failures.append(dir_key)
        return failures
    
    def get_count(self, dir_key):
        """Get count, using last known if sensor failed"""
        return self.counts[dir_key]
    
    def log_state_transition(self, prev_state, new_state, timestamp):
        """Log state transition (Requirement 10.4)"""
        self.state_transitions.append({
            'previous': prev_state,
            'new': new_state,
            'timestamp': timestamp
        })
    
    def log_control_decision(self, vehicle_name, velocity, reason):
        """Log control decision (Requirement 10.5)"""
        self.control_decisions.append({
            'vehicle': vehicle_name,
            'velocity': velocity,
            'reason': reason
        })


class MockModel:
    def __init__(self, name):
        self.name = name

class MockMessage:
    def __init__(self, models=None):
        self.models = models if models is not None else []

class InvalidMessage:
    pass


class TestErrorHandlingProperties(unittest.TestCase):
    """Property-based tests for error handling. Validates: Requirements 10.1, 10.3, 10.4, 10.5"""
    
    @settings(max_examples=100)
    @given(
        initial_count=st.integers(min_value=0, max_value=8),
        timeout_elapsed=st.floats(min_value=1.1, max_value=5.0)
    )
    def test_property_21_sensor_failure_handling(self, initial_count, timeout_elapsed):
        """
        Feature: traffic-control-fix, Property 21: Sensor Failure Handling
        
        For any sensor that fails to provide data for a timeout period, the 
        Traffic_Manager SHALL maintain the last known vehicle count.
        
        Validates: Requirements 10.1
        """
        logic = ErrorHandlingLogic()
        
        # Set initial count
        msg = MockMessage([MockModel(f'car_n{i}') for i in range(initial_count)])
        logic.process_sensor_data(msg, 'n', 0.0)
        
        # Simulate timeout
        failures = logic.check_sensor_failures(timeout_elapsed)
        
        # Sensor should be marked as failed
        self.assertIn('n', failures)
        self.assertTrue(logic.sensor_failed['n'])
        
        # Count should be preserved
        self.assertEqual(logic.get_count('n'), initial_count)
    
    @settings(max_examples=100)
    @given(valid_count=st.integers(min_value=0, max_value=8))
    def test_property_22_invalid_data_handling(self, valid_count):
        """
        Feature: traffic-control-fix, Property 22: Invalid Data Handling
        
        For any sensor data message that fails validation checks, the system 
        SHALL ignore the invalid data.
        
        Validates: Requirements 10.3
        """
        logic = ErrorHandlingLogic()
        
        # Set initial valid count
        valid_msg = MockMessage([MockModel(f'car_n{i}') for i in range(valid_count)])
        logic.process_sensor_data(valid_msg, 'n', 0.0)
        
        # Try to process invalid message
        invalid_msg = InvalidMessage()
        result = logic.process_sensor_data(invalid_msg, 'n', 1.0)
        
        # Should return False and preserve original count
        self.assertFalse(result)
        self.assertEqual(logic.get_count('n'), valid_count)
    
    @settings(max_examples=100)
    @given(
        prev_state=st.sampled_from(['NS_GREEN', 'EW_GREEN', 'ALL_RED']),
        new_state=st.sampled_from(['NS_GREEN', 'EW_GREEN', 'ALL_RED']),
        timestamp=st.floats(min_value=0.0, max_value=1000.0)
    )
    def test_property_23_state_transition_logging(self, prev_state, new_state, timestamp):
        """
        Feature: traffic-control-fix, Property 23: State Transition Logging
        
        For any traffic light state transition, the Traffic_Manager SHALL 
        create a log entry containing the previous state, new state, and timestamp.
        
        Validates: Requirements 10.4
        """
        logic = ErrorHandlingLogic()
        
        # Log transition
        logic.log_state_transition(prev_state, new_state, timestamp)
        
        # Verify log entry
        self.assertEqual(len(logic.state_transitions), 1)
        entry = logic.state_transitions[0]
        self.assertEqual(entry['previous'], prev_state)
        self.assertEqual(entry['new'], new_state)
        self.assertEqual(entry['timestamp'], timestamp)
    
    @settings(max_examples=100)
    @given(
        vehicle_dir=st.sampled_from(['n', 's', 'e', 'w']),
        vehicle_num=st.integers(min_value=1, max_value=4),
        velocity=st.floats(min_value=0.0, max_value=5.0),
        reason=st.sampled_from(['red_light', 'emergency_stop', 'collision_avoidance', 'normal'])
    )
    def test_property_24_control_decision_logging(self, vehicle_dir, vehicle_num, velocity, reason):
        """
        Feature: traffic-control-fix, Property 24: Control Decision Logging
        
        For any vehicle control decision that changes a vehicle's velocity, 
        the Flow_Controller SHALL create a log entry.
        
        Validates: Requirements 10.5
        """
        logic = ErrorHandlingLogic()
        vehicle_name = f'car_{vehicle_dir}{vehicle_num}'
        
        # Log decision
        logic.log_control_decision(vehicle_name, velocity, reason)
        
        # Verify log entry
        self.assertEqual(len(logic.control_decisions), 1)
        entry = logic.control_decisions[0]
        self.assertEqual(entry['vehicle'], vehicle_name)
        self.assertEqual(entry['velocity'], velocity)
        self.assertEqual(entry['reason'], reason)


class TestErrorHandlingEdgeCases(unittest.TestCase):
    """Unit tests for error handling edge cases."""
    
    def test_sensor_recovery_after_failure(self):
        """Test that sensor failure flag clears on recovery"""
        logic = ErrorHandlingLogic()
        
        # Initial data
        msg = MockMessage([MockModel('car_n1')])
        logic.process_sensor_data(msg, 'n', 0.0)
        
        # Trigger failure
        logic.check_sensor_failures(2.0)
        self.assertTrue(logic.sensor_failed['n'])
        
        # Recovery
        logic.process_sensor_data(msg, 'n', 3.0)
        self.assertFalse(logic.sensor_failed['n'])
    
    def test_multiple_sensor_failures(self):
        """Test handling of multiple simultaneous sensor failures"""
        logic = ErrorHandlingLogic()
        
        # Set initial data for all sensors
        for d in ['n', 's', 'e', 'w']:
            msg = MockMessage([MockModel(f'car_{d}1')])
            logic.process_sensor_data(msg, d, 0.0)
        
        # All sensors timeout
        failures = logic.check_sensor_failures(2.0)
        
        self.assertEqual(len(failures), 4)
        for d in ['n', 's', 'e', 'w']:
            self.assertTrue(logic.sensor_failed[d])
            self.assertEqual(logic.get_count(d), 1)
    
    def test_validation_rejects_missing_models(self):
        """Test that validation rejects messages without models attribute"""
        logic = ErrorHandlingLogic()
        
        class NoModels:
            pass
        
        result = logic.validate_sensor_data(NoModels())
        self.assertFalse(result)
    
    def test_validation_rejects_invalid_model(self):
        """Test that validation rejects models without name attribute"""
        logic = ErrorHandlingLogic()
        
        class BadModel:
            pass
        
        class BadMessage:
            models = [BadModel()]
        
        result = logic.validate_sensor_data(BadMessage())
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
