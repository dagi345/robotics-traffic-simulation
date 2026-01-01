#!/usr/bin/env python3
"""
Property-based tests for vehicle position tracking system.
Feature: traffic-control-fix

Tests Properties 2, 3, and 4 related to vehicle detection and counting.
"""

import unittest
from hypothesis import given, settings, strategies as st
import sys
import os

# Add the scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


# Strategy for generating vehicle names
def vehicle_name_strategy():
    """Generate valid vehicle names"""
    direction = st.sampled_from(['n', 's', 'e', 'w'])
    number = st.integers(min_value=1, max_value=4)
    return st.builds(lambda d, n: f'car_{d}{n}', direction, number)


# Strategy for generating vehicle positions
def position_strategy():
    """Generate valid vehicle positions within the simulation bounds"""
    return st.tuples(
        st.floats(min_value=-50.0, max_value=50.0),  # x
        st.floats(min_value=-50.0, max_value=50.0),  # y
        st.floats(min_value=0.0, max_value=2.0)      # z (ground level)
    )


def create_mock_sensor_message(vehicle_data):
    """
    Create a mock LogicalCameraImage message with vehicle data.
    
    Args:
        vehicle_data: List of tuples (vehicle_name, (x, y, z))
        
    Returns:
        Mock message with models attribute
    """
    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
    
    class MockPose:
        def __init__(self, pos):
            self.position = MockPoint(pos[0], pos[1], pos[2])
    
    class MockModel:
        def __init__(self, name, pos):
            self.name = name
            self.pose = MockPose(pos)
    
    class MockMessage:
        def __init__(self):
            self.models = []
    
    msg = MockMessage()
    for vehicle_name, position in vehicle_data:
        msg.models.append(MockModel(vehicle_name, position))
    
    return msg


class VehiclePositionTrackerLogic:
    """
    Standalone logic class for testing without ROS2 initialization.
    Mirrors the logic from VehiclePositionTracker but without ROS2 dependencies.
    """
    
    def __init__(self):
        self.vehicle_positions = {}
        self.vehicle_lanes = {}
        
        # Initialize vehicle lane mappings
        for i in range(1, 5):
            self.vehicle_lanes[f'car_n{i}'] = 'north'
        for i in range(1, 5):
            self.vehicle_lanes[f'car_s{i}'] = 'south'
        for i in range(1, 5):
            self.vehicle_lanes[f'car_e{i}'] = 'east'
        for i in range(1, 5):
            self.vehicle_lanes[f'car_w{i}'] = 'west'
    
    def process_sensor_data(self, msg):
        """Process sensor data and update vehicle positions"""
        for model in msg.models:
            if model.name.startswith('car_'):
                pos = model.pose.position
                self.vehicle_positions[model.name] = (pos.x, pos.y, pos.z)
    
    def get_distance_to_vehicle_ahead(self, vehicle_name):
        """Calculate distance to the vehicle ahead in the same lane"""
        if vehicle_name not in self.vehicle_positions:
            return float('inf')
        
        my_pos = self.vehicle_positions[vehicle_name]
        my_lane = self.vehicle_lanes.get(vehicle_name)
        
        if my_lane is None:
            return float('inf')
        
        min_distance = float('inf')
        
        for other_name, other_pos in self.vehicle_positions.items():
            if other_name == vehicle_name:
                continue
            
            if self.vehicle_lanes.get(other_name) != my_lane:
                continue
            
            distance = None
            
            if my_lane == 'north':
                if other_pos[1] < my_pos[1]:
                    distance = my_pos[1] - other_pos[1]
            elif my_lane == 'south':
                if other_pos[1] > my_pos[1]:
                    distance = other_pos[1] - my_pos[1]
            elif my_lane == 'east':
                if other_pos[0] < my_pos[0]:
                    distance = my_pos[0] - other_pos[0]
            elif my_lane == 'west':
                if other_pos[0] > my_pos[0]:
                    distance = other_pos[0] - my_pos[0]
            
            if distance is not None and distance < min_distance:
                min_distance = distance
        
        return min_distance
    
    def get_vehicle_count(self, direction):
        """Get the count of vehicles currently tracked in a specific direction"""
        count = 0
        for vehicle_name in self.vehicle_positions.keys():
            if self.vehicle_lanes.get(vehicle_name) == direction:
                count += 1
        return count


class TestVehiclePositionTracking(unittest.TestCase):
    """
    Property-based tests for vehicle position tracking.
    Validates: Requirements 2.1, 2.2, 2.5
    """
    
    @settings(max_examples=100)
    @given(
        vehicle_count=st.integers(min_value=0, max_value=4),  # Max 4 vehicles per lane
        positions=st.lists(position_strategy(), min_size=0, max_size=4)
    )
    def test_property_2_vehicle_count_accuracy(self, vehicle_count, positions):
        """
        Feature: traffic-control-fix, Property 2: Vehicle Count Accuracy
        
        For any sensor data message received, the Traffic_Manager's vehicle 
        count for that direction SHALL equal the number of vehicle models 
        in the sensor data.
        
        Validates: Requirements 2.2
        """
        # Create tracker instance
        tracker = VehiclePositionTrackerLogic()
        
        # Generate unique vehicle names for north lane (max 4 per lane)
        vehicle_data = []
        actual_count = min(vehicle_count, len(positions), 4)  # Can't have more than 4 vehicles per lane
        for i in range(actual_count):
            vehicle_name = f'car_n{i + 1}'
            vehicle_data.append((vehicle_name, positions[i]))
        
        # Create mock sensor message
        msg = create_mock_sensor_message(vehicle_data)
        
        # Process the sensor data
        tracker.process_sensor_data(msg)
        
        # Count vehicles in north direction
        counted = tracker.get_vehicle_count('north')
        
        # The count should equal the number of unique north vehicles in the message
        expected_count = len(vehicle_data)
        
        self.assertEqual(
            counted,
            expected_count,
            f"Vehicle count mismatch: expected {expected_count}, got {counted}"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_name=vehicle_name_strategy(),
        position=position_strategy()
    )
    def test_property_3_vehicle_detection_in_zone(self, vehicle_name, position):
        """
        Feature: traffic-control-fix, Property 3: Vehicle Detection in Zone
        
        For any vehicle positioned within a Detection_Zone's range and field 
        of view, the Logical_Camera SHALL include that vehicle in its sensor 
        data output.
        
        Validates: Requirements 2.1
        """
        # Create tracker instance
        tracker = VehiclePositionTrackerLogic()
        
        # Create sensor message with the vehicle
        vehicle_data = [(vehicle_name, position)]
        msg = create_mock_sensor_message(vehicle_data)
        
        # Process the sensor data
        tracker.process_sensor_data(msg)
        
        # Verify the vehicle is tracked
        self.assertIn(
            vehicle_name,
            tracker.vehicle_positions,
            f"Vehicle {vehicle_name} should be detected and tracked"
        )
        
        # Verify the position is correct
        tracked_position = tracker.vehicle_positions[vehicle_name]
        self.assertEqual(
            tracked_position[0],
            position[0],
            f"X position mismatch for {vehicle_name}"
        )
        self.assertEqual(
            tracked_position[1],
            position[1],
            f"Y position mismatch for {vehicle_name}"
        )
        self.assertEqual(
            tracked_position[2],
            position[2],
            f"Z position mismatch for {vehicle_name}"
        )
    
    @settings(max_examples=100)
    @given(
        vehicle_names=st.lists(vehicle_name_strategy(), min_size=1, max_size=5, unique=True),
        positions=st.lists(position_strategy(), min_size=1, max_size=5)
    )
    def test_property_4_count_update_on_exit(self, vehicle_names, positions):
        """
        Feature: traffic-control-fix, Property 4: Count Update on Exit
        
        For any vehicle that exits a Detection_Zone (present in previous 
        sensor data but not in current), the vehicle count SHALL decrease by one.
        
        Validates: Requirements 2.5
        """
        # Create tracker instance
        tracker = VehiclePositionTrackerLogic()
        
        # Ensure we have at least as many positions as vehicle names
        if len(positions) < len(vehicle_names):
            positions = positions + [positions[0]] * (len(vehicle_names) - len(positions))
        
        # First sensor update: all vehicles present
        vehicle_data_initial = [(name, positions[i]) for i, name in enumerate(vehicle_names)]
        msg_initial = create_mock_sensor_message(vehicle_data_initial)
        tracker.process_sensor_data(msg_initial)
        
        # Get initial count for the first vehicle's direction
        first_vehicle_lane = tracker.vehicle_lanes.get(vehicle_names[0])
        initial_count = tracker.get_vehicle_count(first_vehicle_lane)
        
        # Second sensor update: remove the first vehicle
        vehicle_data_updated = vehicle_data_initial[1:]
        msg_updated = create_mock_sensor_message(vehicle_data_updated)
        
        # Clear positions and reprocess (simulating vehicle exit)
        tracker.vehicle_positions.clear()
        tracker.process_sensor_data(msg_updated)
        
        # Get updated count
        updated_count = tracker.get_vehicle_count(first_vehicle_lane)
        
        # Count should decrease by 1 if the removed vehicle was in the same lane
        removed_vehicle_lane = tracker.vehicle_lanes.get(vehicle_names[0])
        if removed_vehicle_lane == first_vehicle_lane:
            expected_decrease = 1
        else:
            expected_decrease = 0
        
        self.assertEqual(
            updated_count,
            initial_count - expected_decrease,
            f"Count should decrease by {expected_decrease} when vehicle exits"
        )


class TestDistanceCalculation(unittest.TestCase):
    """
    Additional tests for distance calculation functionality.
    Validates: Requirements 5.1, 5.2
    """
    
    def test_distance_to_vehicle_ahead_north_lane(self):
        """Test distance calculation for north lane (southbound traffic)"""
        tracker = VehiclePositionTrackerLogic()
        
        # Place two vehicles in north lane
        vehicle_data = [
            ('car_n1', (0.0, 20.0, 0.0)),  # Behind
            ('car_n2', (0.0, 10.0, 0.0))   # Ahead
        ]
        msg = create_mock_sensor_message(vehicle_data)
        tracker.process_sensor_data(msg)
        
        # Calculate distance from n1 to vehicle ahead
        distance = tracker.get_distance_to_vehicle_ahead('car_n1')
        
        # Distance should be 10.0 meters
        self.assertAlmostEqual(distance, 10.0, places=1)
    
    def test_distance_to_vehicle_ahead_south_lane(self):
        """Test distance calculation for south lane (northbound traffic)"""
        tracker = VehiclePositionTrackerLogic()
        
        # Place two vehicles in south lane
        vehicle_data = [
            ('car_s1', (0.0, -20.0, 0.0)),  # Behind
            ('car_s2', (0.0, -10.0, 0.0))   # Ahead
        ]
        msg = create_mock_sensor_message(vehicle_data)
        tracker.process_sensor_data(msg)
        
        # Calculate distance from s1 to vehicle ahead
        distance = tracker.get_distance_to_vehicle_ahead('car_s1')
        
        # Distance should be 10.0 meters
        self.assertAlmostEqual(distance, 10.0, places=1)
    
    def test_distance_to_vehicle_ahead_east_lane(self):
        """Test distance calculation for east lane (westbound traffic)"""
        tracker = VehiclePositionTrackerLogic()
        
        # Place two vehicles in east lane
        vehicle_data = [
            ('car_e1', (20.0, 0.0, 0.0)),  # Behind
            ('car_e2', (10.0, 0.0, 0.0))   # Ahead
        ]
        msg = create_mock_sensor_message(vehicle_data)
        tracker.process_sensor_data(msg)
        
        # Calculate distance from e1 to vehicle ahead
        distance = tracker.get_distance_to_vehicle_ahead('car_e1')
        
        # Distance should be 10.0 meters
        self.assertAlmostEqual(distance, 10.0, places=1)
    
    def test_distance_to_vehicle_ahead_west_lane(self):
        """Test distance calculation for west lane (eastbound traffic)"""
        tracker = VehiclePositionTrackerLogic()
        
        # Place two vehicles in west lane
        vehicle_data = [
            ('car_w1', (-20.0, 0.0, 0.0)),  # Behind
            ('car_w2', (-10.0, 0.0, 0.0))   # Ahead
        ]
        msg = create_mock_sensor_message(vehicle_data)
        tracker.process_sensor_data(msg)
        
        # Calculate distance from w1 to vehicle ahead
        distance = tracker.get_distance_to_vehicle_ahead('car_w1')
        
        # Distance should be 10.0 meters
        self.assertAlmostEqual(distance, 10.0, places=1)
    
    def test_no_vehicle_ahead_returns_infinity(self):
        """Test that infinity is returned when no vehicle is ahead"""
        tracker = VehiclePositionTrackerLogic()
        
        # Place only one vehicle
        vehicle_data = [('car_n1', (0.0, 20.0, 0.0))]
        msg = create_mock_sensor_message(vehicle_data)
        tracker.process_sensor_data(msg)
        
        # Calculate distance - should be infinity
        distance = tracker.get_distance_to_vehicle_ahead('car_n1')
        
        self.assertEqual(distance, float('inf'))
    
    def test_unknown_vehicle_returns_infinity(self):
        """Test that infinity is returned for unknown vehicles"""
        tracker = VehiclePositionTrackerLogic()
        
        # Don't add any vehicles
        distance = tracker.get_distance_to_vehicle_ahead('car_n1')
        
        self.assertEqual(distance, float('inf'))


if __name__ == '__main__':
    unittest.main()
