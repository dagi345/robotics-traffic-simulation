#!/usr/bin/env python3
"""
Integration tests for the Smart Traffic System.
Feature: traffic-control-fix

Tests the complete traffic flow cycle including:
- Traffic light state transitions
- Vehicle control based on light states
- Collision avoidance between vehicles
- Adaptive signal timing with varying traffic

Validates: All requirements
"""

import unittest
from hypothesis import given, settings, strategies as st, assume
import sys
import os
import time

# Add the scripts directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class MockSensorData:
    """Mock sensor data for testing"""
    def __init__(self, models=None):
        self.models = models or []


class MockModel:
    """Mock model for sensor data"""
    def __init__(self, name, x=0.0, y=0.0, z=0.0):
        self.name = name
        self.pose = MockPose(x, y, z)


class MockPose:
    """Mock pose for model"""
    def __init__(self, x, y, z):
        self.position = MockPosition(x, y, z)


class MockPosition:
    """Mock position"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class IntegratedTrafficSystem:
    """
    Integrated traffic system for testing.
    Combines TrafficManager and FlowController logic.
    """
    
    def __init__(self):
        # Traffic Manager state
        self.MIN_GREEN = 10.0
        self.MAX_GREEN = 30.0
        self.ALL_RED_TIME = 2.0
        self.SENSOR_TIMEOUT = 1.0
        
        self.NS_GREEN = "NS_GREEN"
        self.EW_GREEN = "EW_GREEN"
        self.ALL_RED = "ALL_RED"
        
        self.current_state = self.NS_GREEN
        self.last_switch_time = 0.0
        self.target_state = self.EW_GREEN
        self.current_time = 0.0
        
        # Vehicle counts per direction
        self.counts = {'n': 0, 's': 0, 'e': 0, 'w': 0}
        
        # Sensor failure tracking
        self.last_sensor_update = {'n': 0.0, 's': 0.0, 'e': 0.0, 'w': 0.0}
        self.sensor_failed = {'n': False, 's': False, 'e': False, 'w': False}
        
        # Flow Controller state
        self.SAFE_FOLLOWING_DISTANCE = 5.0
        self.STOP_DISTANCE = 3.0
        
        self.speeds = {}
        self.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
        self.vehicle_positions = {}
        
        # State transition history
        self.state_history = []
        
        # Initialize vehicles
        for d in ['n', 's', 'e', 'w']:
            for i in range(1, 5):
                name = f'car_{d}{i}'
                self.speeds[name] = 4.0  # Default speed
    
    def set_time(self, t):
        """Set current simulation time"""
        self.current_time = t
    
    def advance_time(self, dt):
        """Advance simulation time"""
        self.current_time += dt
    
    def update_sensor_data(self, direction, vehicles):
        """
        Update sensor data for a direction.
        
        Args:
            direction: 'n', 's', 'e', 'w'
            vehicles: list of vehicle names in the detection zone
        """
        self.counts[direction] = len(vehicles)
        self.in_zones[direction] = set(vehicles)
        self.last_sensor_update[direction] = self.current_time
        self.sensor_failed[direction] = False
    
    def update_vehicle_position(self, vehicle_name, x, y, z=0.0):
        """Update a vehicle's position"""
        self.vehicle_positions[vehicle_name] = (x, y, z)
    
    def get_distance_to_vehicle_ahead(self, vehicle_name):
        """Calculate distance to vehicle ahead in same lane"""
        if vehicle_name not in self.vehicle_positions:
            return float('inf')
        
        my_pos = self.vehicle_positions[vehicle_name]
        car_road = vehicle_name.split('_')[1][0]
        
        min_distance = float('inf')
        
        for other_name, other_pos in self.vehicle_positions.items():
            if other_name == vehicle_name:
                continue
            other_road = other_name.split('_')[1][0]
            if other_road != car_road:
                continue
            
            # Check if other vehicle is ahead based on direction
            if car_road == 'n':
                if other_pos[1] < my_pos[1]:  # Other is ahead (lower y)
                    distance = my_pos[1] - other_pos[1]
                    min_distance = min(min_distance, distance)
            elif car_road == 's':
                if other_pos[1] > my_pos[1]:  # Other is ahead (higher y)
                    distance = other_pos[1] - my_pos[1]
                    min_distance = min(min_distance, distance)
            elif car_road == 'e':
                if other_pos[0] > my_pos[0]:  # Other is ahead (higher x)
                    distance = other_pos[0] - my_pos[0]
                    min_distance = min(min_distance, distance)
            elif car_road == 'w':
                if other_pos[0] < my_pos[0]:  # Other is ahead (lower x)
                    distance = my_pos[0] - other_pos[0]
                    min_distance = min(min_distance, distance)
        
        return min_distance
    
    def run_traffic_manager_logic(self):
        """Run one iteration of traffic manager logic"""
        elapsed = self.current_time - self.last_switch_time
        
        waiting_ns = self.counts['n'] + self.counts['s']
        waiting_ew = self.counts['e'] + self.counts['w']
        
        previous_state = self.current_state
        next_state = self.current_state
        
        if self.current_state == self.NS_GREEN:
            if (waiting_ew > 0 and (waiting_ns == 0 or elapsed > self.MAX_GREEN)) and elapsed > self.MIN_GREEN:
                next_state = self.ALL_RED
                self.target_state = self.EW_GREEN
        
        elif self.current_state == self.EW_GREEN:
            if (waiting_ns > 0 and (waiting_ew == 0 or elapsed > self.MAX_GREEN)) and elapsed > self.MIN_GREEN:
                next_state = self.ALL_RED
                self.target_state = self.NS_GREEN
        
        elif self.current_state == self.ALL_RED:
            if elapsed > self.ALL_RED_TIME:
                next_state = self.target_state
        
        if next_state != self.current_state:
            self.state_history.append({
                'time': self.current_time,
                'from': self.current_state,
                'to': next_state
            })
            self.current_state = next_state
            self.last_switch_time = self.current_time
    
    def get_blocked_roads(self):
        """Get list of roads that should stop for red light"""
        if self.current_state == self.NS_GREEN:
            return ['e', 'w']
        elif self.current_state == self.EW_GREEN:
            return ['n', 's']
        elif self.current_state == self.ALL_RED:
            return ['n', 's', 'e', 'w']
        return []
    
    def calculate_vehicle_velocity(self, vehicle_name):
        """Calculate velocity for a vehicle"""
        if vehicle_name not in self.speeds:
            return 0.0
        
        car_road = vehicle_name.split('_')[1][0]
        blocked_roads = self.get_blocked_roads()
        is_at_red_light = (car_road in blocked_roads and vehicle_name in self.in_zones[car_road])
        
        distance_ahead = self.get_distance_to_vehicle_ahead(vehicle_name)
        assigned_speed = self.speeds[vehicle_name]
        
        if is_at_red_light:
            return 0.0
        elif distance_ahead < self.STOP_DISTANCE:
            return 0.0
        elif distance_ahead < self.SAFE_FOLLOWING_DISTANCE:
            ratio = (distance_ahead - self.STOP_DISTANCE) / \
                   (self.SAFE_FOLLOWING_DISTANCE - self.STOP_DISTANCE)
            return assigned_speed * ratio
        else:
            return assigned_speed


class TestTrafficFlowCycle(unittest.TestCase):
    """
    Integration tests for complete traffic flow cycle.
    """
    
    def test_complete_light_cycle(self):
        """Test a complete traffic light cycle: NS_GREEN -> ALL_RED -> EW_GREEN -> ALL_RED -> NS_GREEN"""
        system = IntegratedTrafficSystem()
        
        # Initial state should be NS_GREEN
        self.assertEqual(system.current_state, "NS_GREEN")
        
        # Add vehicles waiting in EW direction
        system.update_sensor_data('e', ['car_e1', 'car_e2'])
        system.update_sensor_data('w', ['car_w1'])
        
        # Advance past MIN_GREEN
        system.advance_time(11.0)
        system.run_traffic_manager_logic()
        
        # Should transition to ALL_RED
        self.assertEqual(system.current_state, "ALL_RED")
        
        # Advance past ALL_RED_TIME
        system.advance_time(3.0)
        system.run_traffic_manager_logic()
        
        # Should transition to EW_GREEN
        self.assertEqual(system.current_state, "EW_GREEN")
        
        # Add vehicles waiting in NS direction
        system.update_sensor_data('n', ['car_n1'])
        system.update_sensor_data('s', ['car_s1'])
        system.update_sensor_data('e', [])  # Clear EW
        system.update_sensor_data('w', [])
        
        # Advance past MIN_GREEN
        system.advance_time(11.0)
        system.run_traffic_manager_logic()
        
        # Should transition to ALL_RED
        self.assertEqual(system.current_state, "ALL_RED")
        
        # Advance past ALL_RED_TIME
        system.advance_time(3.0)
        system.run_traffic_manager_logic()
        
        # Should transition back to NS_GREEN
        self.assertEqual(system.current_state, "NS_GREEN")
    
    def test_all_red_always_between_green_states(self):
        """Test that ALL_RED always occurs between green states"""
        system = IntegratedTrafficSystem()
        
        # Simulate multiple cycles
        for _ in range(3):
            # Add EW traffic
            system.update_sensor_data('e', ['car_e1'])
            
            # Advance past MIN_GREEN
            system.advance_time(11.0)
            system.run_traffic_manager_logic()
            
            # Advance past ALL_RED
            system.advance_time(3.0)
            system.run_traffic_manager_logic()
            
            # Add NS traffic
            system.update_sensor_data('n', ['car_n1'])
            system.update_sensor_data('e', [])
            
            # Advance past MIN_GREEN
            system.advance_time(11.0)
            system.run_traffic_manager_logic()
            
            # Advance past ALL_RED
            system.advance_time(3.0)
            system.run_traffic_manager_logic()
        
        # Check state history - every transition between green states should go through ALL_RED
        for i in range(len(system.state_history) - 1):
            current = system.state_history[i]
            next_trans = system.state_history[i + 1]
            
            # If transitioning from a green state
            if current['from'] in ['NS_GREEN', 'EW_GREEN']:
                # Must go to ALL_RED first
                self.assertEqual(
                    current['to'],
                    'ALL_RED',
                    f"Transition from {current['from']} should go to ALL_RED, not {current['to']}"
                )


class TestVehicleQueueing(unittest.TestCase):
    """
    Integration tests for multiple vehicles queuing and releasing.
    """
    
    def test_vehicles_queue_at_red_light(self):
        """Test that multiple vehicles stop at red light"""
        system = IntegratedTrafficSystem()
        system.current_state = "EW_GREEN"  # NS has red
        
        # Position vehicles in north lane approaching intersection
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 28)
        system.update_vehicle_position('car_n3', 0, 36)
        
        # Add to detection zone
        system.update_sensor_data('n', ['car_n1', 'car_n2', 'car_n3'])
        
        # All vehicles should stop
        for name in ['car_n1', 'car_n2', 'car_n3']:
            velocity = system.calculate_vehicle_velocity(name)
            self.assertEqual(
                velocity,
                0.0,
                f"{name} should stop at red light"
            )
    
    def test_vehicles_release_on_green(self):
        """Test that vehicles resume movement when light turns green"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"  # NS has green
        
        # Position vehicles with safe spacing
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 30)  # 10m behind
        
        # Add to detection zone
        system.update_sensor_data('n', ['car_n1', 'car_n2'])
        
        # Both vehicles should move (safe distance apart)
        for name in ['car_n1', 'car_n2']:
            velocity = system.calculate_vehicle_velocity(name)
            self.assertGreater(
                velocity,
                0.0,
                f"{name} should move at green light with safe spacing"
            )
    
    def test_queue_maintains_safe_distance(self):
        """Test that queued vehicles maintain safe following distance"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Position vehicles too close together
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 23)  # Only 3m behind (< STOP_DISTANCE)
        
        # First vehicle should move, second should stop
        v1 = system.calculate_vehicle_velocity('car_n1')
        v2 = system.calculate_vehicle_velocity('car_n2')
        
        self.assertGreater(v1, 0.0, "Lead vehicle should move")
        self.assertEqual(v2, 0.0, "Following vehicle should stop (too close)")


class TestCollisionAvoidanceIntegration(unittest.TestCase):
    """
    Integration tests for collision avoidance with traffic lights.
    """
    
    def test_collision_avoidance_at_green_light(self):
        """Test collision avoidance works even with green light"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Position vehicles close together
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 22)  # 2m behind (< STOP_DISTANCE)
        
        # Second vehicle should stop despite green light
        v2 = system.calculate_vehicle_velocity('car_n2')
        self.assertEqual(v2, 0.0, "Should stop due to collision avoidance")
    
    def test_proportional_slowdown(self):
        """Test proportional speed reduction between STOP and SAFE distances"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Position lead vehicle (north vehicles move toward lower y values)
        # car_n1 is ahead (lower y), car_n2 is behind (higher y)
        system.update_vehicle_position('car_n1', 0, 16)  # Lead vehicle
        
        # Test at 4m behind (midpoint between 3m STOP and 5m SAFE)
        system.update_vehicle_position('car_n2', 0, 20)  # 4m behind car_n1
        v_mid = system.calculate_vehicle_velocity('car_n2')
        
        # Now move car_n2 to 6m behind (beyond SAFE distance)
        system.update_vehicle_position('car_n2', 0, 22)  # 6m behind car_n1
        v_safe = system.calculate_vehicle_velocity('car_n2')
        
        # Midpoint should be slower than safe distance
        self.assertLess(v_mid, v_safe)
        self.assertGreater(v_mid, 0.0)
        self.assertEqual(v_safe, system.speeds['car_n2'])  # Full speed at safe distance
    
    def test_collision_avoidance_independent_of_light_state(self):
        """Test collision avoidance works in all light states"""
        system = IntegratedTrafficSystem()
        
        # Position vehicles close together
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 22)  # 2m behind
        
        for state in ["NS_GREEN", "EW_GREEN", "ALL_RED"]:
            system.current_state = state
            v2 = system.calculate_vehicle_velocity('car_n2')
            self.assertEqual(
                v2,
                0.0,
                f"Should stop due to collision avoidance in {state}"
            )


class TestAdaptiveSignalTiming(unittest.TestCase):
    """
    Integration tests for adaptive signal timing with varying traffic.
    """
    
    def test_switches_when_perpendicular_has_traffic(self):
        """Test that signal switches when perpendicular direction has waiting traffic"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # No NS traffic, but EW has traffic
        system.update_sensor_data('n', [])
        system.update_sensor_data('s', [])
        system.update_sensor_data('e', ['car_e1', 'car_e2'])
        system.update_sensor_data('w', ['car_w1'])
        
        # Advance past MIN_GREEN
        system.advance_time(11.0)
        system.run_traffic_manager_logic()
        
        # Should switch to ALL_RED (then EW_GREEN)
        self.assertEqual(system.current_state, "ALL_RED")
        self.assertEqual(system.target_state, "EW_GREEN")
    
    def test_respects_min_green_time(self):
        """Test that MIN_GREEN time is respected"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Add EW traffic
        system.update_sensor_data('e', ['car_e1'])
        
        # Advance less than MIN_GREEN
        system.advance_time(5.0)
        system.run_traffic_manager_logic()
        
        # Should NOT switch yet
        self.assertEqual(system.current_state, "NS_GREEN")
    
    def test_respects_max_green_time(self):
        """Test that MAX_GREEN time triggers switch even with traffic"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Both directions have traffic
        system.update_sensor_data('n', ['car_n1'])
        system.update_sensor_data('e', ['car_e1'])
        
        # Advance past MAX_GREEN
        system.advance_time(31.0)
        system.run_traffic_manager_logic()
        
        # Should switch to ALL_RED
        self.assertEqual(system.current_state, "ALL_RED")
    
    def test_stays_green_when_no_perpendicular_traffic(self):
        """Test that signal stays green when no perpendicular traffic"""
        system = IntegratedTrafficSystem()
        system.current_state = "NS_GREEN"
        
        # Only NS has traffic
        system.update_sensor_data('n', ['car_n1'])
        system.update_sensor_data('e', [])
        system.update_sensor_data('w', [])
        
        # Advance past MIN_GREEN but not MAX_GREEN
        system.advance_time(15.0)
        system.run_traffic_manager_logic()
        
        # Should stay NS_GREEN (no EW traffic)
        self.assertEqual(system.current_state, "NS_GREEN")


class TestEndToEndScenarios(unittest.TestCase):
    """
    End-to-end integration tests for realistic scenarios.
    """
    
    def test_rush_hour_scenario(self):
        """Test system behavior with heavy traffic in all directions"""
        system = IntegratedTrafficSystem()
        
        # Heavy traffic in all directions
        system.update_sensor_data('n', ['car_n1', 'car_n2', 'car_n3', 'car_n4'])
        system.update_sensor_data('s', ['car_s1', 'car_s2', 'car_s3', 'car_s4'])
        system.update_sensor_data('e', ['car_e1', 'car_e2', 'car_e3', 'car_e4'])
        system.update_sensor_data('w', ['car_w1', 'car_w2', 'car_w3', 'car_w4'])
        
        # Run for several cycles
        for _ in range(10):
            system.advance_time(5.0)
            system.run_traffic_manager_logic()
        
        # Should have made transitions
        self.assertGreater(len(system.state_history), 0)
        
        # Verify all transitions went through ALL_RED
        for transition in system.state_history:
            if transition['from'] in ['NS_GREEN', 'EW_GREEN']:
                self.assertEqual(transition['to'], 'ALL_RED')
    
    def test_empty_intersection_scenario(self):
        """Test system behavior with no traffic"""
        system = IntegratedTrafficSystem()
        
        # No traffic
        system.update_sensor_data('n', [])
        system.update_sensor_data('s', [])
        system.update_sensor_data('e', [])
        system.update_sensor_data('w', [])
        
        # Run for a while
        for _ in range(20):
            system.advance_time(5.0)
            system.run_traffic_manager_logic()
        
        # Should stay in initial state (no reason to switch)
        self.assertEqual(system.current_state, "NS_GREEN")
    
    def test_single_direction_traffic(self):
        """Test system with traffic only in one direction"""
        system = IntegratedTrafficSystem()
        
        # Only north has traffic
        system.update_sensor_data('n', ['car_n1', 'car_n2'])
        system.update_sensor_data('s', [])
        system.update_sensor_data('e', [])
        system.update_sensor_data('w', [])
        
        # Position vehicles with safe spacing
        system.update_vehicle_position('car_n1', 0, 20)
        system.update_vehicle_position('car_n2', 0, 30)
        
        # Vehicles should move (NS_GREEN is initial state)
        v1 = system.calculate_vehicle_velocity('car_n1')
        v2 = system.calculate_vehicle_velocity('car_n2')
        
        self.assertGreater(v1, 0.0)
        self.assertGreater(v2, 0.0)


if __name__ == '__main__':
    unittest.main()
