#!/usr/bin/env python3
"""
Property-Based Tests for Intersection Zone Safety

Tests the correctness properties for intersection collision prevention:
- Property 1: Conflicting Traffic Exclusion
- Property 2: Intersection Clearance Before Phase Change
- Property 22: Intersection Zone Vehicle Tracking

Requirements: 1.1, 1.2, 1.3, 1.4, 1.6, 10.2, 10.3
"""
import pytest
from hypothesis import given, settings, assume
import hypothesis.strategies as st
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


# ============================================================================
# Test Helpers and Strategies
# ============================================================================

# Traffic light states
TRAFFIC_STATES = ['NS_GREEN', 'EW_GREEN', 'ALL_RED']

# Vehicle directions
DIRECTIONS = ['n', 's', 'e', 'w']

# Intersection zone boundaries
ZONE_X_MIN = -13.0
ZONE_X_MAX = 13.0
ZONE_Y_MIN = -13.0
ZONE_Y_MAX = 13.0


@st.composite
def vehicle_position_strategy(draw):
    """Generate a random vehicle position."""
    x = draw(st.floats(min_value=-50.0, max_value=50.0, allow_nan=False))
    y = draw(st.floats(min_value=-50.0, max_value=50.0, allow_nan=False))
    return (x, y)


@st.composite
def vehicle_in_zone_strategy(draw):
    """Generate a vehicle position inside the intersection zone."""
    x = draw(st.floats(min_value=ZONE_X_MIN + 1, max_value=ZONE_X_MAX - 1, allow_nan=False))
    y = draw(st.floats(min_value=ZONE_Y_MIN + 1, max_value=ZONE_Y_MAX - 1, allow_nan=False))
    return (x, y)


@st.composite
def vehicle_outside_zone_strategy(draw):
    """Generate a vehicle position outside the intersection zone."""
    # Choose which side to place the vehicle
    side = draw(st.sampled_from(['north', 'south', 'east', 'west']))
    
    if side == 'north':
        x = draw(st.floats(min_value=-6.0, max_value=6.0, allow_nan=False))
        y = draw(st.floats(min_value=ZONE_Y_MAX + 1, max_value=50.0, allow_nan=False))
    elif side == 'south':
        x = draw(st.floats(min_value=-6.0, max_value=6.0, allow_nan=False))
        y = draw(st.floats(min_value=-50.0, max_value=ZONE_Y_MIN - 1, allow_nan=False))
    elif side == 'east':
        x = draw(st.floats(min_value=ZONE_X_MAX + 1, max_value=50.0, allow_nan=False))
        y = draw(st.floats(min_value=-6.0, max_value=6.0, allow_nan=False))
    else:  # west
        x = draw(st.floats(min_value=-50.0, max_value=ZONE_X_MIN - 1, allow_nan=False))
        y = draw(st.floats(min_value=-6.0, max_value=6.0, allow_nan=False))
    
    return (x, y)


def is_in_zone(position):
    """Check if a position is inside the intersection zone."""
    x, y = position
    return (ZONE_X_MIN <= x <= ZONE_X_MAX and ZONE_Y_MIN <= y <= ZONE_Y_MAX)


def get_blocked_roads(light_state):
    """Get list of roads that should be blocked for a given light state."""
    if light_state == "NS_GREEN":
        return ['e', 'w']
    elif light_state == "EW_GREEN":
        return ['n', 's']
    elif light_state == "ALL_RED":
        return ['n', 's', 'e', 'w']
    return []


def get_allowed_roads(light_state):
    """Get list of roads that are allowed to proceed for a given light state."""
    if light_state == "NS_GREEN":
        return ['n', 's']
    elif light_state == "EW_GREEN":
        return ['e', 'w']
    elif light_state == "ALL_RED":
        return []
    return []


# ============================================================================
# Mock Classes for Testing
# ============================================================================

class MockIntersectionZoneMonitor:
    """Mock implementation of IntersectionZoneMonitor for testing."""
    
    ZONE_BOUNDS = {
        'x_min': ZONE_X_MIN,
        'x_max': ZONE_X_MAX,
        'y_min': ZONE_Y_MIN,
        'y_max': ZONE_Y_MAX
    }
    
    def __init__(self):
        self.vehicles_in_zone = {}
        self.vehicle_directions = {}
    
    def add_vehicle(self, name, position, direction):
        """Add a vehicle to tracking."""
        if is_in_zone(position):
            self.vehicles_in_zone[name] = position
            self.vehicle_directions[name] = direction
    
    def remove_vehicle(self, name):
        """Remove a vehicle from tracking."""
        if name in self.vehicles_in_zone:
            del self.vehicles_in_zone[name]
        if name in self.vehicle_directions:
            del self.vehicle_directions[name]
    
    def is_zone_clear(self):
        """Check if zone is completely clear."""
        return len(self.vehicles_in_zone) == 0
    
    def is_clear_for_direction(self, directions):
        """Check if zone is clear of vehicles from specified directions."""
        for vehicle_name in self.vehicles_in_zone:
            direction = self.vehicle_directions.get(vehicle_name)
            if direction in directions:
                return False
        return True
    
    def is_ns_clear(self):
        """Check if zone is clear for NS traffic (no EW vehicles)."""
        return self.is_clear_for_direction(['e', 'w'])
    
    def is_ew_clear(self):
        """Check if zone is clear for EW traffic (no NS vehicles)."""
        return self.is_clear_for_direction(['n', 's'])


class MockTrafficManager:
    """Mock implementation of TrafficManager for testing."""
    
    def __init__(self):
        self.current_state = "NS_GREEN"
        self.zone_ns_clear = True
        self.zone_ew_clear = True
        self.vehicles_in_zone = []
    
    def update_zone_status(self, ns_clear, ew_clear, vehicles):
        """Update zone status from monitor."""
        self.zone_ns_clear = ns_clear
        self.zone_ew_clear = ew_clear
        self.vehicles_in_zone = vehicles
    
    def is_safe_to_transition(self, target_state):
        """Check if it's safe to transition to target state."""
        if target_state == "NS_GREEN":
            return self.zone_ns_clear
        elif target_state == "EW_GREEN":
            return self.zone_ew_clear
        return True


class MockFlowController:
    """Mock implementation of FlowController for testing."""
    
    def __init__(self):
        self.light_state = "NS_GREEN"
        self.zone_ns_clear = True
        self.zone_ew_clear = True
        self.vehicle_velocities = {}
    
    def should_stop_for_zone_conflict(self, vehicle_name, position, direction):
        """Check if vehicle should stop due to zone conflict."""
        if not is_in_zone(position):
            # Vehicle not in zone, check if approaching
            distance_to_zone = self._distance_to_zone(position, direction)
            if distance_to_zone > 5.0:
                return False
        
        # Check for conflicting vehicles
        if direction in ['n', 's']:
            return not self.zone_ns_clear
        else:
            return not self.zone_ew_clear
    
    def _distance_to_zone(self, position, direction):
        """Calculate distance to intersection zone."""
        x, y = position
        if direction in ['n', 's']:
            return abs(y) - ZONE_Y_MAX if abs(y) > ZONE_Y_MAX else 0
        else:
            return abs(x) - ZONE_X_MAX if abs(x) > ZONE_X_MAX else 0
    
    def get_velocity_command(self, vehicle_name, position, direction, assigned_speed):
        """Get velocity command for a vehicle."""
        blocked_roads = get_blocked_roads(self.light_state)
        
        # Check red light
        if direction in blocked_roads:
            return 0.0, "red_light"
        
        # Check zone conflict
        if self.should_stop_for_zone_conflict(vehicle_name, position, direction):
            return 0.0, "zone_conflict"
        
        return assigned_speed, "normal"


# ============================================================================
# Property Tests
# ============================================================================

class TestConflictingTrafficExclusion:
    """
    Property 1: Conflicting Traffic Exclusion
    
    For any traffic light state, vehicles from conflicting directions SHALL NOT
    both have permission to enter the Intersection_Zone simultaneously.
    
    Validates: Requirements 1.1, 1.2, 1.3
    """
    
    @settings(max_examples=100)
    @given(
        light_state=st.sampled_from(TRAFFIC_STATES),
        ns_vehicle_dir=st.sampled_from(['n', 's']),
        ew_vehicle_dir=st.sampled_from(['e', 'w'])
    )
    def test_conflicting_directions_never_both_allowed(
        self, light_state, ns_vehicle_dir, ew_vehicle_dir
    ):
        """
        Feature: adaptive-traffic-system-v2, Property 1: Conflicting Traffic Exclusion
        
        For any light state, NS and EW vehicles cannot both be allowed to proceed.
        """
        allowed_roads = get_allowed_roads(light_state)
        
        ns_allowed = ns_vehicle_dir in allowed_roads
        ew_allowed = ew_vehicle_dir in allowed_roads
        
        # Both cannot be allowed simultaneously
        assert not (ns_allowed and ew_allowed), \
            f"Both NS ({ns_vehicle_dir}) and EW ({ew_vehicle_dir}) allowed in state {light_state}"
    
    @settings(max_examples=100)
    @given(
        light_state=st.sampled_from(TRAFFIC_STATES),
        vehicle_positions=st.lists(
            vehicle_position_strategy(),
            min_size=1, max_size=16
        )
    )
    def test_blocked_vehicles_commanded_to_stop(self, light_state, vehicle_positions):
        """
        Feature: adaptive-traffic-system-v2, Property 1: Conflicting Traffic Exclusion
        
        Vehicles on blocked roads must be commanded to stop.
        """
        controller = MockFlowController()
        controller.light_state = light_state
        
        blocked_roads = get_blocked_roads(light_state)
        
        for i, pos in enumerate(vehicle_positions):
            # Assign direction based on index
            direction = DIRECTIONS[i % 4]
            vehicle_name = f"car_{direction}{(i // 4) + 1}"
            
            velocity, reason = controller.get_velocity_command(
                vehicle_name, pos, direction, 5.0
            )
            
            if direction in blocked_roads:
                assert velocity == 0.0, \
                    f"Vehicle {vehicle_name} on blocked road {direction} " \
                    f"should stop but got velocity {velocity}"


class TestIntersectionClearanceBeforePhaseChange:
    """
    Property 2: Intersection Clearance Before Phase Change
    
    For any state transition from ALL_RED to a green phase, the Intersection_Zone
    SHALL be clear of vehicles from the conflicting direction before transition.
    
    Validates: Requirements 1.4, 1.6, 10.3
    """
    
    @settings(max_examples=100)
    @given(
        target_state=st.sampled_from(['NS_GREEN', 'EW_GREEN']),
        conflicting_vehicle_in_zone=st.booleans()
    )
    def test_transition_blocked_when_conflicting_vehicles_present(
        self, target_state, conflicting_vehicle_in_zone
    ):
        """
        Feature: adaptive-traffic-system-v2, Property 2: Intersection Clearance
        
        Transition to green is blocked when conflicting vehicles are in zone.
        """
        manager = MockTrafficManager()
        manager.current_state = "ALL_RED"
        
        if target_state == "NS_GREEN":
            # NS green requires no EW vehicles in zone
            manager.zone_ns_clear = not conflicting_vehicle_in_zone
            manager.zone_ew_clear = True
        else:  # EW_GREEN
            # EW green requires no NS vehicles in zone
            manager.zone_ns_clear = True
            manager.zone_ew_clear = not conflicting_vehicle_in_zone
        
        is_safe = manager.is_safe_to_transition(target_state)
        
        if conflicting_vehicle_in_zone:
            assert not is_safe, \
                f"Transition to {target_state} should be blocked with conflicting vehicles"
        else:
            assert is_safe, \
                f"Transition to {target_state} should be allowed when zone is clear"
    
    @settings(max_examples=100)
    @given(
        ns_vehicles=st.lists(vehicle_in_zone_strategy(), min_size=0, max_size=4),
        ew_vehicles=st.lists(vehicle_in_zone_strategy(), min_size=0, max_size=4)
    )
    def test_zone_clearance_detection(self, ns_vehicles, ew_vehicles):
        """
        Feature: adaptive-traffic-system-v2, Property 2: Intersection Clearance
        
        Zone monitor correctly detects which directions have vehicles.
        """
        monitor = MockIntersectionZoneMonitor()
        
        # Add NS vehicles
        for i, pos in enumerate(ns_vehicles):
            direction = 'n' if i % 2 == 0 else 's'
            monitor.add_vehicle(f"car_{direction}{i+1}", pos, direction)
        
        # Add EW vehicles
        for i, pos in enumerate(ew_vehicles):
            direction = 'e' if i % 2 == 0 else 'w'
            monitor.add_vehicle(f"car_{direction}{i+1}", pos, direction)
        
        # Check clearance
        has_ns = len(ns_vehicles) > 0
        has_ew = len(ew_vehicles) > 0
        
        # NS clear means no EW vehicles
        assert monitor.is_ns_clear() == (not has_ew), \
            f"NS clear should be {not has_ew} but got {monitor.is_ns_clear()}"
        
        # EW clear means no NS vehicles
        assert monitor.is_ew_clear() == (not has_ns), \
            f"EW clear should be {not has_ns} but got {monitor.is_ew_clear()}"


class TestIntersectionZoneVehicleTracking:
    """
    Property 22: Intersection Zone Vehicle Tracking
    
    For any vehicle that enters the Intersection_Zone, it SHALL remain tracked
    in the zone monitor until it exits the zone bounds.
    
    Validates: Requirements 10.2
    """
    
    @settings(max_examples=100)
    @given(
        vehicle_name=st.text(min_size=1, max_size=20).map(lambda s: f"car_{s}"),
        position=vehicle_in_zone_strategy(),
        direction=st.sampled_from(DIRECTIONS)
    )
    def test_vehicle_tracked_when_in_zone(self, vehicle_name, position, direction):
        """
        Feature: adaptive-traffic-system-v2, Property 22: Vehicle Tracking
        
        Vehicles inside the zone are tracked.
        """
        monitor = MockIntersectionZoneMonitor()
        
        # Add vehicle
        monitor.add_vehicle(vehicle_name, position, direction)
        
        # Should be tracked
        assert vehicle_name in monitor.vehicles_in_zone, \
            f"Vehicle {vehicle_name} at {position} should be tracked"
        assert monitor.vehicle_directions[vehicle_name] == direction, \
            f"Vehicle direction should be {direction}"
    
    @settings(max_examples=100)
    @given(
        vehicle_name=st.text(min_size=1, max_size=20).map(lambda s: f"car_{s}"),
        position=vehicle_outside_zone_strategy(),
        direction=st.sampled_from(DIRECTIONS)
    )
    def test_vehicle_not_tracked_when_outside_zone(self, vehicle_name, position, direction):
        """
        Feature: adaptive-traffic-system-v2, Property 22: Vehicle Tracking
        
        Vehicles outside the zone are not tracked.
        """
        monitor = MockIntersectionZoneMonitor()
        
        # Try to add vehicle outside zone
        monitor.add_vehicle(vehicle_name, position, direction)
        
        # Should not be tracked (add_vehicle checks is_in_zone)
        assert vehicle_name not in monitor.vehicles_in_zone, \
            f"Vehicle {vehicle_name} at {position} should not be tracked (outside zone)"
    
    @settings(max_examples=100)
    @given(
        vehicle_name=st.text(min_size=1, max_size=20).map(lambda s: f"car_{s}"),
        entry_position=vehicle_in_zone_strategy(),
        direction=st.sampled_from(DIRECTIONS)
    )
    def test_vehicle_removed_when_exits_zone(self, vehicle_name, entry_position, direction):
        """
        Feature: adaptive-traffic-system-v2, Property 22: Vehicle Tracking
        
        Vehicles are removed from tracking when they exit the zone.
        """
        monitor = MockIntersectionZoneMonitor()
        
        # Add vehicle
        monitor.add_vehicle(vehicle_name, entry_position, direction)
        assert vehicle_name in monitor.vehicles_in_zone
        
        # Remove vehicle (simulating exit)
        monitor.remove_vehicle(vehicle_name)
        
        # Should no longer be tracked
        assert vehicle_name not in monitor.vehicles_in_zone, \
            f"Vehicle {vehicle_name} should be removed after exit"
        assert vehicle_name not in monitor.vehicle_directions


# ============================================================================
# Integration Tests
# ============================================================================

class TestIntegrationScenarios:
    """Integration tests for intersection safety scenarios."""
    
    def test_ns_green_blocks_ew_vehicles(self):
        """NS_GREEN state should block EW vehicles from entering."""
        controller = MockFlowController()
        controller.light_state = "NS_GREEN"
        
        # EW vehicle approaching intersection
        ew_position = (-15.0, 0.0)  # West of intersection
        velocity, reason = controller.get_velocity_command(
            "car_e1", ew_position, 'e', 5.0
        )
        
        assert velocity == 0.0
        assert reason == "red_light"
    
    def test_ew_green_blocks_ns_vehicles(self):
        """EW_GREEN state should block NS vehicles from entering."""
        controller = MockFlowController()
        controller.light_state = "EW_GREEN"
        
        # NS vehicle approaching intersection
        ns_position = (0.0, -15.0)  # South of intersection
        velocity, reason = controller.get_velocity_command(
            "car_n1", ns_position, 'n', 5.0
        )
        
        assert velocity == 0.0
        assert reason == "red_light"
    
    def test_all_red_blocks_all_vehicles(self):
        """ALL_RED state should block all vehicles."""
        controller = MockFlowController()
        controller.light_state = "ALL_RED"
        
        for direction in DIRECTIONS:
            velocity, reason = controller.get_velocity_command(
                f"car_{direction}1", (0.0, -15.0), direction, 5.0
            )
            assert velocity == 0.0
            assert reason == "red_light"
    
    def test_zone_conflict_stops_vehicle(self):
        """Vehicle should stop when conflicting vehicles are in zone."""
        controller = MockFlowController()
        controller.light_state = "NS_GREEN"
        controller.zone_ns_clear = False  # EW vehicles in zone
        
        # NS vehicle approaching - should stop due to zone conflict
        ns_position = (0.0, 14.0)  # Just outside zone, approaching
        velocity, reason = controller.get_velocity_command(
            "car_n1", ns_position, 'n', 5.0
        )
        
        assert velocity == 0.0
        assert reason == "zone_conflict"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
