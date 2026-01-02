#!/usr/bin/env python3
"""
Enhanced Traffic Flow Controller with Intersection Zone Awareness and Yellow Light

This module controls vehicle velocities based on traffic light states,
collision avoidance, and intersection zone safety.

Requirements: 1.1-1.3, 4.1-4.5, 5.1-5.5, 6.1-6.5, 7.3, 7.4, 10.5
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from ros_gz_interfaces.msg import LogicalCameraImage
import random
import json
import time
import math
import time

class DriverProfile:
    """unique characteristics for each driver/car."""
    def __init__(self, desired_speed, aggressiveness, time_gap):
        self.desired_speed = desired_speed
        self.aggressiveness = aggressiveness  # affects speed fluctuation
        self.time_gap = time_gap  # seconds
        self.current_speed_target = desired_speed
        self.last_update_time = time.time()



class MultiTrafficFlowControl(Node):
    def __init__(self):
        super().__init__('multi_traffic_flow_control')
        
        self.declare_parameter('min_speed', 40.0)
        self.declare_parameter('max_speed', 42.0)
        self.min_s = self.get_parameter('min_speed').value
        self.max_s = self.get_parameter('max_speed').value
        
        # Collision avoidance constants
        self.SAFE_FOLLOWING_DISTANCE = 5.0  # meters
        self.STOP_DISTANCE = 3.0  # meters - emergency stop threshold
        
        # Yellow light slowdown factor (Requirement 7.3, 7.4)
        self.YELLOW_SLOWDOWN_FACTOR = 0.5  # 50% speed reduction on yellow
        
        # Intersection zone boundaries (matching stop lines at ±13m)
        self.INTERSECTION_ENTRY = 13.0  # Distance from center where vehicles enter
        self.INTERSECTION_SLOWDOWN = 18.0  # Distance to start slowing if zone not clear

        self.car_publishers = {}
        # self.speeds = {}  # REPLACED by driver_profiles
        self.driver_profiles = {}
        self.directions = ['n', 's', 'e', 'w']
        
        # Vehicle counts per direction (requested: 10, 12, 8, 13)
        self.vehicle_counts = {
            'n': 10,  # Northbound
            's': 12,  # Southbound
            'e': 8,   # Eastbound
            'w': 13   # Westbound
        }

        
        # State tracking
        self.light_state = "NS_GREEN"
        # Track which cars are in which detection zone
        self.in_zones = {'n': set(), 's': set(), 'e': set(), 'w': set()}
        
        # CRITICAL: Traffic light states per direction
        # Vehicles look at the light ACROSS the intersection
        # Southbound cars → look at NORTH light
        # Northbound cars → look at SOUTH light
        # Westbound cars → look at EAST light
        # Eastbound cars → look at WEST light
        # Initialize to RED as safe default, will be updated by traffic manager
        self.light_states = {
            'north': 'RED',  # Controls southbound vehicles
            'south': 'RED',  # Controls northbound vehicles
            'east': 'RED',   # Controls westbound vehicles
            'west': 'RED',   # Controls eastbound vehicles
        }
        self.light_states_received = {
            'north': False,
            'south': False,
            'east': False,
            'west': False,
        }
        
        # Stop line positions (matching world file coordinates)
        # World file has stop lines at ±13 (see intersection.world line 79-82)
        # Crosswalks are at ±10 to ±13
        self.STOP_LINE_NORTH = 13.0  # Y coordinate for southbound stop (matches world)
        self.STOP_LINE_SOUTH = -13.0  # Y coordinate for northbound stop (matches world)
        self.STOP_LINE_EAST = 13.0   # X coordinate for westbound stop (matches world)
        self.STOP_LINE_WEST = -13.0  # X coordinate for eastbound stop (matches world)
        
        # Intersection zone status
        self.zone_is_clear = True
        self.zone_ns_clear = True
        self.zone_ew_clear = True
        self.vehicles_in_zone = []
        
        # Track previous velocities for logging
        self.previous_velocities = {}
        
        # Vehicle position tracking (integrated directly into this node)
        self.vehicle_positions = {}  # {vehicle_name: (x, y, z)}
        self.vehicle_lanes = self._initialize_lane_mapping()


        for d in self.directions:
            for i in range(1, self.vehicle_counts[d] + 1):
                name = f'car_{d}{i}'
                topic = f'/model/{name}/cmd_vel'
                self.car_publishers[name] = self.create_publisher(Twist, topic, 10)
                
                # Assign unique driver profile
                # Speed: Gaussian distribution with HIGH variance for realistic heterogeneity
                # Aggressiveness: 0.1 to 0.8 (some very inconsistent drivers)
                # Time gap: 1.0s to 4.0s (some tailgaters, some cautious)
                base_speed = random.uniform(self.min_s, self.max_s)
                
                # Allow significant outliers (some "grandmas", some "speeders")
                # Increase sigma to 5.0 for visible speed differences
                variance = random.gauss(0, 5.0) 
                desired_speed = max(20.0, base_speed + variance) # Cap min speed at 20 so they don't stop
                
                self.driver_profiles[name] = DriverProfile(
                    desired_speed=desired_speed,
                    aggressiveness=random.uniform(0.1, 0.6), # More fluctuation
                    time_gap=random.uniform(1.2, 4.0)        # Wider gap variation
                )

        # Subscriptions for traffic light state
        self.create_subscription(
            String, '/traffic_lights/state',
            self.light_cb, 10)
        
        # Subscribe to individual traffic light states
        # These control which vehicles can proceed
        self.create_subscription(
            String, '/traffic/light/north',
            lambda msg: self.individual_light_cb(msg, 'north'), 10)
        self.create_subscription(
            String, '/traffic/light/south',
            lambda msg: self.individual_light_cb(msg, 'south'), 10)
        self.create_subscription(
            String, '/traffic/light/east',
            lambda msg: self.individual_light_cb(msg, 'east'), 10)
        self.create_subscription(
            String, '/traffic/light/west',
            lambda msg: self.individual_light_cb(msg, 'west'), 10)
        
        # Subscriptions for lane sensors
        self.create_subscription(
            LogicalCameraImage, '/sensors/north',
            lambda msg: self.sensor_cb(msg, 's'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/south',
            lambda msg: self.sensor_cb(msg, 'n'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/east',
            lambda msg: self.sensor_cb(msg, 'w'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/west',
            lambda msg: self.sensor_cb(msg, 'e'), 10)
        
        # Subscriptions for intersection zone status
        self.create_subscription(
            String, '/intersection/zone_status',
            self.zone_status_cb, 10)
        self.create_subscription(
            Bool, '/intersection/ns_clear',
            self.ns_clear_cb, 10)
        self.create_subscription(
            Bool, '/intersection/ew_clear',
            self.ew_clear_cb, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Enhanced Traffic Flow Controller with Zone Awareness Activated')

    def light_cb(self, msg):
        self.light_state = msg.data
    
    def individual_light_cb(self, msg, direction):
        """
        Callback for individual traffic light states.
        
        These topics are published by smart_traffic_manager.py and received
        directly via ROS2 (no Gazebo bridge needed since both nodes run in
        the same ROS2 context).
        
        Args:
            msg: String message with light color ("RED", "YELLOW", "GREEN")
            direction: Which light this is ('north', 'south', 'east', 'west')
        """
        old_state = self.light_states.get(direction, 'UNKNOWN')
        self.light_states[direction] = msg.data
        self.light_states_received[direction] = True
        
        # Only log when state changes to reduce log spam
        if old_state != msg.data:
            self.get_logger().info(f"Light state changed: {direction} {old_state} -> {msg.data}")
        
        # Log if this is the first time receiving this light state
        if not any(self.light_states_received.values()):
            self.get_logger().info(f"Received first light state: {direction} = {msg.data}")
    
    def _initialize_lane_mapping(self):
        """
        Initialize vehicle-to-lane mapping.
        
        Returns:
            dict: Mapping of vehicle names to lane directions
        """
        vehicle_lanes = {}
        
        # North lane: vehicles moving south (decreasing y)
        for i in range(1, self.vehicle_counts['n'] + 1):
            vehicle_lanes[f'car_n{i}'] = 'north'
        
        # South lane: vehicles moving north (increasing y)
        for i in range(1, self.vehicle_counts['s'] + 1):
            vehicle_lanes[f'car_s{i}'] = 'south'
        
        # East lane: vehicles moving west (decreasing x)
        for i in range(1, self.vehicle_counts['e'] + 1):
            vehicle_lanes[f'car_e{i}'] = 'east'
        
        # West lane: vehicles moving east (increasing x)
        for i in range(1, self.vehicle_counts['w'] + 1):
            vehicle_lanes[f'car_w{i}'] = 'west'
        
        return vehicle_lanes
    
    def apply_velocity(self, vehicle_name, velocity):
        """
        Apply velocity to vehicle using standard command.
        
        CRITICAL: Per CORRECT_VEHICLE_DIRECTIONS.md, all vehicles use 
        negative linear.y to move forward relative to their own frame.
        Direction is handled by the world pose rotation.
        """
        msg = Twist()
        msg.linear.y = -velocity
        if vehicle_name in self.car_publishers:
            self.car_publishers[vehicle_name].publish(msg)
        else:
            self.get_logger().error(f"No publisher for vehicle: {vehicle_name}")

    def sensor_cb(self, msg, road_dir):
        """
        Update the set of cars currently in this road's detection zone.
        Also update vehicle positions for collision avoidance.
        """
        current_in_view = {model.name for model in msg.models}
        self.in_zones[road_dir] = current_in_view
        
        # Update vehicle positions directly
        for model in msg.models:
            if model.name.startswith('car_'):
                pos = model.pose.position
                self.vehicle_positions[model.name] = (pos.x, pos.y, pos.z)
    
    def zone_status_cb(self, msg):
        """Process detailed zone status from Intersection Zone Monitor."""
        try:
            status = json.loads(msg.data)
            self.zone_is_clear = status.get('is_clear', True)
            self.zone_ns_clear = status.get('ns_clear', True)
            self.zone_ew_clear = status.get('ew_clear', True)
            self.vehicles_in_zone = status.get('vehicles_in_zone', [])
        except json.JSONDecodeError:
            pass
    
    def ns_clear_cb(self, msg):
        """Process NS clear flag."""
        self.zone_ns_clear = msg.data
    
    def ew_clear_cb(self, msg):
        """Process EW clear flag."""
        self.zone_ew_clear = msg.data
    
    def _get_vehicle_position(self, vehicle_name):
        """Get vehicle position from integrated tracker."""
        return self.vehicle_positions.get(vehicle_name)
    
    def _get_controlling_light(self, vehicle_name):
        """
        Get the traffic light that controls this vehicle.
        
        CRITICAL RULE: Vehicles look at the light ACROSS the intersection.
        - Southbound vehicles (car_s*) → NORTH light
        - Northbound vehicles (car_n*) → SOUTH light
        - Westbound vehicles (car_w*) → EAST light
        - Eastbound vehicles (car_e*) → WEST light
        
        Args:
            vehicle_name: Name like 'car_n1', 'car_s2', etc.
        
        Returns:
            str: Light state ('RED', 'YELLOW', 'GREEN')
        """
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return 'RED'  # Safe default
        
        # Map vehicle direction to controlling light
        light_mapping = {
            's': 'north',  # Southbound → look at NORTH light
            'n': 'south',  # Northbound → look at SOUTH light
            'w': 'east',   # Westbound → look at EAST light
            'e': 'west',   # Eastbound → look at WEST light
        }
        
        controlling_light = light_mapping.get(direction, 'north')
        return self.light_states.get(controlling_light, 'RED')
    
    def _is_at_stop_line(self, vehicle_name):
        """
        Check if vehicle is at or approaching the stop line.
        
        Stop lines are positioned at ±13m from intersection center.
        Vehicles should stop BEFORE reaching the stop line to prevent entering intersection.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle is approaching/at stop line and should check light
        """
        pos = self._get_vehicle_position(vehicle_name)
        if pos is None:
            # Vehicle position not yet tracked
            return False
        
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return False
        
        x, y, z = pos
        
        # Check if vehicle is approaching stop line
        # We detect vehicles from 20m before stop line to 5m past stop line
        # This ensures vehicles stop BEFORE entering intersection
        if direction == 's':  # Southbound (moving south, decreasing Y)
            # Stop line at Y=13, detect from Y=20 to Y=8 (before intersection)
            # Vehicle should stop if Y >= 8 and Y <= 20
            at_line = y >= 8.0 and y <= 20.0
            if at_line:
                self.get_logger().debug(
                    f"{vehicle_name} (southbound): Y={y:.2f}, at_line={at_line}, light={self._get_controlling_light(vehicle_name)}")
            return at_line
        elif direction == 'n':  # Northbound (moving north, increasing Y)
            # Stop line at Y=-13, detect from Y=-20 to Y=-8
            at_line = y <= -8.0 and y >= -20.0
            if at_line:
                self.get_logger().debug(
                    f"{vehicle_name} (northbound): Y={y:.2f}, at_line={at_line}, light={self._get_controlling_light(vehicle_name)}")
            return at_line
        elif direction == 'w':  # Westbound (moving west, decreasing X)
            # Stop line at X=13, detect from X=20 to X=8
            at_line = x >= 8.0 and x <= 20.0
            if at_line:
                self.get_logger().debug(
                    f"{vehicle_name} (westbound): X={x:.2f}, at_line={at_line}, light={self._get_controlling_light(vehicle_name)}")
            return at_line
        elif direction == 'e':  # Eastbound (moving east, increasing X)
            # Stop line at X=-13, detect from X=-20 to X=-8
            at_line = x <= -8.0 and x >= -20.0
            if at_line:
                self.get_logger().debug(
                    f"{vehicle_name} (eastbound): X={x:.2f}, at_line={at_line}, light={self._get_controlling_light(vehicle_name)}")
            return at_line
        
        return False
    
    def _is_in_intersection(self, vehicle_name):
        """
        Check if vehicle is already inside the intersection.
        
        Intersection zone is approximately ±10m from center.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle is in intersection
        """
        pos = self._get_vehicle_position(vehicle_name)
        if pos is None:
            return False
        
        x, y, z = pos
        
        # Intersection is roughly ±10m from center
        in_intersection = abs(x) < 10.0 and abs(y) < 10.0
        return in_intersection
    
    def _is_approaching_stop_line(self, vehicle_name):
        """
        Check if vehicle is approaching the stop line (wider detection zone).
        
        This is used to slow down vehicles before they reach the stop line.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle is approaching stop line
        """
        pos = self._get_vehicle_position(vehicle_name)
        if pos is None:
            return False
        
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return False
        
        x, y, z = pos
        
        # Wider detection zone: 25m before stop line to 5m past stop line
        if direction == 's':  # Southbound
            return y >= 5.0 and y <= 25.0
        elif direction == 'n':  # Northbound
            return y <= -5.0 and y >= -25.0
        elif direction == 'w':  # Westbound
            return x >= 5.0 and x <= 25.0
        elif direction == 'e':  # Eastbound
            return x <= -5.0 and x >= -25.0
        
        return False
    
    def _is_in_detection_zone(self, vehicle_name):
        """
        Check if vehicle is in a detection zone (using sensor data).
        
        This is a fallback when position data isn't available.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle is in a detection zone
        """
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return False
        
        if direction in self.in_zones:
            return vehicle_name in self.in_zones[direction]
        
        return False
    
    def _is_near_intersection_simple(self, vehicle_name):
        """
        Simple check if vehicle is near intersection center.
        
        This is a CRITICAL FALLBACK that works without sensor data.
        Uses position data if available, otherwise uses a time-based heuristic.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle is likely near intersection
        """
        # Try to use position data if available
        pos = self._get_vehicle_position(vehicle_name)
        if pos is not None:
            x, y, z = pos
            # Check if within 15m of intersection center (0, 0)
            distance = (x**2 + y**2)**0.5
            return distance < 15.0
        
        # Fallback: If no position data, assume vehicles approaching intersection
        # after some time (vehicles start 25-35m away, move at 3-5 m/s)
        # After ~5-10 seconds, they're likely near intersection
        # This is a safety mechanism - better to stop early than collide
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return False
        
        # If vehicle is in a detection zone, it's definitely near intersection
        if direction in self.in_zones:
            if vehicle_name in self.in_zones[direction]:
                return True
        
        # Conservative: If we have no data, don't assume position
        # Let other checks handle it
        return False
    
    def _should_stop_for_light(self, vehicle_name):
        """
        Determine if vehicle should stop for traffic light.
        
        STRICT ENFORCEMENT:
        - MUST stop on RED if in detection zone or at stop line
        - MUST stop on YELLOW if in detection zone or at stop line (unless already in intersection)
        - MAY proceed on GREEN
        - Vehicles already in intersection should continue through
        
        Uses detection zone as fallback when position data isn't available.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            bool: True if vehicle must stop
        """
        # Get the controlling light state
        light_state = self._get_controlling_light(vehicle_name)
        
        # If vehicle is already in intersection, don't stop (let it clear)
        if self._is_in_intersection(vehicle_name):
            return False
        
        # Check if vehicle is in a detection zone (fallback when position data missing)
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            direction = None
        
        in_detection_zone = False
        if direction and direction in self.in_zones:
            in_detection_zone = vehicle_name in self.in_zones[direction]
        
        # Check if at stop line (requires position data)
        at_stop_line = self._is_at_stop_line(vehicle_name)
        
        # If we haven't received light states yet, be conservative and stop
        # (but only if in detection zone or at stop line)
        controlling_direction = self._get_controlling_light_direction(vehicle_name)
        if not self.light_states_received.get(controlling_direction, False):
            if in_detection_zone or at_stop_line:
                self.get_logger().debug(
                    f"{vehicle_name}: No light state received yet, stopping (in_zone={in_detection_zone}, at_line={at_stop_line})")
                return True
        
        # RED: Always stop if in detection zone OR at stop line
        if light_state == 'RED':
            if in_detection_zone or at_stop_line:
                self.get_logger().info(
                    f"{vehicle_name}: STOPPING for RED light (in_zone={in_detection_zone}, at_line={at_stop_line})")
                return True
        
        # YELLOW: Stop if in detection zone OR at stop line (safety - don't enter on yellow)
        if light_state == 'YELLOW':
            if in_detection_zone or at_stop_line:
                self.get_logger().info(
                    f"{vehicle_name}: STOPPING for YELLOW light (in_zone={in_detection_zone}, at_line={at_stop_line})")
                return True
        
        # GREEN: Don't stop for light
        return False
    
    def _get_controlling_light_direction(self, vehicle_name):
        """
        Get the direction name of the controlling light for this vehicle.
        
        Args:
            vehicle_name: Name of the vehicle
        
        Returns:
            str: Direction name ('north', 'south', 'east', 'west')
        """
        try:
            direction = vehicle_name.split('_')[1][0]
        except (IndexError, AttributeError):
            return 'north'  # Safe default
        
        # Map vehicle direction to controlling light
        light_mapping = {
            's': 'north',  # Southbound → look at NORTH light
            'n': 'south',  # Northbound → look at SOUTH light
            'w': 'east',   # Westbound → look at EAST light
            'e': 'west',   # Eastbound → look at WEST light
        }
        
        return light_mapping.get(direction, 'north')
    
    def _calculate_lane_distance(self, lane, my_pos, other_pos):
        """
        Calculate distance to vehicle ahead in specific lane.
        
        Based on actual vehicle movements:
        - South lane: vehicles move south (decreasing Y), ahead = lower Y
        - North lane: vehicles move north (increasing Y), ahead = higher Y  
        - West lane: vehicles move west (decreasing X), ahead = lower X
        - East lane: vehicles move east (increasing X), ahead = higher X
        
        Args:
            lane: Lane direction ('north', 'south', 'east', 'west')
            my_pos: My vehicle position (x, y, z)
            other_pos: Other vehicle position (x, y, z)
        
        Returns:
            float: Distance if other vehicle is ahead, None otherwise
        """
        if lane == 'south':
            # Moving south (decreasing Y), vehicle ahead has lower Y
            if other_pos[1] < my_pos[1]:
                return my_pos[1] - other_pos[1]
        elif lane == 'north':
            # Moving north (increasing Y), vehicle ahead has higher Y
            if other_pos[1] > my_pos[1]:
                return other_pos[1] - my_pos[1]
        elif lane == 'west':
            # Moving west (decreasing X), vehicle ahead has lower X
            if other_pos[0] < my_pos[0]:
                return my_pos[0] - other_pos[0]
        elif lane == 'east':
            # Moving east (increasing X), vehicle ahead has higher X
            if other_pos[0] > my_pos[0]:
                return other_pos[0] - my_pos[0]
        return None
    
    def get_distance_to_vehicle_ahead(self, vehicle_name):
        """
        Calculate distance to the vehicle ahead in the same lane.
        
        Args:
            vehicle_name: Name of the vehicle to check
            
        Returns:
            float: Distance in meters to the vehicle ahead, or infinity if no vehicle ahead
        """
        # Return infinity if vehicle position is unknown
        if vehicle_name not in self.vehicle_positions:
            return float('inf')
        
        # Get vehicle's position and lane
        my_pos = self.vehicle_positions[vehicle_name]
        my_lane = self.vehicle_lanes.get(vehicle_name)
        
        # Return infinity if lane is unknown
        if my_lane is None:
            return float('inf')
        
        # Find the closest vehicle ahead in the same lane
        min_distance = float('inf')
        
        for other_name, other_pos in self.vehicle_positions.items():
            # Skip self
            if other_name == vehicle_name:
                continue
            
            # Skip vehicles in different lanes
            if self.vehicle_lanes.get(other_name) != my_lane:
                continue
            
            # Calculate distance based on lane direction
            distance = self._calculate_lane_distance(my_lane, my_pos, other_pos)
            if distance is not None and distance > 0:
                min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _get_distance_to_intersection(self, vehicle_name):
        """
        Calculate distance from vehicle to intersection center.
        
        Returns:
            float: Distance to intersection center, or infinity if unknown
        """
        pos = self._get_vehicle_position(vehicle_name)
        if pos is None:
            return float('inf')
        
        car_road = vehicle_name.split('_')[1][0]
        
        # Calculate distance based on direction
        if car_road in ['n', 's']:
            return abs(pos[1])  # Y distance to center
        else:  # 'e' or 'w'
            return abs(pos[0])  # X distance to center
    
    def _is_approaching_intersection(self, vehicle_name):
        """Check if vehicle is approaching the intersection zone."""
        distance = self._get_distance_to_intersection(vehicle_name)
        return distance < self.INTERSECTION_SLOWDOWN and distance > self.INTERSECTION_ENTRY
    
    def _should_stop_for_zone_conflict(self, vehicle_name):
        """
        Check if vehicle should stop due to conflicting vehicles in zone.
        
        This prevents vehicles from entering the intersection when
        conflicting traffic is still crossing.
        
        Requirements: 1.1, 1.2, 1.3
        """
        car_road = vehicle_name.split('_')[1][0]
        distance = self._get_distance_to_intersection(vehicle_name)
        
        # Only check if vehicle is close to intersection
        if distance > self.INTERSECTION_SLOWDOWN:
            return False
        
        # Check if there are conflicting vehicles in the zone
        if car_road in ['n', 's']:
            # NS vehicle - check if EW vehicles are in zone
            if not self.zone_ns_clear:
                # There are EW vehicles in the zone
                if distance < self.INTERSECTION_ENTRY + 2:
                    return True
        else:  # 'e' or 'w'
            # EW vehicle - check if NS vehicles are in zone
            if not self.zone_ew_clear:
                # There are NS vehicles in the zone
                if distance < self.INTERSECTION_ENTRY + 2:
                    return True
        
        return False

    def timer_callback(self):
        """
        Control vehicle velocities with STRICT traffic light compliance.
        
        PRIORITY ORDER:
        1. Traffic light (RED/YELLOW = STOP, GREEN = PROCEED)
        2. Collision avoidance
        3. Zone conflicts
        4. Normal speed
        
        Implements:
        - STRICT traffic light compliance (Requirements 4.1, 4.2)
        - Collision avoidance (Requirements 5.1, 5.2)
        - Intersection zone conflict prevention (Requirements 1.1, 1.2, 1.3)
        """
        for name, pub in self.car_publishers.items():
            msg = Twist()
            
            # PRIORITY 1: Traffic Light Compliance (HIGHEST PRIORITY)
            # Check if vehicle must stop for traffic light
            must_stop_for_light = self._should_stop_for_light(name)
            
            # Get distance to vehicle ahead for collision avoidance
            distance_ahead = self.get_distance_to_vehicle_ahead(name)
            
            # Check if vehicle should stop for zone conflict
            zone_conflict = self._should_stop_for_zone_conflict(name)
            
            # Determine velocity based on all conditions
            profile = self.driver_profiles[name]
            
            # 0. Update desired speed with small random fluctuations (Simulate human foot irregularity)
            if time.time() - profile.last_update_time > 1.0:
                # Fluctuate by +/- 5% proportional to aggressiveness
                fluctuation = profile.desired_speed * profile.aggressiveness * random.uniform(-0.1, 0.1)
                profile.current_speed_target = max(1.0, profile.desired_speed + fluctuation)
                profile.last_update_time = time.time()
                
            target_velocity = profile.current_speed_target
            reason = ""
            
            # PRIORITY 1: Traffic Light (MUST OBEY - HIGHEST PRIORITY)
            # CRITICAL SAFETY: Stop vehicles when their controlling light is RED/YELLOW
            light_state = self._get_controlling_light(name)
            in_intersection = self._is_in_intersection(name)
            
            # Calculate distance to stop line for smooth slowing
            # We don't have exact distance to stop line in this logic yet, 
            # effectively traffic light stop is treated as an obstacle at the stop line
            
            should_stop_for_light = False
            
            # AGGRESSIVE SAFETY: If light is RED, stop immediately (unless already in intersection)
            if light_state == 'RED':
                if not in_intersection:
                     should_stop_for_light = True
                     reason = "RED_LIGHT_STOP"
            
            # YELLOW: Stop if approaching intersection
            elif light_state == 'YELLOW':
                in_zone = self._is_in_detection_zone(name)
                near_intersection = self._is_near_intersection_simple(name)
                
                if (must_stop_for_light or in_zone or near_intersection) and not in_intersection:
                    should_stop_for_light = True
                    reason = "YELLOW_LIGHT_STOP"
            
            # GREEN: Don't stop for light
            elif must_stop_for_light:
                 should_stop_for_light = True
                 reason = f"traffic_light_{light_state}"

            if should_stop_for_light:
                # If we need to stop for light, we can treat it as '0' speed target
                # But to prevent instant stops, we should ideally decelerate
                # For now, we'll keep the strict stop if very close/in zone, 
                # but could smooth it if we knew distance to stop line.
                # Given strict requirements, we'll forceful stop.
                target_velocity = 0.0

            # PRIORITY 2: Car Following / Collision Avoidance (ACC)
            # Adjust speed based on vehicle ahead
            if distance_ahead != float('inf'):
                # Intelligent collision avoidance logic
                # Desired gap = stop_distance + (current_speed * time_gap)
                # But here we calculate a safe speed based on available distance
                
                # Effective distance available to drive into
                available_space = max(0.0, distance_ahead - self.STOP_DISTANCE)
                
                # Rule: allow 1 time_gap of speed. 
                # e.g. if 10m available and gap is 2s, safe speed is 5m/s.
                safe_following_speed = available_space / profile.time_gap
                
                if safe_following_speed < target_velocity:
                    target_velocity = safe_following_speed
                    reason = f"ACC (dist={distance_ahead:.1f}m)"
                    
            # PRIORITY 3: Zone conflict (prevent intersection collision)
            if zone_conflict:
                target_velocity = 0.0
                reason = "zone_conflict"
                
            # Final application
            self.apply_velocity(name, target_velocity)
            
            if reason and random.random() < 0.01: # Log sparingly
                 self.get_logger().info(f"{name}: Speed {target_velocity:.2f} ({reason})")

            # Log velocity changes (optional, adapted for new logic)
            if random.random() < 0.005:
                 self.get_logger().info(f"{name}: Speed {target_velocity:.2f} ({reason})")
    
    def _log_velocity_change(self, vehicle_name, velocity, reason):
        """Log vehicle control decisions when velocity changes."""
        previous = self.previous_velocities.get(vehicle_name, -1)
        
        if abs(velocity - previous) > 0.1:
            self.get_logger().debug(
                f"Vehicle {vehicle_name}: velocity={velocity:.2f} m/s, reason={reason}")
            self.previous_velocities[vehicle_name] = velocity


def main(args=None):
    rclpy.init(args=args)
    node = MultiTrafficFlowControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
