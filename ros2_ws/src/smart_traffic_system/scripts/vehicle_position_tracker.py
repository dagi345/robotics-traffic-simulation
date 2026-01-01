#!/usr/bin/env python3
"""
Vehicle Position Tracker for Smart Traffic System

This module tracks vehicle positions using logical camera sensor data
and provides distance calculations between vehicles in the same lane.

Requirements: 2.1, 2.2, 2.5, 5.1, 5.2
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage
import math


class VehiclePositionTracker(Node):
    """
    Tracks positions of all vehicles using sensor data from logical cameras.
    
    Subscribes to all four logical camera sensors and maintains a dictionary
    of vehicle positions. Provides methods to calculate distances between
    vehicles in the same lane.
    """
    
    def __init__(self):
        super().__init__('vehicle_position_tracker')
        
        # Dictionary to store vehicle positions: {vehicle_name: (x, y, z)}
        self.vehicle_positions = {}
        
        # Map vehicle names to their lanes
        self.vehicle_lanes = {}
        
        # Initialize vehicle lane mappings
        # North lane: vehicles moving south (decreasing y)
        for i in range(1, 5):
            self.vehicle_lanes[f'car_n{i}'] = 'north'
        
        # South lane: vehicles moving north (increasing y)
        for i in range(1, 5):
            self.vehicle_lanes[f'car_s{i}'] = 'south'
        
        # East lane: vehicles moving west (decreasing x)
        for i in range(1, 5):
            self.vehicle_lanes[f'car_e{i}'] = 'east'
        
        # West lane: vehicles moving east (increasing x)
        for i in range(1, 5):
            self.vehicle_lanes[f'car_w{i}'] = 'west'
        
        # Subscribe to all four logical camera sensors
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/north',
            self.sensor_callback_north,
            10
        )
        
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/south',
            self.sensor_callback_south,
            10
        )
        
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/east',
            self.sensor_callback_east,
            10
        )
        
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/west',
            self.sensor_callback_west,
            10
        )
        
        self.get_logger().info('Vehicle Position Tracker initialized')
    
    def sensor_callback_north(self, msg):
        """Process sensor data from north sensor (detects southbound traffic)"""
        self._process_sensor_data(msg)
    
    def sensor_callback_south(self, msg):
        """Process sensor data from south sensor (detects northbound traffic)"""
        self._process_sensor_data(msg)
    
    def sensor_callback_east(self, msg):
        """Process sensor data from east sensor (detects westbound traffic)"""
        self._process_sensor_data(msg)
    
    def sensor_callback_west(self, msg):
        """Process sensor data from west sensor (detects eastbound traffic)"""
        self._process_sensor_data(msg)
    
    def _process_sensor_data(self, msg):
        """
        Process sensor data and update vehicle positions.
        
        Args:
            msg: LogicalCameraImage message containing detected models
        """
        for model in msg.models:
            # Only track vehicles (models starting with 'car_')
            if model.name.startswith('car_'):
                # Extract position from sensor data
                pos = model.pose.position
                self.vehicle_positions[model.name] = (pos.x, pos.y, pos.z)
    
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
            distance = None
            
            if my_lane == 'north':
                # North lane: vehicles move south (decreasing y)
                # Vehicle ahead has lower y coordinate
                if other_pos[1] < my_pos[1]:
                    distance = my_pos[1] - other_pos[1]
            
            elif my_lane == 'south':
                # South lane: vehicles move north (increasing y)
                # Vehicle ahead has higher y coordinate
                if other_pos[1] > my_pos[1]:
                    distance = other_pos[1] - my_pos[1]
            
            elif my_lane == 'east':
                # East lane: vehicles move west (decreasing x)
                # Vehicle ahead has lower x coordinate
                if other_pos[0] < my_pos[0]:
                    distance = my_pos[0] - other_pos[0]
            
            elif my_lane == 'west':
                # West lane: vehicles move east (increasing x)
                # Vehicle ahead has higher x coordinate
                if other_pos[0] > my_pos[0]:
                    distance = other_pos[0] - my_pos[0]
            
            # Update minimum distance if this vehicle is closer
            if distance is not None and distance < min_distance:
                min_distance = distance
        
        return min_distance
    
    def get_vehicle_count(self, direction):
        """
        Get the count of vehicles currently tracked in a specific direction.
        
        Args:
            direction: Direction to count ('north', 'south', 'east', 'west')
            
        Returns:
            int: Number of vehicles in that direction
        """
        count = 0
        for vehicle_name in self.vehicle_positions.keys():
            if self.vehicle_lanes.get(vehicle_name) == direction:
                count += 1
        return count


def main(args=None):
    rclpy.init(args=args)
    tracker = VehiclePositionTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
