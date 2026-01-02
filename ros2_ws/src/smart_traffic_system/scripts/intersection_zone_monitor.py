#!/usr/bin/env python3
"""
Intersection Zone Monitor for Smart Traffic System

This module monitors vehicles in the intersection zone to ensure safe phase changes.
It tracks vehicles entering/exiting the zone and publishes clearance status.

Requirements: 1.4, 1.6, 10.1, 10.2, 10.3
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import String, Bool
import json
import time


class IntersectionZoneMonitor(Node):
    """
    Monitors vehicles in the intersection zone to ensure safe phase changes.
    
    The intersection zone is defined as the area where roads cross
    (approximately -13m to +13m in both X and Y axes).
    
    This component:
    - Subscribes to intersection zone sensor
    - Tracks vehicles entering/exiting the zone
    - Publishes zone clearance status
    - Logs warnings for stuck vehicles
    """
    
    # Intersection zone boundaries (matching stop lines)
    ZONE_BOUNDS = {
        'x_min': -13.0,
        'x_max': 13.0,
        'y_min': -13.0,
        'y_max': 13.0
    }
    
    # Time threshold for stuck vehicle warning
    STUCK_THRESHOLD = 10.0  # seconds
    
    def __init__(self):
        super().__init__('intersection_zone_monitor')
        
        # Track vehicles currently in the zone: {vehicle_name: entry_time}
        self.vehicles_in_zone = {}
        
        # Track vehicle directions for conflict detection
        self.vehicle_directions = {}  # {vehicle_name: 'n'/'s'/'e'/'w'}
        
        # Subscribe to intersection zone sensor
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/intersection_zone',
            self.zone_sensor_callback,
            10
        )
        
        # Publisher for zone status (JSON format with detailed info)
        self.zone_status_pub = self.create_publisher(
            String,
            '/intersection/zone_status',
            10
        )
        
        # Publisher for simple clearance flag
        self.zone_clear_pub = self.create_publisher(
            Bool,
            '/intersection/is_clear',
            10
        )
        
        # Publisher for NS clearance (clear for north-south traffic)
        self.ns_clear_pub = self.create_publisher(
            Bool,
            '/intersection/ns_clear',
            10
        )
        
        # Publisher for EW clearance (clear for east-west traffic)
        self.ew_clear_pub = self.create_publisher(
            Bool,
            '/intersection/ew_clear',
            10
        )
        
        # Timer for periodic status publishing and stuck vehicle check
        self.create_timer(0.1, self.publish_status)  # 10 Hz
        
        self.get_logger().info('Intersection Zone Monitor initialized')
        self.get_logger().info(f'Zone bounds: X[{self.ZONE_BOUNDS["x_min"]}, {self.ZONE_BOUNDS["x_max"]}], '
                               f'Y[{self.ZONE_BOUNDS["y_min"]}, {self.ZONE_BOUNDS["y_max"]}]')
    
    def zone_sensor_callback(self, msg):
        """
        Process intersection zone sensor data.
        
        Updates the list of vehicles currently in the zone and
        tracks entry times for stuck vehicle detection.
        
        Args:
            msg: LogicalCameraImage message containing detected models
        """
        current_time = time.time()
        current_vehicles = set()
        
        for model in msg.models:
            # Only track vehicles (models starting with 'car_')
            if model.name.startswith('car_'):
                vehicle_name = model.name
                current_vehicles.add(vehicle_name)
                
                # Extract direction from vehicle name (e.g., 'n' from 'car_n1')
                direction = vehicle_name.split('_')[1][0]
                self.vehicle_directions[vehicle_name] = direction
                
                # Track entry time for new vehicles
                if vehicle_name not in self.vehicles_in_zone:
                    self.vehicles_in_zone[vehicle_name] = current_time
                    self.get_logger().info(f'Vehicle {vehicle_name} entered intersection zone')
                else:
                    # Check for stuck vehicles
                    entry_time = self.vehicles_in_zone[vehicle_name]
                    time_in_zone = current_time - entry_time
                    
                    if time_in_zone > self.STUCK_THRESHOLD:
                        self.get_logger().warning(
                            f'Vehicle {vehicle_name} stuck in intersection for '
                            f'{time_in_zone:.1f} seconds'
                        )
        
        # Remove vehicles that have exited the zone
        exited_vehicles = set(self.vehicles_in_zone.keys()) - current_vehicles
        for vehicle_name in exited_vehicles:
            entry_time = self.vehicles_in_zone[vehicle_name]
            transit_time = current_time - entry_time
            del self.vehicles_in_zone[vehicle_name]
            if vehicle_name in self.vehicle_directions:
                del self.vehicle_directions[vehicle_name]
            self.get_logger().info(
                f'Vehicle {vehicle_name} exited intersection zone '
                f'(transit time: {transit_time:.1f}s)'
            )
    
    def is_zone_clear(self):
        """
        Check if the intersection zone is completely clear.
        
        Returns:
            bool: True if no vehicles are in the zone
        """
        return len(self.vehicles_in_zone) == 0
    
    def is_clear_for_direction(self, directions):
        """
        Check if the intersection zone is clear of vehicles from specified directions.
        
        This is used to determine if it's safe to give green to a particular
        direction - we need to ensure no conflicting vehicles are in the zone.
        
        Args:
            directions: List of direction codes to check (e.g., ['n', 's'])
        
        Returns:
            bool: True if no vehicles from the specified directions are in the zone
        """
        for vehicle_name in self.vehicles_in_zone:
            direction = self.vehicle_directions.get(vehicle_name)
            if direction in directions:
                return False
        return True
    
    def is_ns_clear(self):
        """
        Check if zone is clear for north-south traffic.
        
        Returns True if no east-west vehicles are in the zone,
        meaning it's safe to give green to NS traffic.
        
        Returns:
            bool: True if safe for NS traffic
        """
        return self.is_clear_for_direction(['e', 'w'])
    
    def is_ew_clear(self):
        """
        Check if zone is clear for east-west traffic.
        
        Returns True if no north-south vehicles are in the zone,
        meaning it's safe to give green to EW traffic.
        
        Returns:
            bool: True if safe for EW traffic
        """
        return self.is_clear_for_direction(['n', 's'])
    
    def get_vehicles_in_zone(self):
        """
        Get list of vehicles currently in the intersection zone.
        
        Returns:
            list: List of vehicle names in the zone
        """
        return list(self.vehicles_in_zone.keys())
    
    def get_zone_status(self):
        """
        Get detailed zone status as a dictionary.
        
        Returns:
            dict: Zone status including clearance flags and vehicle list
        """
        return {
            'is_clear': self.is_zone_clear(),
            'ns_clear': self.is_ns_clear(),
            'ew_clear': self.is_ew_clear(),
            'vehicles_in_zone': self.get_vehicles_in_zone(),
            'vehicle_count': len(self.vehicles_in_zone),
            'timestamp': time.time()
        }
    
    def publish_status(self):
        """
        Publish current zone status to all topics.
        """
        # Publish detailed status as JSON
        status = self.get_zone_status()
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.zone_status_pub.publish(status_msg)
        
        # Publish simple clearance flags
        clear_msg = Bool()
        clear_msg.data = status['is_clear']
        self.zone_clear_pub.publish(clear_msg)
        
        ns_clear_msg = Bool()
        ns_clear_msg.data = status['ns_clear']
        self.ns_clear_pub.publish(ns_clear_msg)
        
        ew_clear_msg = Bool()
        ew_clear_msg.data = status['ew_clear']
        self.ew_clear_pub.publish(ew_clear_msg)


def main(args=None):
    rclpy.init(args=args)
    monitor = IntersectionZoneMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
