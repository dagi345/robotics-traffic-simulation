#!/usr/bin/env python3
"""
Unit tests for bridge configuration.
Feature: traffic-control-fix

Tests that all required topics are properly configured in the bridge.
Validates: Requirements 8.1, 8.2, 8.3
"""

import unittest
import yaml
import os


class TestBridgeConfiguration(unittest.TestCase):
    """Unit tests for bridge configuration. Validates: Requirements 8.1, 8.2, 8.3"""
    
    @classmethod
    def setUpClass(cls):
        """Load bridge configuration"""
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'bridge_config.yaml'
        )
        with open(config_path, 'r') as f:
            cls.config = yaml.safe_load(f)
        
        # Build lookup dictionaries
        cls.ros_topics = {entry['ros_topic_name']: entry for entry in cls.config}
    
    def test_all_vehicle_cmd_vel_topics_bridged(self):
        """Test all 16 vehicle cmd_vel topics are bridged (Requirement 8.1)"""
        expected_vehicles = []
        for direction in ['n', 's', 'e', 'w']:
            for num in range(1, 5):
                expected_vehicles.append(f'car_{direction}{num}')
        
        for vehicle in expected_vehicles:
            topic = f'/model/{vehicle}/cmd_vel'
            self.assertIn(
                topic,
                self.ros_topics,
                f"Vehicle cmd_vel topic {topic} should be in bridge config"
            )
            
            entry = self.ros_topics[topic]
            self.assertEqual(entry['direction'], 'ROS_TO_GZ')
            self.assertEqual(entry['ros_type_name'], 'geometry_msgs/msg/Twist')
    
    def test_all_sensor_topics_bridged(self):
        """Test all 4 sensor topics are bridged (Requirement 8.2)"""
        expected_sensors = ['/sensors/north', '/sensors/south', '/sensors/east', '/sensors/west']
        
        for sensor in expected_sensors:
            self.assertIn(
                sensor,
                self.ros_topics,
                f"Sensor topic {sensor} should be in bridge config"
            )
            
            entry = self.ros_topics[sensor]
            self.assertEqual(entry['direction'], 'GZ_TO_ROS')
            self.assertEqual(entry['ros_type_name'], 'ros_gz_interfaces/msg/LogicalCameraImage')
    
    def test_traffic_light_command_topic_bridged(self):
        """Test traffic light command topic is bridged (Requirement 8.3)"""
        topic = '/traffic_lights/commands'
        
        self.assertIn(
            topic,
            self.ros_topics,
            f"Traffic light command topic {topic} should be in bridge config"
        )
        
        entry = self.ros_topics[topic]
        self.assertEqual(entry['direction'], 'ROS_TO_GZ')
        self.assertEqual(entry['ros_type_name'], 'ros_gz_interfaces/msg/Light')
    
    def test_topic_names_match_node_expectations(self):
        """Test topic names match what nodes expect"""
        # Vehicle topics should match pattern /model/car_{direction}{number}/cmd_vel
        cmd_vel_topics = [t for t in self.ros_topics if '/cmd_vel' in t]
        vehicle_topics = [t for t in cmd_vel_topics if '/model/car_' in t]
        
        for topic in vehicle_topics:
            self.assertTrue(
                topic.startswith('/model/car_'),
                f"Vehicle topic {topic} should start with /model/car_"
            )
        
        # Sensor topics should be /sensors/{direction} plus intersection zone plus pedestrian sensors
        sensor_topics = [t for t in self.ros_topics if t.startswith('/sensors/')]
        # 4 directional sensors + 1 intersection zone sensor + 4 pedestrian sensors = 9 total
        self.assertEqual(len(sensor_topics), 9)
        
        for direction in ['north', 'south', 'east', 'west']:
            self.assertIn(f'/sensors/{direction}', sensor_topics)
        
        # Intersection zone sensor
        self.assertIn('/sensors/intersection_zone', sensor_topics)


if __name__ == '__main__':
    unittest.main()
