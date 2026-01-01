#!/usr/bin/env python3
"""
Unit tests for sensor configuration in the traffic control system.
Tests sensor positions, field of view coverage, and update rates.

Requirements: 7.1, 7.2, 7.5
"""

import unittest
import xml.etree.ElementTree as ET
import os
import math


class TestSensorConfiguration(unittest.TestCase):
    """Test sensor configuration in the world file"""
    
    @classmethod
    def setUpClass(cls):
        """Load and parse the world file once for all tests"""
        # Find the world file
        world_file = os.path.join(
            os.path.dirname(__file__),
            '..',
            'worlds',
            'intersection.world'
        )
        
        if not os.path.exists(world_file):
            raise FileNotFoundError(f"World file not found: {world_file}")
        
        # Parse the XML
        tree = ET.parse(world_file)
        cls.root = tree.getroot()
        cls.world = cls.root.find('.//world')
        
        # Extract sensor information
        cls.sensors = {}
        for model in cls.world.findall('.//model'):
            name = model.get('name')
            if name and name.startswith('sensor_'):
                sensor_info = cls._extract_sensor_info(model)
                if sensor_info:
                    cls.sensors[name] = sensor_info
    
    @staticmethod
    def _extract_sensor_info(model):
        """Extract sensor configuration from model element"""
        pose_elem = model.find('.//pose')
        if pose_elem is None:
            return None
        
        pose_text = pose_elem.text.strip()
        pose_values = [float(x) for x in pose_text.split()]
        
        sensor_elem = model.find('.//sensor[@type="logical_camera"]')
        if sensor_elem is None:
            return None
        
        logical_camera = sensor_elem.find('.//logical_camera')
        near = float(logical_camera.find('near').text)
        far = float(logical_camera.find('far').text)
        fov = float(logical_camera.find('horizontal_fov').text)
        
        update_rate_elem = sensor_elem.find('update_rate')
        update_rate = float(update_rate_elem.text) if update_rate_elem is not None else 0
        
        return {
            'position': (pose_values[0], pose_values[1], pose_values[2]),
            'orientation': (pose_values[3], pose_values[4], pose_values[5]),
            'near': near,
            'far': far,
            'fov': fov,
            'update_rate': update_rate
        }
    
    def test_sensor_positions_relative_to_stop_lines(self):
        """
        Test that sensors are positioned to cover the approach lanes before the stop line.
        Stop lines are at ±13m, sensors should be at ±20m to cover approach area.
        
        Requirement 7.1: Sensors SHALL be positioned to cover approach lanes before stop line
        """
        STOP_LINE_POSITION = 13.0
        EXPECTED_SENSOR_POSITION = 20.0
        
        # Test north sensor (y-axis)
        self.assertIn('sensor_north_road', self.sensors)
        north_pos = self.sensors['sensor_north_road']['position']
        self.assertEqual(north_pos[1], EXPECTED_SENSOR_POSITION,
                        f"North sensor should be at y={EXPECTED_SENSOR_POSITION}m")
        self.assertGreater(north_pos[1], STOP_LINE_POSITION,
                          "North sensor should be beyond stop line")
        
        # Test south sensor (y-axis)
        self.assertIn('sensor_south_road', self.sensors)
        south_pos = self.sensors['sensor_south_road']['position']
        self.assertEqual(south_pos[1], -EXPECTED_SENSOR_POSITION,
                        f"South sensor should be at y=-{EXPECTED_SENSOR_POSITION}m")
        self.assertLess(south_pos[1], -STOP_LINE_POSITION,
                       "South sensor should be beyond stop line")
        
        # Test east sensor (x-axis)
        self.assertIn('sensor_east_road', self.sensors)
        east_pos = self.sensors['sensor_east_road']['position']
        self.assertEqual(east_pos[0], EXPECTED_SENSOR_POSITION,
                        f"East sensor should be at x={EXPECTED_SENSOR_POSITION}m")
        self.assertGreater(east_pos[0], STOP_LINE_POSITION,
                          "East sensor should be beyond stop line")
        
        # Test west sensor (x-axis)
        self.assertIn('sensor_west_road', self.sensors)
        west_pos = self.sensors['sensor_west_road']['position']
        self.assertEqual(west_pos[0], -EXPECTED_SENSOR_POSITION,
                        f"West sensor should be at x=-{EXPECTED_SENSOR_POSITION}m")
        self.assertLess(west_pos[0], -STOP_LINE_POSITION,
                       "West sensor should be beyond stop line")
    
    def test_sensor_field_of_view_coverage(self):
        """
        Test that lane sensors have sufficient field of view to detect all vehicles in their lanes.
        
        Requirement 7.2: Sensors SHALL have sufficient field of view to detect all vehicles
        """
        EXPECTED_FAR_RANGE = 15.0
        EXPECTED_NEAR_RANGE = 1.0
        EXPECTED_FOV = 1.5  # radians
        STOP_LINE_POSITION = 13.0
        
        # Only test lane sensors (north, south, east, west), not intersection zone sensor
        lane_sensors = {k: v for k, v in self.sensors.items() 
                       if 'intersection' not in k}
        
        for sensor_name, sensor_info in lane_sensors.items():
            with self.subTest(sensor=sensor_name):
                # Check far range
                self.assertEqual(sensor_info['far'], EXPECTED_FAR_RANGE,
                               f"{sensor_name} should have far range of {EXPECTED_FAR_RANGE}m")
                
                # Check near range
                self.assertEqual(sensor_info['near'], EXPECTED_NEAR_RANGE,
                               f"{sensor_name} should have near range of {EXPECTED_NEAR_RANGE}m")
                
                # Check FOV
                self.assertEqual(sensor_info['fov'], EXPECTED_FOV,
                               f"{sensor_name} should have FOV of {EXPECTED_FOV} radians")
                
                # Verify sensor can detect vehicles at stop line
                sensor_pos = sensor_info['position']
                sensor_height = sensor_pos[2]
                
                # Calculate if stop line is within detection range
                if 'north' in sensor_name or 'south' in sensor_name:
                    distance_to_stop = abs(abs(sensor_pos[1]) - STOP_LINE_POSITION)
                else:  # east or west
                    distance_to_stop = abs(abs(sensor_pos[0]) - STOP_LINE_POSITION)
                
                # Distance from sensor to stop line (accounting for height)
                actual_distance = math.sqrt(distance_to_stop**2 + sensor_height**2)
                
                self.assertLessEqual(actual_distance, EXPECTED_FAR_RANGE,
                                   f"{sensor_name} should be able to detect vehicles at stop line")
    
    def test_sensor_update_rate(self):
        """
        Test that lane sensors have sufficient update rate for real-time detection.
        
        Requirement 7.5: Sensor update rate SHALL be at least 5 Hz (now 10 Hz)
        """
        MINIMUM_UPDATE_RATE = 5.0
        EXPECTED_UPDATE_RATE = 10.0
        
        # Only test lane sensors (north, south, east, west), not intersection zone sensor
        lane_sensors = {k: v for k, v in self.sensors.items() 
                       if 'intersection' not in k}
        
        for sensor_name, sensor_info in lane_sensors.items():
            with self.subTest(sensor=sensor_name):
                update_rate = sensor_info['update_rate']
                
                # Check meets minimum requirement
                self.assertGreaterEqual(update_rate, MINIMUM_UPDATE_RATE,
                                      f"{sensor_name} update rate should be at least {MINIMUM_UPDATE_RATE} Hz")
                
                # Check meets expected configuration
                self.assertEqual(update_rate, EXPECTED_UPDATE_RATE,
                               f"{sensor_name} update rate should be {EXPECTED_UPDATE_RATE} Hz")
    
    def test_sensor_orientation(self):
        """
        Test that lane sensors are oriented to look down at the road surface.
        
        Requirement 7.4: Sensors SHALL be oriented to look down at road surface
        """
        EXPECTED_PITCH = 1.5707  # 90 degrees in radians (looking straight down)
        TOLERANCE = 0.01
        
        # Only test lane sensors (north, south, east, west), not intersection zone sensor
        lane_sensors = {k: v for k, v in self.sensors.items() 
                       if 'intersection' not in k}
        
        for sensor_name, sensor_info in lane_sensors.items():
            with self.subTest(sensor=sensor_name):
                orientation = sensor_info['orientation']
                pitch = orientation[1]  # pitch is the second value (roll, pitch, yaw)
                
                self.assertAlmostEqual(pitch, EXPECTED_PITCH, delta=TOLERANCE,
                                     msg=f"{sensor_name} should be oriented to look down (pitch={EXPECTED_PITCH})")
    
    def test_sensor_coverage_of_approach_area(self):
        """
        Test that lane sensors positioned at 20m with 15m range cover the stop line at 13m.
        
        Requirement 7.3: Sensors SHALL be positioned close enough to detect vehicles at stop line
        """
        SENSOR_POSITION = 20.0
        SENSOR_RANGE = 15.0
        STOP_LINE_POSITION = 13.0
        
        # Calculate coverage: sensor at 20m with 15m range covers from 5m to 35m
        # (accounting for sensor height of 10m, the ground coverage is approximately this)
        min_coverage = SENSOR_POSITION - SENSOR_RANGE
        max_coverage = SENSOR_POSITION + SENSOR_RANGE
        
        # Stop line should be within coverage
        self.assertLessEqual(STOP_LINE_POSITION, max_coverage,
                           "Stop line should be within sensor maximum range")
        self.assertGreaterEqual(STOP_LINE_POSITION, min_coverage,
                              "Stop line should be within sensor minimum range")
        
        # Only test lane sensors (north, south, east, west), not intersection zone sensor
        lane_sensors = {k: v for k, v in self.sensors.items() 
                       if 'intersection' not in k}
        
        # Verify all lane sensors have this configuration
        for sensor_name, sensor_info in lane_sensors.items():
            with self.subTest(sensor=sensor_name):
                pos = sensor_info['position']
                sensor_coord = abs(pos[0]) if pos[0] != 0 else abs(pos[1])
                
                self.assertEqual(sensor_coord, SENSOR_POSITION,
                               f"{sensor_name} should be at {SENSOR_POSITION}m")
                self.assertEqual(sensor_info['far'], SENSOR_RANGE,
                               f"{sensor_name} should have range of {SENSOR_RANGE}m")
    
    def test_intersection_zone_sensor_configuration(self):
        """
        Test that the intersection zone sensor is properly configured.
        
        Requirement 10.1: Intersection zone sensor SHALL cover the entire intersection area
        """
        EXPECTED_POSITION = (0.0, 0.0)  # Center of intersection
        EXPECTED_FAR_RANGE = 25.0  # Must cover from -13 to +13 in both X and Y
        EXPECTED_UPDATE_RATE = 10.0
        
        # Find intersection zone sensor
        intersection_sensors = {k: v for k, v in self.sensors.items() 
                               if 'intersection' in k}
        
        self.assertEqual(len(intersection_sensors), 1,
                        "Should have exactly one intersection zone sensor")
        
        sensor_name = list(intersection_sensors.keys())[0]
        sensor_info = intersection_sensors[sensor_name]
        
        # Check position (should be at center of intersection)
        pos = sensor_info['position']
        self.assertEqual(pos[0], EXPECTED_POSITION[0],
                        f"Intersection sensor should be at x={EXPECTED_POSITION[0]}")
        self.assertEqual(pos[1], EXPECTED_POSITION[1],
                        f"Intersection sensor should be at y={EXPECTED_POSITION[1]}")
        
        # Check range (must cover entire intersection zone)
        self.assertEqual(sensor_info['far'], EXPECTED_FAR_RANGE,
                        f"Intersection sensor should have range of {EXPECTED_FAR_RANGE}m")
        
        # Check update rate
        self.assertGreaterEqual(sensor_info['update_rate'], EXPECTED_UPDATE_RATE,
                               f"Intersection sensor update rate should be at least {EXPECTED_UPDATE_RATE} Hz")


if __name__ == '__main__':
    unittest.main()
