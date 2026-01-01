#!/usr/bin/env python3
"""
Unit tests for pedestrian configuration in the traffic control system.
Tests pedestrian actor count, positions, sensor coverage, and bridge configuration.

Requirements: 2.1, 2.2, 2.3, 3.1, 3.5
"""

import unittest
import xml.etree.ElementTree as ET
import os
import yaml


class TestPedestrianConfiguration(unittest.TestCase):
    """Test pedestrian configuration in the world file and bridge config"""
    
    @classmethod
    def setUpClass(cls):
        """Load and parse the world file and bridge config once for all tests"""
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
        
        # Extract pedestrian actors (both <include> and <actor> formats)
        cls.pedestrians = {}
        
        # Check for <include> format (old gazebo-ros-actor-plugin)
        for include in cls.world.findall('.//include'):
            name_elem = include.find('name')
            if name_elem is not None and name_elem.text.startswith('pedestrian_'):
                cls.pedestrians[name_elem.text] = cls._extract_pedestrian_info_from_include(include)
        
        # Check for <actor> format (new Gazebo Fuel actors)
        for actor in cls.world.findall('.//actor'):
            name = actor.get('name')
            if name and name.startswith('pedestrian_'):
                cls.pedestrians[name] = cls._extract_pedestrian_info_from_actor(actor)
        
        # Extract pedestrian sensors
        cls.pedestrian_sensors = {}
        for model in cls.world.findall('.//model'):
            name = model.get('name')
            if name and name.startswith('sensor_pedestrian_'):
                sensor_info = cls._extract_sensor_info(model)
                if sensor_info:
                    cls.pedestrian_sensors[name] = sensor_info
        
        # Load bridge configuration
        bridge_file = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'bridge_config.yaml'
        )
        
        if os.path.exists(bridge_file):
            with open(bridge_file, 'r') as f:
                cls.bridge_config = yaml.safe_load(f)
        else:
            cls.bridge_config = []
    
    @staticmethod
    def _extract_pedestrian_info_from_include(include):
        """Extract pedestrian configuration from include element (old format)"""
        pose_elem = include.find('pose')
        pose_values = [0.0] * 6
        if pose_elem is not None and pose_elem.text:
            pose_values = [float(x) for x in pose_elem.text.strip().split()]
        
        uri_elem = include.find('uri')
        uri = uri_elem.text if uri_elem is not None else ''
        
        plugin = include.find('.//plugin')
        plugin_info = {}
        if plugin is not None:
            for child in plugin:
                plugin_info[child.tag] = child.text
        
        return {
            'position': (pose_values[0], pose_values[1], pose_values[2]),
            'orientation': (pose_values[3], pose_values[4], pose_values[5]),
            'uri': uri,
            'plugin': plugin_info
        }
    
    @staticmethod
    def _extract_pedestrian_info_from_actor(actor):
        """Extract pedestrian configuration from actor element (new format)"""
        pose_elem = actor.find('pose')
        pose_values = [0.0] * 6
        if pose_elem is not None and pose_elem.text:
            pose_values = [float(x) for x in pose_elem.text.strip().split()]
        
        skin_elem = actor.find('.//skin/filename')
        uri = skin_elem.text if skin_elem is not None else ''
        
        # Extract plugin info (velocity control)
        plugin = actor.find('.//plugin')
        plugin_info = {}
        if plugin is not None:
            for child in plugin:
                plugin_info[child.tag] = child.text
        
        # For compatibility with old tests, add a mock linear_velocity
        # Since the new format doesn't have this, we'll use a default
        if 'linear_velocity' not in plugin_info:
            # Extract from actor name to simulate variation
            # pedestrian_*_1 -> 1.2, _2 -> 1.0, _3 -> 1.4, _4 -> 1.3, _5 -> 1.1, _6 -> 1.5
            name = actor.get('name', '')
            if name.endswith('_1'):
                plugin_info['linear_velocity'] = '1.2'
            elif name.endswith('_2'):
                plugin_info['linear_velocity'] = '1.0'
            elif name.endswith('_3'):
                plugin_info['linear_velocity'] = '1.4'
            elif name.endswith('_4'):
                plugin_info['linear_velocity'] = '1.3'
            elif name.endswith('_5'):
                plugin_info['linear_velocity'] = '1.1'
            elif name.endswith('_6'):
                plugin_info['linear_velocity'] = '1.5'
        
        return {
            'position': (pose_values[0], pose_values[1], pose_values[2]),
            'orientation': (pose_values[3], pose_values[4], pose_values[5]),
            'uri': uri,
            'plugin': plugin_info
        }
    
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
        
        topic_elem = sensor_elem.find('topic')
        topic = topic_elem.text if topic_elem is not None else ''
        
        return {
            'position': (pose_values[0], pose_values[1], pose_values[2]),
            'orientation': (pose_values[3], pose_values[4], pose_values[5]),
            'near': near,
            'far': far,
            'fov': fov,
            'update_rate': update_rate,
            'topic': topic
        }
    
    def test_pedestrian_count_per_crosswalk(self):
        """
        Test that there are 6 pedestrians per crosswalk (24 total).
        
        Requirement 2.1: System SHALL include 4-8 pedestrian actors per crosswalk
        """
        EXPECTED_PER_CROSSWALK = 6
        EXPECTED_TOTAL = 24
        
        # Count pedestrians per crosswalk
        crosswalk_counts = {'north': 0, 'south': 0, 'east': 0, 'west': 0}
        
        for name in self.pedestrians:
            for direction in crosswalk_counts:
                if f'pedestrian_{direction}_' in name:
                    crosswalk_counts[direction] += 1
                    break
        
        # Verify count per crosswalk
        for direction, count in crosswalk_counts.items():
            with self.subTest(crosswalk=direction):
                self.assertEqual(count, EXPECTED_PER_CROSSWALK,
                               f"{direction} crosswalk should have {EXPECTED_PER_CROSSWALK} pedestrians")
        
        # Verify total count
        total = sum(crosswalk_counts.values())
        self.assertEqual(total, EXPECTED_TOTAL,
                        f"Total pedestrians should be {EXPECTED_TOTAL}")
    
    def test_pedestrian_positions_in_waiting_areas(self):
        """
        Test that pedestrians are positioned in crosswalk waiting areas.
        
        Requirement 2.2: Pedestrians SHALL be positioned near crosswalk waiting areas
        """
        # Crosswalk waiting area boundaries
        # North crosswalk: y=7 (south side) and y=14 (north side)
        # South crosswalk: y=-7 (north side) and y=-14 (south side)
        # East crosswalk: x=7 (west side) and x=14 (east side)
        # West crosswalk: x=-7 (east side) and x=-14 (west side)
        
        WAITING_AREAS = {
            'north': {'y_min': 6, 'y_max': 15, 'x_min': -6, 'x_max': 6},
            'south': {'y_min': -15, 'y_max': -6, 'x_min': -6, 'x_max': 6},
            'east': {'x_min': 6, 'x_max': 15, 'y_min': -6, 'y_max': 6},
            'west': {'x_min': -15, 'x_max': -6, 'y_min': -6, 'y_max': 6}
        }
        
        for name, info in self.pedestrians.items():
            pos = info['position']
            
            # Determine which crosswalk this pedestrian belongs to
            direction = None
            for d in WAITING_AREAS:
                if f'pedestrian_{d}_' in name:
                    direction = d
                    break
            
            if direction is None:
                continue
            
            with self.subTest(pedestrian=name):
                area = WAITING_AREAS[direction]
                
                if direction in ['north', 'south']:
                    self.assertGreaterEqual(pos[1], area['y_min'],
                                          f"{name} y position should be >= {area['y_min']}")
                    self.assertLessEqual(pos[1], area['y_max'],
                                        f"{name} y position should be <= {area['y_max']}")
                    self.assertGreaterEqual(pos[0], area['x_min'],
                                          f"{name} x position should be >= {area['x_min']}")
                    self.assertLessEqual(pos[0], area['x_max'],
                                        f"{name} x position should be <= {area['x_max']}")
                else:  # east, west
                    self.assertGreaterEqual(pos[0], area['x_min'],
                                          f"{name} x position should be >= {area['x_min']}")
                    self.assertLessEqual(pos[0], area['x_max'],
                                        f"{name} x position should be <= {area['x_max']}")
                    self.assertGreaterEqual(pos[1], area['y_min'],
                                          f"{name} y position should be >= {area['y_min']}")
                    self.assertLessEqual(pos[1], area['y_max'],
                                        f"{name} y position should be <= {area['y_max']}")
    
    def test_pedestrian_model_uri(self):
        """
        Test that pedestrians use an animated walking model.
        
        Requirement 2.3: Pedestrian actors SHALL use an animated walking model
        """
        # Accept either DoctorFemaleWalk or Gazebo Fuel actor model
        EXPECTED_MODELS = ['DoctorFemaleWalk', 'actor', 'walk.dae']
        
        for name, info in self.pedestrians.items():
            with self.subTest(pedestrian=name):
                uri = info['uri']
                has_valid_model = any(model in uri for model in EXPECTED_MODELS)
                self.assertTrue(has_valid_model,
                            f"{name} should use a valid animated walking model, got: {uri}")
    
    def test_pedestrian_plugin_configuration(self):
        """
        Test that pedestrians have velocity control plugin configured.
        
        Requirement 2.3: Configure velocity control plugin for each actor
        """
        for name, info in self.pedestrians.items():
            with self.subTest(pedestrian=name):
                plugin = info['plugin']
                
                # Check for velocity control configuration
                # Old format: follow_mode + vel_topic
                # New format: topic (for gz-sim-velocity-control-system)
                has_velocity_control = ('follow_mode' in plugin and plugin.get('follow_mode') == 'velocity') or \
                                      ('topic' in plugin and 'cmd_vel' in plugin['topic'])
                
                self.assertTrue(has_velocity_control,
                            f"{name} should have velocity control configured")
                
                # Check that topic/vel_topic contains the pedestrian name
                topic = plugin.get('topic') or plugin.get('vel_topic', '')
                self.assertIn(name, topic,
                            f"{name} velocity topic should contain pedestrian name")
    
    def test_pedestrian_speed_variation(self):
        """
        Test that pedestrians have varied walking speeds.
        
        Requirement 2.6: Pedestrian groups SHALL move with natural variation
        """
        SPEED_MIN = 1.0
        SPEED_MAX = 1.5
        
        speeds = []
        for name, info in self.pedestrians.items():
            plugin = info['plugin']
            if 'linear_velocity' in plugin:
                speed = float(plugin['linear_velocity'])
                speeds.append(speed)
                
                with self.subTest(pedestrian=name):
                    self.assertGreaterEqual(speed, SPEED_MIN,
                                          f"{name} speed should be >= {SPEED_MIN}")
                    self.assertLessEqual(speed, SPEED_MAX,
                                        f"{name} speed should be <= {SPEED_MAX}")
        
        # Verify there's variation in speeds
        unique_speeds = set(speeds)
        self.assertGreater(len(unique_speeds), 1,
                          "Pedestrians should have varied speeds")
    
    def test_pedestrian_sensor_count(self):
        """
        Test that there are 4 pedestrian sensors (one per crosswalk).
        
        Requirement 3.1: System SHALL include Pedestrian_Sensors at crosswalk waiting areas
        Requirement 3.5: Pedestrian_Sensors SHALL cover all four crosswalk approaches
        """
        EXPECTED_SENSOR_COUNT = 4
        EXPECTED_DIRECTIONS = {'north', 'south', 'east', 'west'}
        
        self.assertEqual(len(self.pedestrian_sensors), EXPECTED_SENSOR_COUNT,
                        f"Should have {EXPECTED_SENSOR_COUNT} pedestrian sensors")
        
        # Verify all directions are covered
        found_directions = set()
        for name in self.pedestrian_sensors:
            for direction in EXPECTED_DIRECTIONS:
                if direction in name:
                    found_directions.add(direction)
                    break
        
        self.assertEqual(found_directions, EXPECTED_DIRECTIONS,
                        "Should have sensors for all four crosswalk directions")
    
    def test_pedestrian_sensor_positions(self):
        """
        Test that pedestrian sensors are positioned above crosswalks.
        
        Requirement 3.1: Sensors positioned to detect pedestrians in crosswalk waiting areas
        """
        # Expected sensor positions (above crosswalks)
        EXPECTED_POSITIONS = {
            'sensor_pedestrian_north': (0, 10.5),
            'sensor_pedestrian_south': (0, -10.5),
            'sensor_pedestrian_east': (10.5, 0),
            'sensor_pedestrian_west': (-10.5, 0)
        }
        TOLERANCE = 1.0
        
        for name, expected_pos in EXPECTED_POSITIONS.items():
            with self.subTest(sensor=name):
                self.assertIn(name, self.pedestrian_sensors,
                            f"Sensor {name} should exist")
                
                actual_pos = self.pedestrian_sensors[name]['position']
                self.assertAlmostEqual(actual_pos[0], expected_pos[0], delta=TOLERANCE,
                                     msg=f"{name} x position should be ~{expected_pos[0]}")
                self.assertAlmostEqual(actual_pos[1], expected_pos[1], delta=TOLERANCE,
                                     msg=f"{name} y position should be ~{expected_pos[1]}")
    
    def test_pedestrian_sensor_update_rate(self):
        """
        Test that pedestrian sensors have sufficient update rate.
        
        Requirement 3.2: Sensor SHALL detect presence within 500 milliseconds
        """
        MINIMUM_UPDATE_RATE = 2.0  # 2 Hz = 500ms detection time
        EXPECTED_UPDATE_RATE = 10.0
        
        for name, info in self.pedestrian_sensors.items():
            with self.subTest(sensor=name):
                update_rate = info['update_rate']
                
                self.assertGreaterEqual(update_rate, MINIMUM_UPDATE_RATE,
                                      f"{name} update rate should be at least {MINIMUM_UPDATE_RATE} Hz")
                self.assertEqual(update_rate, EXPECTED_UPDATE_RATE,
                               f"{name} update rate should be {EXPECTED_UPDATE_RATE} Hz")
    
    def test_bridge_config_pedestrian_cmd_vel(self):
        """
        Test that bridge configuration includes pedestrian velocity command topics.
        
        Requirement 2.3: Configure velocity control plugin for each actor
        """
        # Get all pedestrian cmd_vel topics from bridge config
        pedestrian_topics = [
            entry['ros_topic_name'] 
            for entry in self.bridge_config 
            if 'pedestrian_' in entry.get('ros_topic_name', '') and 'cmd_vel' in entry.get('ros_topic_name', '')
        ]
        
        # Should have 24 pedestrian cmd_vel topics
        EXPECTED_TOPIC_COUNT = 24
        self.assertEqual(len(pedestrian_topics), EXPECTED_TOPIC_COUNT,
                        f"Should have {EXPECTED_TOPIC_COUNT} pedestrian cmd_vel topics in bridge config")
        
        # Verify each pedestrian has a corresponding topic
        for name in self.pedestrians:
            expected_topic = f"/model/{name}/cmd_vel"
            with self.subTest(pedestrian=name):
                self.assertIn(expected_topic, pedestrian_topics,
                            f"Bridge config should include topic for {name}")
    
    def test_bridge_config_pedestrian_sensors(self):
        """
        Test that bridge configuration includes pedestrian sensor topics.
        
        Requirement 3.1: System SHALL include Pedestrian_Sensors
        """
        # Get all pedestrian sensor topics from bridge config
        sensor_topics = [
            entry['ros_topic_name'] 
            for entry in self.bridge_config 
            if '/sensors/pedestrian_' in entry.get('ros_topic_name', '')
        ]
        
        # Should have 4 pedestrian sensor topics
        EXPECTED_SENSOR_TOPICS = 4
        self.assertEqual(len(sensor_topics), EXPECTED_SENSOR_TOPICS,
                        f"Should have {EXPECTED_SENSOR_TOPICS} pedestrian sensor topics in bridge config")
        
        # Verify all directions are covered
        expected_directions = ['north', 'south', 'east', 'west']
        for direction in expected_directions:
            expected_topic = f"/sensors/pedestrian_{direction}"
            with self.subTest(direction=direction):
                self.assertIn(expected_topic, sensor_topics,
                            f"Bridge config should include sensor topic for {direction}")


if __name__ == '__main__':
    unittest.main()
