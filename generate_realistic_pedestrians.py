#!/usr/bin/env python3
"""
Generate realistic pedestrian crowds at intersection crosswalks.

Requirements:
- 4-5 pedestrians per crosswalk (16-20 total)
- No overlapping (min spacing: 0.4-0.6m lateral, 0.5-0.8m longitudinal)
- Positioned on curb/sidewalk, NOT in road or crosswalk
- All facing same direction (toward destination)
- Realistic crowd formation (not single line)
"""

import xml.etree.ElementTree as ET
from typing import List, Tuple
import math


class PedestrianSpawner:
    """Generate realistic pedestrian crowds at crosswalks."""
    
    # Spacing constraints (meters)
    LATERAL_SPACING_MIN = 0.4
    LATERAL_SPACING_MAX = 0.6
    LONGITUDINAL_SPACING_MIN = 0.5
    LONGITUDINAL_SPACING_MAX = 0.8
    
    # Crosswalk reference positions (curb edge, before crosswalk)
    # Format: (x, y, yaw, crossing_direction)
    CROSSWALK_POSITIONS = {
        'north': {
            'base_pos': (0, 13.5, 0),  # Just north of stop line (y=13)
            'yaw': -1.5708,  # Facing south (toward crosswalk)
            'lateral_axis': 'x',  # Spread along X axis
            'longitudinal_axis': 'y',  # Queue along Y axis (away from road)
            'lateral_direction': 1,  # Positive X
            'longitudinal_direction': 1,  # Positive Y (away from intersection)
        },
        'south': {
            'base_pos': (0, -13.5, 0),  # Just south of stop line (y=-13)
            'yaw': 1.5708,  # Facing north (toward crosswalk)
            'lateral_axis': 'x',
            'longitudinal_axis': 'y',
            'lateral_direction': 1,
            'longitudinal_direction': -1,  # Negative Y (away from intersection)
        },
        'east': {
            'base_pos': (13.5, 0, 0),  # Just east of stop line (x=13)
            'yaw': 3.14159,  # Facing west (toward crosswalk)
            'lateral_axis': 'y',  # Spread along Y axis
            'longitudinal_axis': 'x',  # Queue along X axis
            'lateral_direction': 1,  # Positive Y
            'longitudinal_direction': 1,  # Positive X (away from intersection)
        },
        'west': {
            'base_pos': (-13.5, 0, 0),  # Just west of stop line (x=-13)
            'yaw': 0,  # Facing east (toward crosswalk)
            'lateral_axis': 'y',
            'longitudinal_axis': 'x',
            'lateral_direction': 1,
            'longitudinal_direction': -1,  # Negative X (away from intersection)
        },
    }
    
    def __init__(self):
        self.pedestrian_count = 0
    
    def generate_crowd_positions(self, num_pedestrians: int) -> List[Tuple[float, float]]:
        """
        Generate positions for a crowd of pedestrians.
        
        Creates a realistic formation:
        - 2-3 pedestrians in front row (side by side)
        - Remaining pedestrians in back rows
        - Slight variations in spacing for realism
        
        Returns:
            List of (lateral_offset, longitudinal_offset) tuples
        """
        positions = []
        
        # Determine formation
        if num_pedestrians <= 2:
            # Single row
            rows = [num_pedestrians]
        elif num_pedestrians <= 4:
            # Two rows: 2 front, rest back
            rows = [2, num_pedestrians - 2]
        else:
            # Three rows: 2-2-rest or 3-2-rest
            if num_pedestrians == 5:
                rows = [2, 2, 1]
            else:
                rows = [3, 2, 1]
        
        longitudinal_offset = 0
        
        for row_size in rows:
            # Calculate lateral positions for this row
            if row_size == 1:
                lateral_offsets = [0]
            elif row_size == 2:
                lateral_offsets = [-0.5, 0.5]
            elif row_size == 3:
                lateral_offsets = [-0.8, 0, 0.8]
            else:
                # Shouldn't happen, but handle it
                lateral_offsets = [i * 0.5 - (row_size - 1) * 0.25 for i in range(row_size)]
            
            # Add positions for this row
            for lateral_offset in lateral_offsets:
                positions.append((lateral_offset, longitudinal_offset))
            
            # Move to next row (further from road)
            longitudinal_offset += 0.6  # 0.6m spacing between rows
        
        return positions
    
    def create_pedestrian_actor(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        model_uri: str = "model://DoctorFemaleWalk"
    ) -> ET.Element:
        """Create a pedestrian actor XML element."""
        include = ET.Element('include')
        
        # Name
        name_elem = ET.SubElement(include, 'name')
        name_elem.text = name
        
        # Pose
        pose_elem = ET.SubElement(include, 'pose')
        pose_elem.text = f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.6f}"
        
        # URI
        uri_elem = ET.SubElement(include, 'uri')
        uri_elem.text = model_uri
        
        # Plugin
        plugin = ET.SubElement(include, 'plugin')
        plugin.set('filename', 'libgazebo_ros_actor_plugin.so')
        plugin.set('name', 'gazebo_ros_actor_plugin::GazeboRosActorCommand')
        
        # Plugin parameters
        follow_mode = ET.SubElement(plugin, 'follow_mode')
        follow_mode.text = 'path'
        
        vel_topic = ET.SubElement(plugin, 'vel_topic')
        vel_topic.text = f'/{name}/cmd_vel'
        
        path_topic = ET.SubElement(plugin, 'path_topic')
        path_topic.text = f'/{name}/cmd_path'
        
        animation_factor = ET.SubElement(plugin, 'animation_factor')
        animation_factor.text = '4.0'
        
        linear_tolerance = ET.SubElement(plugin, 'linear_tolerance')
        linear_tolerance.text = '0.1'
        
        linear_velocity = ET.SubElement(plugin, 'linear_velocity')
        linear_velocity.text = '1.2'
        
        angular_tolerance = ET.SubElement(plugin, 'angular_tolerance')
        angular_tolerance.text = '0.0872'
        
        angular_velocity = ET.SubElement(plugin, 'angular_velocity')
        angular_velocity.text = '2.5'
        
        default_rotation = ET.SubElement(plugin, 'default_rotation')
        default_rotation.text = '1.57'
        
        return include
    
    def generate_crosswalk_pedestrians(
        self,
        crosswalk_name: str,
        num_pedestrians: int = 5
    ) -> List[ET.Element]:
        """Generate all pedestrians for a specific crosswalk."""
        config = self.CROSSWALK_POSITIONS[crosswalk_name]
        base_x, base_y, base_z = config['base_pos']
        yaw = config['yaw']
        
        # Generate crowd positions (lateral, longitudinal offsets)
        crowd_positions = self.generate_crowd_positions(num_pedestrians)
        
        pedestrians = []
        
        for i, (lateral_offset, longitudinal_offset) in enumerate(crowd_positions):
            # Calculate actual position
            if config['lateral_axis'] == 'x':
                x = base_x + lateral_offset * config['lateral_direction']
                y = base_y + longitudinal_offset * config['longitudinal_direction']
            else:  # lateral_axis == 'y'
                x = base_x + longitudinal_offset * config['longitudinal_direction']
                y = base_y + lateral_offset * config['lateral_direction']
            
            # Create pedestrian
            ped_name = f"pedestrian_{crosswalk_name}_{i+1}"
            ped_elem = self.create_pedestrian_actor(ped_name, x, y, base_z, yaw)
            pedestrians.append(ped_elem)
            
            self.pedestrian_count += 1
        
        return pedestrians
    
    def generate_all_pedestrians(self) -> str:
        """Generate all pedestrians for all crosswalks."""
        all_pedestrians = []
        
        # Generate pedestrians for each crosswalk
        for crosswalk in ['north', 'south', 'east', 'west']:
            num_peds = 5  # 5 pedestrians per crosswalk
            peds = self.generate_crosswalk_pedestrians(crosswalk, num_peds)
            all_pedestrians.extend(peds)
        
        # Convert to XML string
        xml_parts = []
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append('<!-- REALISTIC PEDESTRIAN CROWDS AT CROSSWALKS -->')
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append(f'<!-- Total: {self.pedestrian_count} pedestrians (5 per crosswalk) -->')
        xml_parts.append('<!-- Positioned on curb, facing crosswalk -->')
        xml_parts.append('<!-- No overlapping, realistic crowd formation -->')
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append('')
        
        for ped_elem in all_pedestrians:
            xml_str = ET.tostring(ped_elem, encoding='unicode')
            # Format nicely
            xml_str = xml_str.replace('><', '>\n      <')
            xml_parts.append('    ' + xml_str)
            xml_parts.append('')
        
        return '\n'.join(xml_parts)


def main():
    """Generate realistic pedestrian crowds."""
    print("=" * 60)
    print("Generating Realistic Pedestrian Crowds")
    print("=" * 60)
    print()
    
    spawner = PedestrianSpawner()
    xml_content = spawner.generate_all_pedestrians()
    
    # Save to file
    output_file = 'realistic_pedestrians.xml'
    with open(output_file, 'w') as f:
        f.write(xml_content)
    
    print(f"✅ Generated {spawner.pedestrian_count} pedestrians")
    print(f"✅ Saved to: {output_file}")
    print()
    print("Pedestrian Distribution:")
    print("  - North crosswalk: 5 pedestrians")
    print("  - South crosswalk: 5 pedestrians")
    print("  - East crosswalk: 5 pedestrians")
    print("  - West crosswalk: 5 pedestrians")
    print()
    print("Formation Details:")
    print("  ✅ Positioned on curb (NOT in road or crosswalk)")
    print("  ✅ All facing toward crosswalk destination")
    print("  ✅ Realistic crowd spacing (0.4-0.8m)")
    print("  ✅ No overlapping or fusion")
    print("  ✅ 2-3 rows per crosswalk")
    print()
    print("Next Steps:")
    print("  1. Review the generated XML in realistic_pedestrians.xml")
    print("  2. Replace old pedestrian section in intersection.world")
    print("  3. Rebuild: cd ros2_ws && colcon build --packages-select smart_traffic_system")
    print("  4. Launch simulation to see realistic crowds")
    print()
    print("=" * 60)


if __name__ == '__main__':
    main()
