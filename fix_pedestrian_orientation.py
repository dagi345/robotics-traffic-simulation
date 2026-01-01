#!/usr/bin/env python3
"""
Fix pedestrian positions and orientations.

CRITICAL FIXES:
1. Position pedestrians on SIDE of road (perpendicular to vehicle flow)
2. Orient pedestrians to face ACROSS road (perpendicular to vehicle direction)

Vehicle Flow:
- North-South road: Vehicles travel along Y-axis
- East-West road: Vehicles travel along X-axis

Pedestrian Placement:
- North/South crosswalks: Pedestrians on EAST/WEST sides, facing EAST/WEST
- East/West crosswalks: Pedestrians on NORTH/SOUTH sides, facing NORTH/SOUTH
"""

import xml.etree.ElementTree as ET
from typing import List, Tuple


class PedestrianOrientationFixer:
    """Fix pedestrian positions and orientations relative to crosswalks."""
    
    # Crosswalk configurations
    # Format: (base_x, base_y, yaw, lateral_axis, longitudinal_axis, lat_dir, long_dir)
    CROSSWALK_CONFIGS = {
        # NORTH CROSSWALK
        # - Crosswalk at Y=10 to Y=13 (north side of intersection)
        # - Vehicles travel N-S (along Y-axis)
        # - Pedestrians cross E-W (perpendicular to vehicles)
        # - Position: WEST side of road (negative X)
        # - Face: EAST (across the road, toward positive X)
        'north': {
            'base_pos': (-7.0, 11.5, 0),  # West side of north crosswalk
            'yaw': 0.0,  # Face EAST (0 radians)
            'lateral_axis': 'y',  # Spread along Y (along crosswalk)
            'longitudinal_axis': 'x',  # Queue along X (away from road)
            'lateral_direction': 1,  # Positive Y
            'longitudinal_direction': -1,  # Negative X (away from road center)
        },
        
        # SOUTH CROSSWALK
        # - Crosswalk at Y=-13 to Y=-10 (south side of intersection)
        # - Vehicles travel N-S (along Y-axis)
        # - Pedestrians cross E-W (perpendicular to vehicles)
        # - Position: EAST side of road (positive X)
        # - Face: WEST (across the road, toward negative X)
        'south': {
            'base_pos': (7.0, -11.5, 0),  # East side of south crosswalk
            'yaw': 3.14159,  # Face WEST (180 degrees)
            'lateral_axis': 'y',  # Spread along Y (along crosswalk)
            'longitudinal_axis': 'x',  # Queue along X (away from road)
            'lateral_direction': -1,  # Negative Y
            'longitudinal_direction': 1,  # Positive X (away from road center)
        },
        
        # EAST CROSSWALK
        # - Crosswalk at X=10 to X=13 (east side of intersection)
        # - Vehicles travel E-W (along X-axis)
        # - Pedestrians cross N-S (perpendicular to vehicles)
        # - Position: NORTH side of road (positive Y)
        # - Face: SOUTH (across the road, toward negative Y)
        'east': {
            'base_pos': (11.5, 7.0, 0),  # North side of east crosswalk
            'yaw': -1.5708,  # Face SOUTH (-90 degrees)
            'lateral_axis': 'x',  # Spread along X (along crosswalk)
            'longitudinal_axis': 'y',  # Queue along Y (away from road)
            'lateral_direction': 1,  # Positive X
            'longitudinal_direction': 1,  # Positive Y (away from road center)
        },
        
        # WEST CROSSWALK
        # - Crosswalk at X=-13 to X=-10 (west side of intersection)
        # - Vehicles travel E-W (along X-axis)
        # - Pedestrians cross N-S (perpendicular to vehicles)
        # - Position: SOUTH side of road (negative Y)
        # - Face: NORTH (across the road, toward positive Y)
        'west': {
            'base_pos': (-11.5, -7.0, 0),  # South side of west crosswalk
            'yaw': 1.5708,  # Face NORTH (90 degrees)
            'lateral_axis': 'x',  # Spread along X (along crosswalk)
            'longitudinal_axis': 'y',  # Queue along Y (away from road)
            'lateral_direction': -1,  # Negative X
            'longitudinal_direction': -1,  # Negative Y (away from road center)
        },
    }
    
    def __init__(self):
        self.pedestrian_count = 0
    
    def generate_crowd_positions(self, num_pedestrians: int) -> List[Tuple[float, float]]:
        """
        Generate positions for a crowd (KEEP EXISTING SPACING).
        
        Returns:
            List of (lateral_offset, longitudinal_offset) tuples
        """
        positions = []
        
        # Same formation as before: 2-2-1
        if num_pedestrians <= 2:
            rows = [num_pedestrians]
        elif num_pedestrians <= 4:
            rows = [2, num_pedestrians - 2]
        else:
            if num_pedestrians == 5:
                rows = [2, 2, 1]
            else:
                rows = [3, 2, 1]
        
        longitudinal_offset = 0
        
        for row_size in rows:
            if row_size == 1:
                lateral_offsets = [0]
            elif row_size == 2:
                lateral_offsets = [-0.5, 0.5]
            elif row_size == 3:
                lateral_offsets = [-0.8, 0, 0.8]
            else:
                lateral_offsets = [i * 0.5 - (row_size - 1) * 0.25 for i in range(row_size)]
            
            for lateral_offset in lateral_offsets:
                positions.append((lateral_offset, longitudinal_offset))
            
            longitudinal_offset += 0.6  # Same 0.6m spacing
        
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
        
        name_elem = ET.SubElement(include, 'name')
        name_elem.text = name
        
        pose_elem = ET.SubElement(include, 'pose')
        pose_elem.text = f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.6f}"
        
        uri_elem = ET.SubElement(include, 'uri')
        uri_elem.text = model_uri
        
        plugin = ET.SubElement(include, 'plugin')
        plugin.set('filename', 'libgazebo_ros_actor_plugin.so')
        plugin.set('name', 'gazebo_ros_actor_plugin::GazeboRosActorCommand')
        
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
        """Generate pedestrians for a crosswalk with CORRECT position and orientation."""
        config = self.CROSSWALK_CONFIGS[crosswalk_name]
        base_x, base_y, base_z = config['base_pos']
        yaw = config['yaw']
        
        # Generate crowd positions (SAME spacing as before)
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
        """Generate all pedestrians with CORRECTED positions and orientations."""
        all_pedestrians = []
        
        for crosswalk in ['north', 'south', 'east', 'west']:
            num_peds = 5
            peds = self.generate_crosswalk_pedestrians(crosswalk, num_peds)
            all_pedestrians.extend(peds)
        
        # Convert to XML string
        xml_parts = []
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append('<!-- CORRECTED PEDESTRIAN POSITIONS & ORIENTATIONS -->')
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append(f'<!-- Total: {self.pedestrian_count} pedestrians (5 per crosswalk) -->')
        xml_parts.append('<!-- ✅ Positioned on SIDE of road (perpendicular to vehicle flow) -->')
        xml_parts.append('<!-- ✅ Facing ACROSS road (perpendicular to vehicle direction) -->')
        xml_parts.append('<!-- ✅ Same spacing, count, and formation as before -->')
        xml_parts.append('<!-- ========================================== -->')
        xml_parts.append('')
        
        for ped_elem in all_pedestrians:
            xml_str = ET.tostring(ped_elem, encoding='unicode')
            xml_str = xml_str.replace('><', '>\n      <')
            xml_parts.append('    ' + xml_str)
            xml_parts.append('')
        
        return '\n'.join(xml_parts)


def main():
    """Generate corrected pedestrian configuration."""
    print("=" * 70)
    print("FIXING PEDESTRIAN POSITIONS AND ORIENTATIONS")
    print("=" * 70)
    print()
    print("CRITICAL FIXES:")
    print("  1. Position pedestrians on SIDE of road (not in vehicle lanes)")
    print("  2. Orient pedestrians to face ACROSS road (not toward vehicles)")
    print()
    print("KEEPING UNCHANGED:")
    print("  ✅ Number of pedestrians (5 per crosswalk)")
    print("  ✅ Spacing (0.5m lateral, 0.6m longitudinal)")
    print("  ✅ Formation (2-2-1 rows)")
    print("  ✅ Collision behavior")
    print()
    
    fixer = PedestrianOrientationFixer()
    xml_content = fixer.generate_all_pedestrians()
    
    output_file = 'corrected_pedestrians.xml'
    with open(output_file, 'w') as f:
        f.write(xml_content)
    
    print(f"✅ Generated {fixer.pedestrian_count} pedestrians")
    print(f"✅ Saved to: {output_file}")
    print()
    print("Corrected Positions:")
    print("  - North crosswalk: WEST side, facing EAST")
    print("  - South crosswalk: EAST side, facing WEST")
    print("  - East crosswalk: NORTH side, facing SOUTH")
    print("  - West crosswalk: SOUTH side, facing NORTH")
    print()
    print("=" * 70)


if __name__ == '__main__':
    main()
