#!/usr/bin/env python3
"""
Script to add pedestrian actors to the intersection world file.
This will insert pedestrian models at crosswalk locations.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom

def prettify_xml(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def create_pedestrian_actor(name, x, y, z, yaw, model_uri="model://DoctorFemaleWalk"):
    """
    Create a pedestrian actor element.
    
    Args:
        name: Unique name for the pedestrian
        x, y, z: Position coordinates
        yaw: Rotation in radians
        model_uri: URI of the pedestrian model
    
    Returns:
        XML element for the actor
    """
    include = ET.Element('include')
    
    # Name
    name_elem = ET.SubElement(include, 'name')
    name_elem.text = name
    
    # Pose
    pose_elem = ET.SubElement(include, 'pose')
    pose_elem.text = f"{x} {y} {z} 0 0 {yaw}"
    
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
    linear_velocity.text = '1.0'
    
    angular_tolerance = ET.SubElement(plugin, 'angular_tolerance')
    angular_tolerance.text = '0.0872'
    
    angular_velocity = ET.SubElement(plugin, 'angular_velocity')
    angular_velocity.text = '2.5'
    
    default_rotation = ET.SubElement(plugin, 'default_rotation')
    default_rotation.text = '1.57'
    
    return include

def generate_pedestrian_xml():
    """Generate XML for all pedestrians at crosswalk locations."""
    
    pedestrians = []
    
    # North crosswalk pedestrians (waiting to cross east-west)
    # Position: Near Y=13 (north stop line), waiting to cross
    pedestrians.append({
        'name': 'pedestrian_north_1',
        'x': -8, 'y': 14, 'z': 0,
        'yaw': -1.57,  # Facing west (ready to cross)
        'model': 'model://DoctorFemaleWalk'
    })
    pedestrians.append({
        'name': 'pedestrian_north_2',
        'x': 8, 'y': 14, 'z': 0,
        'yaw': 1.57,  # Facing east (ready to cross)
        'model': 'model://DoctorFemaleWalk'
    })
    
    # South crosswalk pedestrians (waiting to cross east-west)
    # Position: Near Y=-13 (south stop line)
    pedestrians.append({
        'name': 'pedestrian_south_1',
        'x': -8, 'y': -14, 'z': 0,
        'yaw': -1.57,  # Facing west
        'model': 'model://DoctorFemaleWalk'
    })
    pedestrians.append({
        'name': 'pedestrian_south_2',
        'x': 8, 'y': -14, 'z': 0,
        'yaw': 1.57,  # Facing east
        'model': 'model://DoctorFemaleWalk'
    })
    
    # East crosswalk pedestrians (waiting to cross north-south)
    # Position: Near X=13 (east stop line)
    pedestrians.append({
        'name': 'pedestrian_east_1',
        'x': 14, 'y': -8, 'z': 0,
        'yaw': 3.14,  # Facing south
        'model': 'model://DoctorFemaleWalk'
    })
    pedestrians.append({
        'name': 'pedestrian_east_2',
        'x': 14, 'y': 8, 'z': 0,
        'yaw': 0,  # Facing north
        'model': 'model://DoctorFemaleWalk'
    })
    
    # West crosswalk pedestrians (waiting to cross north-south)
    # Position: Near X=-13 (west stop line)
    pedestrians.append({
        'name': 'pedestrian_west_1',
        'x': -14, 'y': -8, 'z': 0,
        'yaw': 3.14,  # Facing south
        'model': 'model://DoctorFemaleWalk'
    })
    pedestrians.append({
        'name': 'pedestrian_west_2',
        'x': -14, 'y': 8, 'z': 0,
        'yaw': 0,  # Facing north
        'model': 'model://DoctorFemaleWalk'
    })
    
    # Generate XML for each pedestrian
    xml_strings = []
    for ped in pedestrians:
        actor_elem = create_pedestrian_actor(
            ped['name'], ped['x'], ped['y'], ped['z'], 
            ped['yaw'], ped['model']
        )
        xml_string = ET.tostring(actor_elem, encoding='unicode')
        xml_strings.append(xml_string)
    
    return '\n\n'.join(xml_strings)

def main():
    """Generate pedestrian XML and save to file."""
    print("Generating pedestrian actor XML...")
    
    xml_content = generate_pedestrian_xml()
    
    # Save to file
    output_file = 'pedestrian_actors.xml'
    with open(output_file, 'w') as f:
        f.write('<!-- Pedestrian Actors for Intersection -->\n')
        f.write('<!-- Add this section to your intersection.world file before </world> -->\n\n')
        f.write(xml_content)
    
    print(f"✓ Pedestrian XML generated and saved to: {output_file}")
    print("\nPedestrian Locations:")
    print("  - North crosswalk: 2 pedestrians")
    print("  - South crosswalk: 2 pedestrians")
    print("  - East crosswalk: 2 pedestrians")
    print("  - West crosswalk: 2 pedestrians")
    print("  Total: 8 pedestrians")
    print("\nNext steps:")
    print("  1. Review the generated XML in pedestrian_actors.xml")
    print("  2. Copy the content and paste it into intersection.world before </world>")
    print("  3. Build the workspace: cd ros2_ws && colcon build")
    print("  4. Launch the simulation to see pedestrians")

if __name__ == '__main__':
    main()
