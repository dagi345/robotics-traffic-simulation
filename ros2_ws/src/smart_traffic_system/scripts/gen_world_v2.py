import random

def get_pose(direction, lane_index, distance):
    # Lane offsets
    # Southbound (moving south): lanes at x=-2, x=-5
    # Northbound (moving north): lanes at x=2, x=5
    # Westbound (moving west): lanes at y=2, y=5
    # Eastbound (moving east): lanes at y=-2, y=-5
    
    # Z is always 0.01
    z = 0.01
    
    if direction == 's':
        x = -2.0 if lane_index == 1 else -5.0
        y = distance # Positive Y, facing South
        yaw = 0
        return f"{x} {y} {z} 0 0 {yaw}"
        
    elif direction == 'n':
        x = 2.0 if lane_index == 1 else 5.0
        y = -distance # Negative Y, facing North
        yaw = 3.1415
        return f"{x} {y} {z} 0 0 {yaw}"
        
    elif direction == 'w':
        x = distance # Positive X, facing West
        y = 2.0 if lane_index == 1 else 5.0
        yaw = -1.5707
        return f"{x} {y} {z} 0 0 {yaw}"
        
    elif direction == 'e':
        x = -distance # Negative X, facing East
        y = -2.0 if lane_index == 1 else -5.0
        yaw = 1.5707
        return f"{x} {y} {z} 0 0 {yaw}"

def generate_vehicle_xml():
    counts = {
        'n': 10,
        's': 12,
        'e': 8,
        'w': 13
    }
    
    # Start positions (distance from center)
    # Start further back to allow for sparse spacing
    base_start_dist = 25.0 
    
    xml_output = []
    
    # Process each direction
    for d, count in counts.items():
        xml_output.append(f"    <!-- {d.upper()} VEHICLES -->")
        
        # Track distance for each lane separately
        # lane 1 distance, lane 2 distance
        lane_dists = [base_start_dist, base_start_dist]
        
        # Randomize initial start slightly so lanes aren't perfectly aligned
        lane_dists[0] += random.uniform(0, 5)
        lane_dists[1] += random.uniform(0, 5)
        
        for i in range(1, count + 1):
            # Alternate lanes: 1, 2, 1, 2...
            lane_idx = (i % 2) 
            # If lane_idx is 0 (even number), it maps to lane index 1 (inner) or 0 (outer)? 
            # Let's say i=1 -> lane 1, i=2 -> lane 2
            current_lane = 1 if lane_idx == 1 else 2
            
            # Get current distance for this lane
            dist = lane_dists[lane_idx-1]
            
            name = f"car_{d}{i}"
            pose = get_pose(d, current_lane, dist)
            
            xml_output.append(f"    <include>")
            xml_output.append(f"      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>")
            xml_output.append(f"      <name>{name}</name>")
            xml_output.append(f"      <pose>{pose}</pose>")
            xml_output.append(f"      <plugin filename=\"gz-sim-velocity-control-system\" name=\"gz::sim::systems::VelocityControl\">")
            xml_output.append(f"        <initial_linear>0 0 0</initial_linear>")
            xml_output.append(f"        <initial_angular>0 0 0</initial_angular>")
            xml_output.append(f"      </plugin>")
            xml_output.append(f"    </include>")
            
            # Increment distance for next car in this lane
            # Random gap: 8m to 25m (sparse!)
            gap = random.uniform(8.0, 25.0)
            lane_dists[lane_idx-1] += gap

    return "\n".join(xml_output)

if __name__ == "__main__":
    print(generate_vehicle_xml())
