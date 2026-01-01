#!/usr/bin/env python3
# Read the world file, remove malformed pedestrian section, keep vehicles and sensors

with open('ros2_ws/src/smart_traffic_system/worlds/intersection.world.backup', 'r') as f:
    lines = f.readlines()

# Keep lines 1-144 (everything before the malformed section)
# Skip lines 145-406 (malformed pedestrians and vehicles)
# Keep lines 407-end (sensors and closing tags)

output_lines = lines[:144]  # Lines 1-144

# Add proper vehicle includes (16 vehicles as before)
vehicles_xml = """
    <!-- Realistic Vehicles: 4 Roads, 2 Lanes per Road, 4 Cars per Road = 16 Cars -->
    
    <!-- SOUTHBOUND (Coming from North, x=-2 and x=-5) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_s1</name>
      <pose>-2 30 0.01 0 0 0</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_s2</name>
      <pose>-2 25 0.01 0 0 0</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_s3</name>
      <pose>-5 30 0.01 0 0 0</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_s4</name>
      <pose>-5 25 0.01 0 0 0</pose>
    </include>

    <!-- NORTHBOUND (Coming from South, x=2 and x=5) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_n1</name>
      <pose>2 -30 0.01 0 0 3.1415</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_n2</name>
      <pose>2 -25 0.01 0 0 3.1415</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_n3</name>
      <pose>5 -30 0.01 0 0 3.1415</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_n4</name>
      <pose>5 -25 0.01 0 0 3.1415</pose>
    </include>

    <!-- WESTBOUND (Coming from East, y=2 and y=5) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_w1</name>
      <pose>30 2 0.01 0 0 -1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_w2</name>
      <pose>25 2 0.01 0 0 -1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_w3</name>
      <pose>30 5 0.01 0 0 -1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_w4</name>
      <pose>25 5 0.01 0 0 -1.5707</pose>
    </include>

    <!-- EASTBOUND (Coming from West, y=-2 and y=-5) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_e1</name>
      <pose>-30 -2 0.01 0 0 1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_e2</name>
      <pose>-25 -2 0.01 0 0 1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_e3</name>
      <pose>-30 -5 0.01 0 0 1.5707</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
      <name>prius_e4</name>
      <pose>-25 -5 0.01 0 0 1.5707</pose>
    </include>

"""

output_lines.append(vehicles_xml)

# Add lines from 407 onwards (sensors and closing)
output_lines.extend(lines[406:])

# Write the fixed file
with open('ros2_ws/src/smart_traffic_system/worlds/intersection.world', 'w') as f:
    f.writelines(output_lines)

print("World file fixed!")
print(f"Kept lines 1-144, added vehicles, kept lines 407-{len(lines)}")
