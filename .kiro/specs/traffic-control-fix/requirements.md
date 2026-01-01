# Requirements Document

## Introduction

This specification addresses critical issues in the Smart Traffic Light System simulation that prevent proper vehicle control and cause collisions at the intersection. The system must dynamically control traffic signals based on real-time vehicle density while ensuring safe vehicle movement and collision avoidance.

## Glossary

- **Traffic_Manager**: The ROS 2 node responsible for analyzing sensor data and controlling traffic light states
- **Flow_Controller**: The ROS 2 node responsible for controlling individual vehicle velocities based on traffic light states
- **Logical_Camera**: Gazebo sensor that detects vehicles within a defined field of view
- **Traffic_Light**: Gazebo model with controllable red and green lamp intensities
- **Vehicle**: Prius Hybrid model controlled via velocity commands
- **Intersection_Zone**: The central area where roads cross (approximately -13m to +13m in both X and Y axes)
- **Detection_Zone**: The area monitored by logical cameras to detect approaching vehicles
- **Safe_Following_Distance**: Minimum distance that must be maintained between vehicles (5 meters)
- **Stop_Line**: The position where vehicles must stop when the light is red (at y=±13 or x=±13)

## Requirements

### Requirement 1: Traffic Light Control System

**User Story:** As a traffic system operator, I want traffic lights to be properly controlled by the Traffic_Manager, so that vehicles receive correct signals at the intersection.

#### Acceptance Criteria

1. WHEN the Traffic_Manager publishes a light command, THEN the system SHALL update the corresponding traffic light model in Gazebo
2. WHEN the traffic light state is NS_GREEN, THEN the system SHALL set north and south traffic lights to green and east and west traffic lights to red
3. WHEN the traffic light state is EW_GREEN, THEN the system SHALL set east and west traffic lights to green and north and south traffic lights to red
4. WHEN the traffic light state is ALL_RED, THEN the system SHALL set all traffic lights to red
5. THE Traffic_Manager SHALL use the correct traffic light model names that exist in the Gazebo world file

### Requirement 2: Vehicle Detection and Counting

**User Story:** As a traffic system operator, I want vehicles to be accurately detected and counted, so that the system can make informed decisions about signal timing.

#### Acceptance Criteria

1. WHEN a vehicle enters a Detection_Zone, THEN the Logical_Camera SHALL detect the vehicle and include it in the sensor data
2. WHEN the Traffic_Manager receives sensor data, THEN it SHALL accurately count the number of vehicles in each direction
3. THE Detection_Zone SHALL be positioned to detect vehicles before they reach the Stop_Line
4. THE Detection_Zone SHALL have sufficient range to detect vehicles waiting at the Stop_Line
5. WHEN a vehicle exits a Detection_Zone, THEN the system SHALL update the vehicle count accordingly

### Requirement 3: Adaptive Signal Timing

**User Story:** As a traffic system operator, I want signal timing to adapt based on vehicle density, so that traffic flows efficiently and waiting times are minimized.

#### Acceptance Criteria

1. WHEN a direction has waiting vehicles and the perpendicular direction is empty, THEN the Traffic_Manager SHALL switch to give the waiting direction a green signal after the minimum green time
2. WHEN a direction has been green for the maximum green time, THEN the Traffic_Manager SHALL switch to the perpendicular direction if it has waiting vehicles
3. WHEN switching between green states, THEN the Traffic_Manager SHALL transition through an ALL_RED state for the configured safety interval
4. THE Traffic_Manager SHALL maintain a green signal for at least the minimum green time duration
5. THE Traffic_Manager SHALL not exceed the maximum green time duration for any direction

### Requirement 4: Vehicle Stopping at Red Lights

**User Story:** As a driver, I want my vehicle to stop at red lights, so that I can safely wait for a green signal.

#### Acceptance Criteria

1. WHEN a vehicle approaches a red light and enters the Detection_Zone, THEN the Flow_Controller SHALL command the vehicle to stop
2. WHEN a vehicle is stopped at a red light, THEN it SHALL remain stationary until the light turns green
3. WHEN the light turns green for a stopped vehicle, THEN the Flow_Controller SHALL command the vehicle to resume movement
4. THE vehicle SHALL stop before crossing the Stop_Line when the light is red
5. WHEN a vehicle is beyond the Detection_Zone and the light is green, THEN the vehicle SHALL continue moving at its assigned speed

### Requirement 5: Collision Avoidance Between Vehicles

**User Story:** As a driver, I want my vehicle to maintain safe distance from other vehicles, so that collisions are prevented.

#### Acceptance Criteria

1. WHEN a vehicle detects another vehicle within the Safe_Following_Distance ahead, THEN the Flow_Controller SHALL reduce the vehicle's speed or stop it
2. WHEN the vehicle ahead moves and the distance exceeds the Safe_Following_Distance, THEN the Flow_Controller SHALL allow the following vehicle to resume normal speed
3. THE Flow_Controller SHALL continuously monitor the distance between vehicles in the same lane
4. WHEN multiple vehicles are queued at a red light, THEN they SHALL maintain the Safe_Following_Distance between each other
5. THE collision avoidance logic SHALL work independently of the traffic light state

### Requirement 6: Vehicle Movement Coordination

**User Story:** As a traffic system operator, I want vehicles to move smoothly through the intersection, so that traffic flows efficiently without collisions.

#### Acceptance Criteria

1. WHEN a vehicle receives a velocity command, THEN the system SHALL publish it to the correct Gazebo topic for that vehicle
2. THE Flow_Controller SHALL publish velocity commands at a rate sufficient for smooth vehicle control (at least 10 Hz)
3. WHEN a vehicle is commanded to stop, THEN its velocity SHALL be set to zero
4. WHEN a vehicle is commanded to move, THEN its velocity SHALL be set to its assigned speed in the correct direction
5. THE system SHALL use the correct velocity axis (linear.y for north/south, linear.x for east/west) based on vehicle orientation

### Requirement 7: Sensor Placement and Configuration

**User Story:** As a traffic system operator, I want sensors positioned optimally, so that vehicles are detected at the right time and location.

#### Acceptance Criteria

1. THE Logical_Camera sensors SHALL be positioned to cover the approach lanes before the Stop_Line
2. THE Logical_Camera sensors SHALL have sufficient field of view to detect all vehicles in their assigned lanes
3. THE Logical_Camera sensors SHALL be positioned close enough to detect vehicles waiting at the Stop_Line
4. WHEN a sensor is positioned, THEN it SHALL be oriented to look down at the road surface
5. THE sensor update rate SHALL be sufficient for real-time vehicle detection (at least 5 Hz)

### Requirement 8: System Integration and Communication

**User Story:** As a system integrator, I want all ROS 2 and Gazebo components to communicate correctly, so that the system functions as a cohesive whole.

#### Acceptance Criteria

1. THE ROS-Gazebo bridge SHALL correctly map all vehicle velocity command topics
2. THE ROS-Gazebo bridge SHALL correctly map all logical camera sensor topics
3. THE ROS-Gazebo bridge SHALL correctly map the traffic light control topic
4. WHEN the Traffic_Manager publishes a light command, THEN it SHALL reach the Gazebo world within 100 milliseconds
5. WHEN a Logical_Camera detects a vehicle, THEN the sensor data SHALL reach the Traffic_Manager within 200 milliseconds

### Requirement 9: Vehicle Initialization and Reset

**User Story:** As a simulation operator, I want vehicles to start in proper positions and states, so that the simulation begins correctly.

#### Acceptance Criteria

1. WHEN the simulation starts, THEN all vehicles SHALL be positioned in their designated lanes
2. WHEN the simulation starts, THEN all vehicles SHALL be assigned random speeds within the configured range
3. THE vehicle initial positions SHALL ensure they do not overlap or collide at startup
4. THE vehicle orientations SHALL match their direction of travel
5. WHEN the simulation starts, THEN the Traffic_Manager SHALL initialize to a known state (NS_GREEN)

### Requirement 10: Error Handling and Robustness

**User Story:** As a system operator, I want the system to handle errors gracefully, so that temporary issues do not cause system failure.

#### Acceptance Criteria

1. WHEN a sensor fails to provide data, THEN the Traffic_Manager SHALL use the last known vehicle count for that direction
2. WHEN the ROS-Gazebo bridge is temporarily disconnected, THEN the system SHALL continue operating with cached data
3. WHEN invalid sensor data is received, THEN the system SHALL log a warning and ignore the invalid data
4. THE Traffic_Manager SHALL log state transitions for debugging and monitoring
5. THE Flow_Controller SHALL log vehicle control decisions for debugging and monitoring
