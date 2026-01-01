# Requirements Document

## Introduction

This specification defines the requirements for completing the Smart Adaptive Traffic Light System simulation. The system must provide intelligent traffic control that responds to real-time vehicle and pedestrian conditions, demonstrating improved traffic flow compared to traditional fixed-time systems. The key additions include fixing intersection collision logic, adding pedestrian simulation with detection-based crossing, and implementing traffic flow metrics for comparison.

## Glossary

- **Traffic_Manager**: The ROS 2 node responsible for analyzing sensor data and controlling traffic light states for both vehicles and pedestrians
- **Flow_Controller**: The ROS 2 node responsible for controlling individual vehicle velocities based on traffic light states and collision avoidance
- **Pedestrian_Controller**: The ROS 2 node responsible for controlling pedestrian actor movements based on crossing signals
- **Logical_Camera**: Gazebo sensor that detects vehicles or pedestrians within a defined field of view
- **Pedestrian_Sensor**: Logical camera positioned to detect pedestrians waiting at crosswalks
- **Traffic_Light**: Gazebo model with controllable red, yellow, and green lamp intensities
- **Pedestrian_Signal**: Visual indicator showing walk/don't walk status for pedestrians
- **Vehicle**: Prius Hybrid model controlled via velocity commands
- **Pedestrian_Actor**: Animated walking human model using gazebo-ros-actor-plugin
- **Intersection_Zone**: The central area where roads cross (approximately -13m to +13m in both X and Y axes)
- **Crosswalk**: Designated pedestrian crossing area at each approach to the intersection
- **Safe_Crossing_Time**: Maximum time allowed for pedestrians to cross (configurable, default 15 seconds)
- **Clearance_Interval**: Time between pedestrian crossing end and vehicle green (yellow light duration)
- **Metrics_Collector**: Component that tracks and reports traffic flow statistics

## Requirements

### Requirement 1: Intersection Collision Prevention

**User Story:** As a traffic system operator, I want vehicles from conflicting directions to never enter the intersection simultaneously, so that collisions are prevented.

#### Acceptance Criteria

1. WHEN the traffic light state is NS_GREEN, THEN the system SHALL prevent east-west vehicles from entering the Intersection_Zone
2. WHEN the traffic light state is EW_GREEN, THEN the system SHALL prevent north-south vehicles from entering the Intersection_Zone
3. WHEN the traffic light state is ALL_RED, THEN the system SHALL prevent all vehicles from entering the Intersection_Zone
4. WHEN a vehicle is already inside the Intersection_Zone, THEN the system SHALL allow it to complete its crossing before stopping conflicting traffic
5. THE system SHALL enforce a minimum clearance time between conflicting green phases to allow vehicles to clear the intersection
6. WHEN transitioning between green phases, THEN the system SHALL verify the Intersection_Zone is clear before allowing the new direction to proceed

### Requirement 2: Pedestrian Actor Simulation

**User Story:** As a simulation operator, I want realistic pedestrian actors at crosswalks, so that the system can demonstrate pedestrian-aware traffic control.

#### Acceptance Criteria

1. THE system SHALL include 4-8 pedestrian actors per crosswalk (16-32 total pedestrians)
2. WHEN pedestrians are spawned, THEN they SHALL be positioned near crosswalk waiting areas
3. THE pedestrian actors SHALL use the DoctorFemaleWalk animated model from gazebo-ros-actor-plugin
4. WHEN pedestrians are waiting, THEN they SHALL exhibit idle behavior (stationary or slight movement)
5. WHEN pedestrians are crossing, THEN they SHALL walk across the crosswalk at realistic speeds (1.0-1.5 m/s)
6. THE pedestrian groups SHALL move with natural variation (not all at exactly the same speed or path)

### Requirement 3: Pedestrian Detection System

**User Story:** As a traffic system operator, I want the system to detect pedestrians waiting at crosswalks, so that it can provide safe crossing opportunities.

#### Acceptance Criteria

1. THE system SHALL include Pedestrian_Sensors positioned to detect pedestrians in crosswalk waiting areas
2. WHEN a pedestrian enters a crosswalk waiting area, THEN the Pedestrian_Sensor SHALL detect their presence within 500 milliseconds
3. THE Pedestrian_Sensor SHALL accurately count the number of pedestrians waiting at each crosswalk
4. WHEN pedestrians leave the waiting area (to cross or depart), THEN the count SHALL update accordingly
5. THE Pedestrian_Sensors SHALL cover all four crosswalk approaches (north, south, east, west)

### Requirement 4: Smart Pedestrian Crossing Logic

**User Story:** As a pedestrian, I want the traffic system to detect me waiting and provide a safe crossing opportunity, so that I can cross the road safely.

#### Acceptance Criteria

1. WHEN pedestrians are detected waiting at a crosswalk, THEN the Traffic_Manager SHALL initiate a pedestrian crossing phase within a reasonable time (max 60 seconds wait)
2. WHEN a pedestrian crossing phase begins, THEN the system SHALL turn the vehicle traffic light red for the road being crossed
3. WHEN the vehicle traffic light turns red, THEN the system SHALL wait for the Clearance_Interval before signaling pedestrians to cross
4. WHEN the pedestrian signal shows "walk", THEN the Pedestrian_Controller SHALL command waiting pedestrians to cross
5. THE pedestrian crossing phase SHALL last for the configured Safe_Crossing_Time
6. WHEN the Safe_Crossing_Time expires, THEN the system SHALL transition through yellow to green for vehicles
7. IF pedestrians are still in the crosswalk when Safe_Crossing_Time expires, THEN the system SHALL extend the red light until the crosswalk is clear

### Requirement 5: Pedestrian Movement Control

**User Story:** As a simulation operator, I want pedestrians to move realistically in response to crossing signals, so that the simulation is believable.

#### Acceptance Criteria

1. WHEN the pedestrian signal shows "walk", THEN waiting pedestrians SHALL begin crossing the road
2. WHEN pedestrians are crossing, THEN they SHALL follow the crosswalk path to the opposite side
3. WHEN pedestrians reach the opposite side, THEN they SHALL stop and wait in the new waiting area
4. WHEN the pedestrian signal shows "don't walk", THEN pedestrians in the waiting area SHALL remain stationary
5. THE Pedestrian_Controller SHALL publish velocity commands to pedestrian actors via the gazebo-ros-actor-plugin interface
6. WHEN multiple pedestrians cross simultaneously, THEN they SHALL maintain reasonable spacing (not overlap)

### Requirement 6: Traffic Light State Machine Enhancement

**User Story:** As a traffic system operator, I want the traffic light system to handle both vehicle and pedestrian phases, so that all road users are safely managed.

#### Acceptance Criteria

1. THE Traffic_Manager SHALL support the following states: NS_GREEN, EW_GREEN, NS_PED_CROSSING, EW_PED_CROSSING, ALL_RED
2. WHEN in NS_PED_CROSSING state, THEN north and south crosswalks SHALL show "walk" and east-west traffic SHALL be stopped
3. WHEN in EW_PED_CROSSING state, THEN east and west crosswalks SHALL show "walk" and north-south traffic SHALL be stopped
4. THE state machine SHALL prioritize pedestrian safety over vehicle throughput
5. WHEN transitioning from a pedestrian phase to a vehicle phase, THEN the system SHALL include a yellow light warning period
6. THE system SHALL prevent conflicting pedestrian and vehicle movements

### Requirement 7: Yellow Light Implementation

**User Story:** As a driver, I want to see yellow lights during transitions, so that I have warning before the light changes.

#### Acceptance Criteria

1. THE traffic light model SHALL support yellow lamp control in addition to red and green
2. WHEN transitioning from green to red, THEN the system SHALL display yellow for a configurable duration (default 3 seconds)
3. WHEN the yellow light is displayed, THEN approaching vehicles SHALL begin slowing down
4. THE Flow_Controller SHALL reduce vehicle speed when approaching a yellow light
5. WHEN the yellow light duration expires, THEN the light SHALL turn red

### Requirement 8: Traffic Flow Metrics Collection

**User Story:** As a researcher, I want to collect traffic flow metrics, so that I can compare adaptive vs fixed-time performance.

#### Acceptance Criteria

1. THE Metrics_Collector SHALL track average vehicle wait time at each approach
2. THE Metrics_Collector SHALL track vehicle throughput (vehicles passing through per minute)
3. THE Metrics_Collector SHALL track maximum queue length at each approach
4. THE Metrics_Collector SHALL track pedestrian wait time before crossing
5. THE Metrics_Collector SHALL track total pedestrian crossings completed
6. THE system SHALL publish metrics to a ROS topic for monitoring and logging
7. THE system SHALL support running in both adaptive mode and fixed-time mode for comparison

### Requirement 9: Fixed-Time Baseline Mode

**User Story:** As a researcher, I want to run the system in fixed-time mode, so that I can compare it against the adaptive mode.

#### Acceptance Criteria

1. THE Traffic_Manager SHALL support a configurable mode parameter (adaptive vs fixed-time)
2. WHEN in fixed-time mode, THEN the system SHALL cycle through phases at fixed intervals regardless of traffic
3. WHEN in fixed-time mode, THEN pedestrian phases SHALL occur at fixed intervals
4. THE fixed-time cycle duration SHALL be configurable (default 60 seconds per full cycle)
5. THE system SHALL log which mode is active for metric correlation

### Requirement 10: Intersection Zone Monitoring

**User Story:** As a traffic system operator, I want to monitor the intersection zone, so that the system knows when it's safe to change phases.

#### Acceptance Criteria

1. THE system SHALL include a sensor or detection method to identify vehicles currently in the Intersection_Zone
2. WHEN a vehicle enters the Intersection_Zone, THEN the system SHALL track it until it exits
3. WHEN checking if a phase change is safe, THEN the system SHALL verify no conflicting vehicles are in the Intersection_Zone
4. THE Intersection_Zone monitoring SHALL update at least 10 times per second
5. IF a vehicle is stuck in the Intersection_Zone for more than 10 seconds, THEN the system SHALL log a warning

### Requirement 11: Pedestrian Signal Visualization

**User Story:** As a simulation observer, I want to see pedestrian crossing signals, so that I can understand the system state.

#### Acceptance Criteria

1. THE simulation SHALL include visual pedestrian signal indicators at each crosswalk
2. WHEN pedestrians should wait, THEN the signal SHALL display a red/stop indicator
3. WHEN pedestrians may cross, THEN the signal SHALL display a green/walk indicator
4. THE pedestrian signals SHALL be synchronized with the Traffic_Manager state
5. THE pedestrian signal state SHALL be published to a ROS topic for monitoring

</content>
