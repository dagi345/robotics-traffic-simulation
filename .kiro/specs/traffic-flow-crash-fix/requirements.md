# Requirements Document: Traffic Flow Crash Fix

## Introduction

This specification addresses critical bugs causing vehicles to crash at the intersection. The diagnosis revealed that vehicles are not properly controlled by traffic lights due to incorrect velocity axis mapping, faulty position tracking, and detection zone issues. This fix will ensure vehicles stop at red lights, move on green lights, and never collide at the intersection.

## Glossary

- **Velocity_Axis**: The coordinate axis (X or Y) used to control vehicle forward motion
- **Detection_Zone**: Area monitored by sensors where vehicles must obey traffic signals
- **Stop_Line**: Physical line at ±13m where vehicles must stop for red lights
- **Vehicle_Direction**: Cardinal direction of vehicle travel (north/south/east/west)
- **Position_Tracker**: Component that monitors vehicle locations for collision avoidance
- **Intersection_Zone**: Central area (-13m to +13m) where roads cross

## Requirements

### Requirement 1: Correct Velocity Axis Control

**User Story:** As a simulation operator, I want vehicles to move in their intended direction, so that east-west vehicles don't move incorrectly.

#### Acceptance Criteria

1. WHEN a northbound vehicle receives a velocity command, THEN the system SHALL use positive linear.y
2. WHEN a southbound vehicle receives a velocity command, THEN the system SHALL use negative linear.y
3. WHEN an eastbound vehicle receives a velocity command, THEN the system SHALL use positive linear.x
4. WHEN a westbound vehicle receives a velocity command, THEN the system SHALL use negative linear.x
5. THE system SHALL determine velocity axis based on vehicle name prefix (n/s/e/w)

### Requirement 2: Functional Vehicle Position Tracking

**User Story:** As a traffic controller, I want accurate vehicle position tracking, so that collision avoidance works correctly.

#### Acceptance Criteria

1. THE VehiclePositionTracker SHALL be implemented as a standalone ROS node OR as a helper class
2. WHEN a vehicle is detected by a sensor, THEN its position SHALL be recorded within 100ms
3. THE position tracker SHALL maintain positions for all 16 vehicles
4. WHEN calculating distance to vehicle ahead, THEN the system SHALL return accurate distance in meters
5. THE position tracking SHALL update at least 10 times per second

### Requirement 3: Unique Vehicle Starting Positions

**User Story:** As a simulation operator, I want all vehicles to start in unique positions, so that no collisions occur at startup.

#### Acceptance Criteria

1. WHEN the simulation starts, THEN no two vehicles SHALL occupy the same position
2. THE minimum distance between any two vehicles at startup SHALL be at least 5 meters
3. WHEN vehicles are positioned, THEN they SHALL be aligned with their designated lanes
4. THE vehicle positions SHALL be verified before simulation launch

### Requirement 4: Complete Detection Zone Coverage

**User Story:** As a traffic controller, I want sensors to detect all vehicles approaching the intersection, so that red light enforcement works correctly.

#### Acceptance Criteria

1. THE detection zones SHALL cover all areas from stop lines to at least 20 meters away
2. WHEN a vehicle is within 20 meters of a stop line, THEN it SHALL be detected by the appropriate sensor
3. THE detection zones SHALL have no gaps between sensor coverage and stop lines
4. WHEN a vehicle crosses the stop line, THEN it SHALL still be tracked until it exits the intersection

### Requirement 5: Absolute Red Light Enforcement

**User Story:** As a driver, I want my vehicle to ALWAYS stop at red lights, so that I don't enter the intersection illegally.

#### Acceptance Criteria

1. WHEN a vehicle is in a detection zone AND its direction has a red light, THEN its velocity SHALL be set to zero
2. THE vehicle SHALL remain stopped until the light turns green for its direction
3. WHEN a vehicle is stopped at a red light, THEN it SHALL be positioned before the stop line
4. THE system SHALL enforce red lights with 100% reliability (no vehicles run red lights)
5. WHEN multiple vehicles queue at a red light, THEN they SHALL maintain safe following distance

### Requirement 6: Smooth Velocity Transitions

**User Story:** As a simulation observer, I want vehicles to accelerate and decelerate smoothly, so that the simulation looks realistic.

#### Acceptance Criteria

1. WHEN a vehicle needs to stop, THEN it SHALL decelerate gradually over at least 1 second
2. WHEN a vehicle resumes from stopped, THEN it SHALL accelerate gradually to target speed
3. THE acceleration rate SHALL not exceed 3 m/s²
4. THE deceleration rate SHALL not exceed 5 m/s²
5. WHEN a vehicle approaches a red light, THEN it SHALL begin slowing down early

### Requirement 7: Intersection Collision Prevention

**User Story:** As a traffic system operator, I want zero collisions at the intersection, so that the simulation demonstrates safe traffic control.

#### Acceptance Criteria

1. WHEN vehicles from perpendicular directions have conflicting green lights, THEN the system SHALL prevent simultaneous intersection entry
2. WHEN a vehicle is in the intersection zone, THEN conflicting traffic SHALL remain stopped
3. THE system SHALL verify intersection is clear before allowing phase changes
4. WHEN the simulation runs for 10 minutes, THEN zero collisions SHALL occur at the intersection
5. THE system SHALL log any near-miss events (vehicles within 2m of collision)

### Requirement 8: Correct Sensor-Direction Mapping

**User Story:** As a developer, I want clear and correct sensor-to-direction mapping, so that the code is maintainable and bug-free.

#### Acceptance Criteria

1. THE code SHALL document which sensor detects which traffic direction
2. WHEN the north sensor detects a vehicle, THEN it SHALL be assigned to southbound traffic control
3. WHEN the south sensor detects a vehicle, THEN it SHALL be assigned to northbound traffic control
4. WHEN the east sensor detects a vehicle, THEN it SHALL be assigned to westbound traffic control
5. WHEN the west sensor detects a vehicle, THEN it SHALL be assigned to eastbound traffic control

### Requirement 9: Vehicle Queue Management

**User Story:** As a driver, I want to maintain safe distance from the vehicle ahead, so that rear-end collisions don't occur.

#### Acceptance Criteria

1. WHEN a vehicle detects another vehicle within 5 meters ahead, THEN it SHALL reduce speed proportionally
2. WHEN a vehicle detects another vehicle within 3 meters ahead, THEN it SHALL stop immediately
3. THE system SHALL maintain safe following distance regardless of traffic light state
4. WHEN multiple vehicles queue at a red light, THEN they SHALL form an orderly line
5. WHEN the front vehicle in a queue starts moving, THEN following vehicles SHALL resume in order

### Requirement 10: System Validation and Testing

**User Story:** As a quality assurance engineer, I want comprehensive tests, so that I can verify the system works correctly.

#### Acceptance Criteria

1. THE system SHALL include unit tests for velocity axis selection
2. THE system SHALL include integration tests for complete traffic cycles
3. THE system SHALL include property-based tests for collision avoidance
4. WHEN tests are run, THEN they SHALL verify zero red-light violations
5. THE system SHALL include a simulation test that runs for 10 minutes without crashes
