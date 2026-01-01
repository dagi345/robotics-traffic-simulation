# Design Document: Traffic Control System Fix

## Overview

This design addresses the critical issues preventing proper vehicle control in the Smart Traffic Light System simulation. The solution involves fixing traffic light naming and control, repositioning sensors for optimal detection, implementing collision avoidance logic, and ensuring proper vehicle stopping behavior at red lights.

The design follows a modular architecture with clear separation between traffic signal management, vehicle flow control, and sensor data processing. All components communicate via ROS 2 topics with proper bridging to Gazebo.

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo Simulation                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Traffic      │  │  Logical     │  │   Vehicles   │      │
│  │ Lights       │  │  Cameras     │  │  (16 Prius)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
           │                  │                  │
           │ Light Commands   │ Sensor Data      │ Velocity Cmds
           │                  │                  │
┌──────────┴──────────────────┴──────────────────┴─────────────┐
│                    ROS-Gazebo Bridge                          │
└──────────┬──────────────────┬──────────────────┬─────────────┘
           │                  │                  │
    ┌──────▼──────┐    ┌─────▼──────┐    ┌─────▼──────┐
    │   Traffic   │    │   Traffic  │    │  Vehicle   │
    │   Manager   │───▶│    Flow    │───▶│  Position  │
    │             │    │ Controller │    │  Tracker   │
    └─────────────┘    └────────────┘    └────────────┘
         ROS 2              ROS 2              ROS 2
```

### Component Responsibilities

1. **Traffic_Manager Node**
   - Subscribes to logical camera sensor data
   - Counts vehicles in each direction
   - Implements adaptive signal timing logic
   - Publishes traffic light commands
   - Publishes current traffic state

2. **Flow_Controller Node**
   - Subscribes to traffic light state
   - Subscribes to logical camera sensor data
   - Tracks vehicle positions (via sensor data)
   - Implements collision avoidance logic
   - Publishes velocity commands for each vehicle

3. **Vehicle_Position_Tracker** (New Component)
   - Subscribes to all logical camera sensors
   - Maintains a map of vehicle positions
   - Calculates distances between vehicles in the same lane
   - Provides position data to Flow_Controller

4. **Logical_Cameras**
   - Positioned at optimal locations to detect approaching vehicles
   - Provide real-time vehicle detection data
   - Cover the approach lanes and stop line areas

5. **Traffic_Lights**
   - Named: `tl_north`, `tl_south`, `tl_east`, `tl_west`
   - Controllable via light intensity commands
   - Positioned to be visible to approaching vehicles

## Components and Interfaces

### Traffic Light Control

**Issue:** Current code tries to control lights named `traffic_light_nw/ne/sw/se` but the world file has `tl_north/south/east/west`.

**Solution:**
```python
# Correct traffic light names matching world file
TRAFFIC_LIGHTS = {
    'north': 'tl_north',
    'south': 'tl_south', 
    'east': 'tl_east',
    'west': 'tl_west'
}

def update_lights(self, state):
    """Update traffic lights based on current state"""
    if state == "NS_GREEN":
        self.set_light('north', green=True, red=False)
        self.set_light('south', green=True, red=False)
        self.set_light('east', green=False, red=True)
        self.set_light('west', green=False, red=True)
    elif state == "EW_GREEN":
        self.set_light('north', green=False, red=True)
        self.set_light('south', green=False, red=True)
        self.set_light('east', green=True, red=False)
        self.set_light('west', green=True, red=False)
    elif state == "ALL_RED":
        for direction in ['north', 'south', 'east', 'west']:
            self.set_light(direction, green=False, red=True)

def set_light(self, direction, green, red):
    """Set individual light state"""
    light_name = TRAFFIC_LIGHTS[direction]
    
    # Set green lamp
    green_msg = Light()
    green_msg.name = f"{light_name}::housing::green_lamp"
    green_msg.intensity = 1.0 if green else 0.0
    self.light_pub.publish(green_msg)
    
    # Set red lamp
    red_msg = Light()
    red_msg.name = f"{light_name}::housing::red_lamp"
    red_msg.intensity = 1.0 if red else 0.0
    self.light_pub.publish(red_msg)
```

### Sensor Positioning

**Issue:** Sensors are positioned too far from the intersection (at ±30m), causing detection gaps.

**Solution:** Reposition sensors closer to the intersection to detect vehicles approaching the stop line:

```xml
<!-- North sensor: detects southbound traffic -->
<model name="sensor_north_road">
  <static>true</static>
  <pose>0 20 10 0 1.5707 0</pose>  <!-- Changed from y=30 to y=20 -->
  <link name="link">
    <sensor name="logical_camera" type="logical_camera">
      <logical_camera>
        <near>1</near>
        <far>15</far>  <!-- Reduced from 30 to 15 -->
        <horizontal_fov>1.5</horizontal_fov>
        <aspect_ratio>1</aspect_ratio>
      </logical_camera>
      <visualize>true</visualize>
      <update_rate>10</update_rate>  <!-- Increased from 5 to 10 Hz -->
    </sensor>
  </link>
</model>

<!-- Similar adjustments for south (y=-20), east (x=20), west (x=-20) -->
```

**Rationale:**
- Position at 20m allows detection of vehicles from 5m to 20m from intersection center
- Covers the stop line (at ±13m) and approach area
- 15m range ensures vehicles are tracked until they enter the intersection
- 10 Hz update rate provides smoother tracking

### Vehicle Position Tracking

**New Component:** Track vehicle positions to enable collision avoidance.

```python
class VehiclePositionTracker:
    """Tracks positions of all vehicles using sensor data"""
    
    def __init__(self):
        self.vehicle_positions = {}  # {vehicle_name: (x, y, z)}
        self.vehicle_lanes = {
            # Map vehicle names to their lanes
            'prius_n1': 'north', 'prius_n2': 'north', ...
            'prius_s1': 'south', 'prius_s2': 'south', ...
            'prius_e1': 'east', 'prius_e2': 'east', ...
            'prius_w1': 'west', 'prius_w2': 'west', ...
        }
        
        # Subscribe to all sensors
        self.create_subscription(LogicalCameraImage, '/sensors/north', 
                                self.sensor_callback_north, 10)
        # ... similar for south, east, west
    
    def sensor_callback_north(self, msg):
        """Process sensor data from north sensor"""
        for model in msg.models:
            if model.name.startswith('prius_'):
                # Extract position from sensor data
                pos = model.pose.position
                self.vehicle_positions[model.name] = (pos.x, pos.y, pos.z)
    
    def get_distance_to_vehicle_ahead(self, vehicle_name):
        """Calculate distance to the vehicle ahead in the same lane"""
        if vehicle_name not in self.vehicle_positions:
            return float('inf')
        
        my_pos = self.vehicle_positions[vehicle_name]
        my_lane = self.vehicle_lanes[vehicle_name]
        
        # Find vehicles in the same lane ahead of this vehicle
        min_distance = float('inf')
        
        for other_name, other_pos in self.vehicle_positions.items():
            if other_name == vehicle_name:
                continue
            if self.vehicle_lanes.get(other_name) != my_lane:
                continue
            
            # Check if other vehicle is ahead
            if my_lane == 'north':
                if other_pos[1] < my_pos[1]:  # Other is ahead (lower y)
                    distance = my_pos[1] - other_pos[1]
                    min_distance = min(min_distance, distance)
            elif my_lane == 'south':
                if other_pos[1] > my_pos[1]:  # Other is ahead (higher y)
                    distance = other_pos[1] - my_pos[1]
                    min_distance = min(min_distance, distance)
            elif my_lane == 'east':
                if other_pos[0] > my_pos[0]:  # Other is ahead (higher x)
                    distance = other_pos[0] - my_pos[0]
                    min_distance = min(min_distance, distance)
            elif my_lane == 'west':
                if other_pos[0] < my_pos[0]:  # Other is ahead (lower x)
                    distance = my_pos[0] - other_pos[0]
                    min_distance = min(min_distance, distance)
        
        return min_distance
```

### Collision Avoidance Logic

**Issue:** Vehicles don't maintain safe following distances, causing collisions.

**Solution:** Implement distance-based speed control in Flow_Controller:

```python
class MultiTrafficFlowControl(Node):
    def __init__(self):
        # ... existing initialization ...
        
        self.SAFE_FOLLOWING_DISTANCE = 5.0  # meters
        self.STOP_DISTANCE = 3.0  # meters - emergency stop
        
        # Create position tracker
        self.position_tracker = VehiclePositionTracker()
    
    def timer_callback(self):
        """Control vehicle velocities with collision avoidance"""
        # Determine blocked roads based on light state
        blocked_roads = self.get_blocked_roads()
        
        for vehicle_name, pub in self.car_publishers.items():
            msg = Twist()
            
            # Get vehicle's road direction
            car_road = vehicle_name.split('_')[1][0]
            
            # Check if vehicle should stop for red light
            is_at_red_light = (car_road in blocked_roads and 
                              vehicle_name in self.in_zones[car_road])
            
            # Check distance to vehicle ahead
            distance_ahead = self.position_tracker.get_distance_to_vehicle_ahead(
                vehicle_name)
            
            # Determine velocity based on conditions
            if is_at_red_light:
                # Stop for red light
                velocity = 0.0
            elif distance_ahead < self.STOP_DISTANCE:
                # Emergency stop - too close to vehicle ahead
                velocity = 0.0
            elif distance_ahead < self.SAFE_FOLLOWING_DISTANCE:
                # Slow down - approaching safe following distance
                # Linear interpolation between 0 and normal speed
                ratio = (distance_ahead - self.STOP_DISTANCE) / \
                       (self.SAFE_FOLLOWING_DISTANCE - self.STOP_DISTANCE)
                velocity = self.speeds[vehicle_name] * ratio
            else:
                # Normal speed - safe distance maintained
                velocity = self.speeds[vehicle_name]
            
            # Set velocity in correct axis based on vehicle orientation
            if car_road in ['n', 's']:
                msg.linear.y = -velocity if car_road == 's' else velocity
            else:  # 'e' or 'w'
                msg.linear.x = velocity if car_road == 'e' else -velocity
            
            pub.publish(msg)
    
    def get_blocked_roads(self):
        """Get list of roads that should stop for red light"""
        if self.light_state == "NS_GREEN":
            return ['e', 'w']
        elif self.light_state == "EW_GREEN":
            return ['n', 's']
        elif self.light_state == "ALL_RED":
            return ['n', 's', 'e', 'w']
        return []
```

## Data Models

### Vehicle State
```python
@dataclass
class VehicleState:
    name: str
    position: Tuple[float, float, float]  # (x, y, z)
    lane: str  # 'north', 'south', 'east', 'west'
    assigned_speed: float
    current_velocity: float
    in_detection_zone: bool
```

### Traffic Light State
```python
class TrafficLightState(Enum):
    NS_GREEN = "NS_GREEN"
    EW_GREEN = "EW_GREEN"
    ALL_RED = "ALL_RED"
```

### Sensor Configuration
```python
@dataclass
class SensorConfig:
    name: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float]  # roll, pitch, yaw
    near_range: float
    far_range: float
    fov: float
    update_rate: float
```



## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: Traffic Light State Mapping

*For any* traffic light state (NS_GREEN, EW_GREEN, or ALL_RED), the traffic light configuration SHALL correctly map to the expected light colors: NS_GREEN sets north/south to green and east/west to red, EW_GREEN sets east/west to green and north/south to red, and ALL_RED sets all lights to red.

**Validates: Requirements 1.2, 1.3, 1.4**

### Property 2: Vehicle Count Accuracy

*For any* sensor data message received, the Traffic_Manager's vehicle count for that direction SHALL equal the number of vehicle models in the sensor data.

**Validates: Requirements 2.2**

### Property 3: Vehicle Detection in Zone

*For any* vehicle positioned within a Detection_Zone's range and field of view, the Logical_Camera SHALL include that vehicle in its sensor data output.

**Validates: Requirements 2.1**

### Property 4: Count Update on Exit

*For any* vehicle that exits a Detection_Zone (present in previous sensor data but not in current), the vehicle count SHALL decrease by one.

**Validates: Requirements 2.5**

### Property 5: Adaptive Signal Switching

*For any* traffic state where one direction has waiting vehicles and the perpendicular direction is empty, after the minimum green time has elapsed, the Traffic_Manager SHALL initiate a transition to give the waiting direction a green signal.

**Validates: Requirements 3.1, 3.2**

### Property 6: ALL_RED Transition

*For any* transition from NS_GREEN to EW_GREEN or from EW_GREEN to NS_GREEN, the system SHALL pass through the ALL_RED state for at least the configured safety interval duration.

**Validates: Requirements 3.3**

### Property 7: Green Time Bounds

*For any* green state duration, it SHALL be greater than or equal to the minimum green time and less than or equal to the maximum green time (unless no perpendicular traffic exists).

**Validates: Requirements 3.4, 3.5**

### Property 8: Red Light Stopping

*For any* vehicle in a Detection_Zone when its direction has a red light, the Flow_Controller SHALL command that vehicle's velocity to zero.

**Validates: Requirements 4.1, 4.2**

### Property 9: Green Light Resumption

*For any* vehicle that was stopped at a red light, when the light turns green for its direction, the Flow_Controller SHALL command a non-zero velocity for that vehicle.

**Validates: Requirements 4.3**

### Property 10: Stop Line Compliance

*For any* vehicle approaching a red light, its position SHALL remain on the approach side of the Stop_Line (not crossing into the intersection).

**Validates: Requirements 4.4**

### Property 11: Green Light Movement

*For any* vehicle outside the Detection_Zone when its direction has a green light and no vehicle is within Safe_Following_Distance ahead, the velocity command SHALL equal the vehicle's assigned speed.

**Validates: Requirements 4.5**

### Property 12: Collision Avoidance Speed Reduction

*For any* vehicle with another vehicle within the Safe_Following_Distance ahead in the same lane, the Flow_Controller SHALL command a velocity less than or equal to a safe speed (proportional to distance).

**Validates: Requirements 5.1**

### Property 13: Speed Resumption After Safe Distance

*For any* vehicle that had reduced speed due to a vehicle ahead, when the distance to the vehicle ahead exceeds the Safe_Following_Distance, the Flow_Controller SHALL allow the velocity to increase toward the assigned speed.

**Validates: Requirements 5.2**

### Property 14: Queue Spacing

*For any* set of vehicles queued at a red light in the same lane, the distance between each consecutive pair SHALL be greater than or equal to the Safe_Following_Distance.

**Validates: Requirements 5.4**

### Property 15: Collision Avoidance Independence

*For any* traffic light state (red, green, or transitioning), the collision avoidance logic SHALL still enforce Safe_Following_Distance constraints between vehicles in the same lane.

**Validates: Requirements 5.5**

### Property 16: Velocity Command Routing

*For any* vehicle, velocity commands SHALL be published to the topic `/model/{vehicle_name}/cmd_vel` where {vehicle_name} matches the vehicle's identifier.

**Validates: Requirements 6.1**

### Property 17: Velocity Command Values

*For any* vehicle, when commanded to stop the velocity SHALL be zero, and when commanded to move the velocity SHALL equal the assigned speed with correct sign for direction.

**Validates: Requirements 6.3, 6.4**

### Property 18: Velocity Axis Mapping

*For any* vehicle, the velocity command SHALL use linear.y for north/south lanes and linear.x for east/west lanes, with correct sign based on direction of travel.

**Validates: Requirements 6.5**

### Property 19: Initial Speed Assignment

*For any* vehicle at simulation start, the assigned speed SHALL be within the configured minimum and maximum speed range.

**Validates: Requirements 9.2**

### Property 20: Initial Vehicle Spacing

*For any* pair of vehicles in the same lane at simulation start, the distance between them SHALL be greater than or equal to the Safe_Following_Distance.

**Validates: Requirements 9.3**

### Property 21: Sensor Failure Handling

*For any* sensor that fails to provide data for a timeout period, the Traffic_Manager SHALL maintain the last known vehicle count for that direction until new data arrives.

**Validates: Requirements 10.1**

### Property 22: Invalid Data Handling

*For any* sensor data message that fails validation checks, the system SHALL ignore the invalid data and log a warning message.

**Validates: Requirements 10.3**

### Property 23: State Transition Logging

*For any* traffic light state transition, the Traffic_Manager SHALL create a log entry containing the previous state, new state, and timestamp.

**Validates: Requirements 10.4**

### Property 24: Control Decision Logging

*For any* vehicle control decision that changes a vehicle's velocity, the Flow_Controller SHALL create a log entry containing the vehicle name, decision reason, and commanded velocity.

**Validates: Requirements 10.5**

## Error Handling

### Traffic Light Control Errors

1. **Invalid Light Name**: If a light command references a non-existent light model, log an error and skip the command
2. **Bridge Communication Failure**: If light commands fail to reach Gazebo, retry up to 3 times with exponential backoff
3. **Light State Inconsistency**: If light state query returns unexpected values, log a warning and continue with assumed state

### Sensor Data Errors

1. **Missing Sensor Data**: If a sensor hasn't published data for 1 second, use last known count and log a warning
2. **Malformed Sensor Data**: If sensor data cannot be parsed, ignore the message and log an error
3. **Sensor Position Errors**: If vehicle positions are outside expected ranges, validate against world bounds before using

### Vehicle Control Errors

1. **Invalid Vehicle Name**: If a velocity command references an unknown vehicle, log an error and skip
2. **Velocity Command Failure**: If a command fails to publish, retry on next control cycle
3. **Position Tracking Loss**: If a vehicle's position is unknown, assume it's outside detection zones and use default behavior

### Collision Detection Errors

1. **Distance Calculation Failure**: If distance cannot be calculated (missing position data), assume safe distance
2. **Lane Assignment Error**: If a vehicle's lane cannot be determined, log an error and use conservative (stopped) behavior
3. **Multiple Vehicles at Same Position**: If two vehicles report identical positions, log a warning and treat as collision risk

## Testing Strategy

### Dual Testing Approach

This system will be validated using both unit tests and property-based tests:

- **Unit tests**: Verify specific examples, edge cases, and error conditions
- **Property tests**: Verify universal properties across all inputs
- Both approaches are complementary and necessary for comprehensive coverage

### Unit Testing Focus

Unit tests will cover:

1. **Specific Scenarios**
   - Single vehicle approaching red light
   - Two vehicles queued at red light
   - Light transition from NS_GREEN to EW_GREEN
   - Vehicle entering and exiting detection zone

2. **Edge Cases**
   - Vehicle exactly at Safe_Following_Distance
   - Vehicle exactly at Stop_Line
   - Simultaneous state transitions
   - Empty intersection (no vehicles)

3. **Error Conditions**
   - Missing sensor data
   - Invalid light commands
   - Malformed vehicle names
   - Out-of-range positions

4. **Integration Points**
   - ROS topic communication
   - Gazebo bridge functionality
   - Multi-node coordination

### Property-Based Testing

Property tests will use **Hypothesis** (Python PBT library) with minimum 100 iterations per test.

Each property test will:
- Generate random vehicle configurations (positions, speeds, lanes)
- Generate random traffic light states and timing
- Generate random sensor data
- Verify the corresponding correctness property holds

**Test Configuration:**
```python
from hypothesis import given, settings
import hypothesis.strategies as st

@settings(max_examples=100)
@given(
    vehicle_positions=st.lists(st.tuples(st.floats(-50, 50), st.floats(-50, 50)), 
                                min_size=1, max_size=16),
    light_state=st.sampled_from(['NS_GREEN', 'EW_GREEN', 'ALL_RED']),
    vehicle_speeds=st.lists(st.floats(3.0, 5.0), min_size=1, max_size=16)
)
def test_property_X(vehicle_positions, light_state, vehicle_speeds):
    """Feature: traffic-control-fix, Property X: [property description]"""
    # Test implementation
    pass
```

### Test Organization

```
tests/
├── unit/
│   ├── test_traffic_manager.py
│   ├── test_flow_controller.py
│   ├── test_position_tracker.py
│   └── test_integration.py
├── property/
│   ├── test_light_properties.py
│   ├── test_collision_properties.py
│   ├── test_timing_properties.py
│   └── test_control_properties.py
└── fixtures/
    ├── sensor_data.py
    ├── vehicle_configs.py
    └── world_configs.py
```

### Testing Priorities

**High Priority** (Must pass before deployment):
- Property 8: Red Light Stopping
- Property 12: Collision Avoidance Speed Reduction
- Property 14: Queue Spacing
- Property 1: Traffic Light State Mapping
- Property 6: ALL_RED Transition

**Medium Priority** (Should pass for full functionality):
- Property 5: Adaptive Signal Switching
- Property 7: Green Time Bounds
- Property 10: Stop Line Compliance
- Property 16: Velocity Command Routing
- Property 18: Velocity Axis Mapping

**Low Priority** (Nice to have for robustness):
- Property 21: Sensor Failure Handling
- Property 22: Invalid Data Handling
- Property 23: State Transition Logging
- Property 24: Control Decision Logging
