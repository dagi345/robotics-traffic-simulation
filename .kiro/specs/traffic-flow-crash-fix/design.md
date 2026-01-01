# Design Document: Traffic Flow Crash Fix

## Overview

This design addresses the critical bugs identified in the traffic system diagnosis that cause vehicles to crash at the intersection. The primary issues are: incorrect velocity axis mapping for east/west vehicles, non-functional vehicle position tracking, duplicate vehicle positions, and insufficient detection zone coverage.

The solution involves fixing the velocity control logic, refactoring the position tracker, correcting vehicle positions, and ensuring complete sensor coverage.

## Architecture

### Current System (Broken)
```
┌─────────────────────────────────────────────────────────┐
│              Traffic Flow Controller                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  VehiclePositionTracker (Node inside Node) ❌   │  │
│  │  - Subscriptions don't work properly             │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ALL vehicles use linear.y ❌                           │
│  - East/West vehicles move incorrectly                  │
└─────────────────────────────────────────────────────────┘
```

### Fixed System
```
┌─────────────────────────────────────────────────────────┐
│              Traffic Flow Controller                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Position Tracking (Helper Class) ✓              │  │
│  │  - Integrated directly into controller            │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  Velocity Axis Selection ✓                              │
│  - N/S vehicles: linear.y                               │
│  - E/W vehicles: linear.x                               │
└─────────────────────────────────────────────────────────┘
```

## Components and Interfaces

### Fix #1: Velocity Axis Mapping

**Current Code (BROKEN):**
```python
# ALL vehicles use linear.y
msg.linear.y = -velocity
pub.publish(msg)
```

**Fixed Code:**
```python
def get_velocity_axis_and_sign(self, vehicle_name):
    """
    Determine correct velocity axis and sign based on vehicle direction.
    
    Args:
        vehicle_name: Name like 'car_n1', 'car_s2', 'car_e3', 'car_w4'
    
    Returns:
        tuple: (axis, sign) where axis is 'x' or 'y' and sign is 1 or -1
    """
    direction = vehicle_name.split('_')[1][0]
    
    if direction == 'n':
        return ('y', 1)   # Northbound: positive Y
    elif direction == 's':
        return ('y', -1)  # Southbound: negative Y
    elif direction == 'e':
        return ('x', 1)   # Eastbound: positive X
    elif direction == 'w':
        return ('x', -1)  # Westbound: negative X
    else:
        self.get_logger().error(f"Unknown direction for {vehicle_name}")
        return ('y', 0)

def apply_velocity(self, vehicle_name, velocity):
    """Apply velocity to vehicle using correct axis."""
    axis, sign = self.get_velocity_axis_and_sign(vehicle_name)
    
    msg = Twist()
    if axis == 'x':
        msg.linear.x = sign * velocity
    else:  # axis == 'y'
        msg.linear.y = sign * velocity
    
    self.car_publishers[vehicle_name].publish(msg)
```

### Fix #2: Vehicle Position Tracking Refactor

**Option A: Integrate into Traffic Flow Controller (RECOMMENDED)**

```python
class MultiTrafficFlowControl(Node):
    def __init__(self):
        super().__init__('multi_traffic_flow_control')
        
        # Vehicle position tracking (integrated)
        self.vehicle_positions = {}  # {vehicle_name: (x, y, z)}
        self.vehicle_lanes = self._initialize_lane_mapping()
        
        # Subscribe to sensors for position tracking
        self.create_subscription(
            LogicalCameraImage, '/sensors/north',
            lambda msg: self.update_positions_from_sensor(msg), 10)
        # ... similar for other sensors
    
    def update_positions_from_sensor(self, msg):
        """Update vehicle positions from sensor data."""
        for model in msg.models:
            if model.name.startswith('car_'):
                pos = model.pose.position
                self.vehicle_positions[model.name] = (pos.x, pos.y, pos.z)
    
    def get_distance_to_vehicle_ahead(self, vehicle_name):
        """Calculate distance to vehicle ahead in same lane."""
        if vehicle_name not in self.vehicle_positions:
            return float('inf')
        
        my_pos = self.vehicle_positions[vehicle_name]
        my_lane = self.vehicle_lanes.get(vehicle_name)
        
        min_distance = float('inf')
        
        for other_name, other_pos in self.vehicle_positions.items():
            if other_name == vehicle_name:
                continue
            if self.vehicle_lanes.get(other_name) != my_lane:
                continue
            
            # Calculate distance based on lane direction
            distance = self._calculate_lane_distance(my_lane, my_pos, other_pos)
            if distance is not None and distance > 0:
                min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _calculate_lane_distance(self, lane, my_pos, other_pos):
        """Calculate distance to vehicle ahead in specific lane."""
        if lane == 'north':
            # Moving south (decreasing Y), vehicle ahead has lower Y
            if other_pos[1] < my_pos[1]:
                return my_pos[1] - other_pos[1]
        elif lane == 'south':
            # Moving north (increasing Y), vehicle ahead has higher Y
            if other_pos[1] > my_pos[1]:
                return other_pos[1] - my_pos[1]
        elif lane == 'east':
            # Moving west (decreasing X), vehicle ahead has lower X
            if other_pos[0] < my_pos[0]:
                return my_pos[0] - other_pos[0]
        elif lane == 'west':
            # Moving east (increasing X), vehicle ahead has higher X
            if other_pos[0] > my_pos[0]:
                return other_pos[0] - my_pos[0]
        return None
```

### Fix #3: Correct Vehicle Starting Positions

**World File Changes:**
```xml
<!-- WESTBOUND - FIX DUPLICATE POSITIONS -->
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
  <name>car_w1</name>
  <pose>30 2 0.01 0 0 -1.5707</pose>
</include>
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
  <name>car_w2</name>
  <pose>25 2 0.01 0 0 -1.5707</pose>  <!-- CHANGED from y=7 -->
</include>
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
  <name>car_w3</name>
  <pose>35 5 0.01 0 0 -1.5707</pose>  <!-- CHANGED from x=30, y=12 -->
</include>
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid</uri>
  <name>car_w4</name>
  <pose>30 5 0.01 0 0 -1.5707</pose>  <!-- CHANGED from x=25, y=7 -->
</include>
```

### Fix #4: Smooth Velocity Transitions

```python
class MultiTrafficFlowControl(Node):
    def __init__(self):
        # ... existing init ...
        
        # Velocity smoothing
        self.current_velocities = {}  # {vehicle_name: current_velocity}
        self.target_velocities = {}   # {vehicle_name: target_velocity}
        self.MAX_ACCELERATION = 3.0   # m/s²
        self.MAX_DECELERATION = 5.0   # m/s²
        self.CONTROL_RATE = 0.1       # seconds (10 Hz)
    
    def smooth_velocity_transition(self, vehicle_name, target_velocity):
        """
        Calculate smooth velocity transition.
        
        Args:
            vehicle_name: Name of vehicle
            target_velocity: Desired velocity
        
        Returns:
            float: Velocity to apply this timestep
        """
        current = self.current_velocities.get(vehicle_name, 0.0)
        
        # Calculate velocity change
        delta = target_velocity - current
        
        # Limit acceleration/deceleration
        if delta > 0:
            # Accelerating
            max_change = self.MAX_ACCELERATION * self.CONTROL_RATE
            delta = min(delta, max_change)
        else:
            # Decelerating
            max_change = self.MAX_DECELERATION * self.CONTROL_RATE
            delta = max(delta, -max_change)
        
        # Calculate new velocity
        new_velocity = current + delta
        
        # Update current velocity
        self.current_velocities[vehicle_name] = new_velocity
        
        return new_velocity
```

### Fix #5: Enhanced Red Light Enforcement

```python
def should_stop_for_red_light(self, vehicle_name):
    """
    Determine if vehicle should stop for red light.
    
    Returns:
        tuple: (should_stop, reason)
    """
    car_road = vehicle_name.split('_')[1][0]
    
    # Check if this direction has red light
    blocked_roads = self.get_blocked_roads()
    if car_road not in blocked_roads:
        return (False, "green_light")
    
    # Check if vehicle is in detection zone
    if vehicle_name not in self.in_zones[car_road]:
        return (False, "not_in_zone")
    
    # Check distance to stop line
    distance_to_stop = self.get_distance_to_stop_line(vehicle_name)
    if distance_to_stop < 0:
        # Already past stop line - let it clear intersection
        return (False, "past_stop_line")
    
    # Vehicle should stop
    return (True, "red_light_in_zone")

def get_distance_to_stop_line(self, vehicle_name):
    """
    Calculate distance to stop line.
    
    Returns:
        float: Distance in meters (negative if past stop line)
    """
    pos = self.vehicle_positions.get(vehicle_name)
    if pos is None:
        return float('inf')
    
    car_road = vehicle_name.split('_')[1][0]
    
    # Stop lines are at ±13m
    if car_road == 'n':
        return pos[1] - 13.0  # Moving toward +13
    elif car_road == 's':
        return -13.0 - pos[1]  # Moving toward -13
    elif car_road == 'e':
        return -13.0 - pos[0]  # Moving toward -13
    elif car_road == 'w':
        return pos[0] - 13.0  # Moving toward +13
    
    return float('inf')
```

### Fix #6: Sensor Direction Mapping Documentation

```python
class MultiTrafficFlowControl(Node):
    def __init__(self):
        super().__init__('multi_traffic_flow_control')
        
        # SENSOR TO TRAFFIC DIRECTION MAPPING
        # ====================================
        # Sensors are positioned to detect approaching traffic:
        # - sensor_north_road (Y=+20): Detects SOUTHBOUND traffic (cars moving south)
        # - sensor_south_road (Y=-20): Detects NORTHBOUND traffic (cars moving north)
        # - sensor_east_road (X=+20): Detects WESTBOUND traffic (cars moving west)
        # - sensor_west_road (X=-20): Detects EASTBOUND traffic (cars moving east)
        #
        # Therefore, sensor callbacks map to the OPPOSITE direction:
        
        self.create_subscription(
            LogicalCameraImage, '/sensors/north',
            lambda msg: self.sensor_cb(msg, 's'),  # North sensor → southbound traffic
            10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/south',
            lambda msg: self.sensor_cb(msg, 'n'),  # South sensor → northbound traffic
            10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/east',
            lambda msg: self.sensor_cb(msg, 'w'),  # East sensor → westbound traffic
            10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/west',
            lambda msg: self.sensor_cb(msg, 'e'),  # West sensor → eastbound traffic
            10)
```

## Data Models

### Vehicle Control State
```python
@dataclass
class VehicleControlState:
    name: str
    direction: str  # 'n', 's', 'e', 'w'
    position: Tuple[float, float, float]
    current_velocity: float
    target_velocity: float
    in_detection_zone: bool
    distance_to_stop_line: float
    distance_to_vehicle_ahead: float
    should_stop_for_red: bool
    should_stop_for_collision: bool
```

## Error Handling

### Velocity Axis Selection Errors
- If vehicle name doesn't match pattern, log error and default to zero velocity
- If direction character is invalid, log error and stop vehicle

### Position Tracking Errors
- If vehicle position is unknown, assume it's far from intersection
- If distance calculation fails, assume safe distance

### Sensor Data Errors
- If sensor data is malformed, skip that update cycle
- If no vehicles detected, clear the detection zone set

## Testing Strategy

### Unit Tests

1. **Test Velocity Axis Selection**
   - Verify northbound uses (y, +1)
   - Verify southbound uses (y, -1)
   - Verify eastbound uses (x, +1)
   - Verify westbound uses (x, -1)

2. **Test Distance Calculations**
   - Test distance to vehicle ahead in each lane
   - Test distance to stop line from various positions
   - Test edge cases (at stop line, past stop line)

3. **Test Red Light Logic**
   - Verify vehicles stop when in zone with red light
   - Verify vehicles don't stop when not in zone
   - Verify vehicles don't stop with green light

### Integration Tests

1. **Complete Traffic Cycle**
   - Start with NS_GREEN
   - Verify NS vehicles move, EW vehicles stop
   - Transition to EW_GREEN
   - Verify EW vehicles move, NS vehicles stop

2. **Collision Avoidance**
   - Place two vehicles in same lane
   - Verify following vehicle maintains safe distance
   - Verify vehicles queue properly at red light

3. **Intersection Clearance**
   - Verify vehicles clear intersection before phase change
   - Verify no vehicles enter during ALL_RED

### Simulation Tests

1. **10-Minute Crash-Free Test**
   - Run simulation for 10 minutes
   - Monitor for any collisions
   - Verify zero crashes occur

2. **Red Light Compliance Test**
   - Monitor all vehicles for 5 minutes
   - Verify 100% red light compliance
   - Log any violations

3. **Queue Formation Test**
   - Observe vehicles queuing at red lights
   - Verify proper spacing
   - Verify orderly release on green

## Implementation Priority

### Phase 1: Critical Fixes (Must Do First)
1. Fix velocity axis mapping
2. Integrate position tracking into traffic_flow.py
3. Fix duplicate vehicle positions
4. Test basic vehicle movement

### Phase 2: Safety Enhancements
5. Add smooth velocity transitions
6. Strengthen red light enforcement
7. Add stop line distance checking
8. Test red light compliance

### Phase 3: Polish and Validation
9. Add comprehensive logging
10. Run full simulation tests
11. Verify zero crashes
12. Document system behavior
