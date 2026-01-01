# Design Document: Adaptive Traffic System V2

## Overview

This design completes the Smart Adaptive Traffic Light System by addressing intersection collision issues, adding pedestrian simulation with detection-based crossing, and implementing traffic flow metrics. The system will demonstrate intelligent traffic control that responds to real-time vehicle and pedestrian conditions.

The key architectural additions include:
1. **Intersection Zone Monitor** - Tracks vehicles in the intersection to prevent collisions
2. **Pedestrian Simulation** - Animated pedestrian actors using gazebo-ros-actor-plugin
3. **Pedestrian Detection** - Sensors at crosswalks to detect waiting pedestrians
4. **Enhanced State Machine** - Supports vehicle and pedestrian phases with yellow transitions
5. **Metrics Collector** - Tracks performance for adaptive vs fixed-time comparison

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulation                                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │
│  │ Traffic      │  │  Logical     │  │   Vehicles   │  │ Pedestrian  │ │
│  │ Lights       │  │  Cameras     │  │  (16 Prius)  │  │ Actors      │ │
│  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
           │                  │                  │              │
           │ Light Commands   │ Sensor Data      │ Vel Cmds     │ Actor Cmds
           │                  │                  │              │
┌──────────┴──────────────────┴──────────────────┴──────────────┴─────────┐
│                         ROS-Gazebo Bridge                                │
└──────────┬──────────────────┬──────────────────┬──────────────┬─────────┘
           │                  │                  │              │
    ┌──────▼──────┐    ┌─────▼──────┐    ┌─────▼──────┐  ┌────▼────────┐
    │   Traffic   │    │  Vehicle   │    │Intersection│  │ Pedestrian  │
    │   Manager   │◄───│   Flow     │◄───│   Zone     │  │ Controller  │
    │  (Enhanced) │    │ Controller │    │  Monitor   │  │             │
    └──────┬──────┘    └────────────┘    └────────────┘  └─────────────┘
           │                                                    │
    ┌──────▼──────┐                                      ┌─────▼──────┐
    │  Metrics    │                                      │ Pedestrian │
    │  Collector  │                                      │  Detector  │
    └─────────────┘                                      └────────────┘
```

### Component Responsibilities

1. **Traffic_Manager (Enhanced)**
   - Manages extended state machine with pedestrian phases
   - Coordinates vehicle and pedestrian signals
   - Implements adaptive and fixed-time modes
   - Publishes traffic light and pedestrian signal commands

2. **Intersection_Zone_Monitor (New)**
   - Subscribes to intersection zone sensor
   - Tracks vehicles currently in the intersection
   - Provides clearance status to Traffic_Manager
   - Prevents phase changes when intersection not clear

3. **Pedestrian_Controller (New)**
   - Subscribes to pedestrian signal state
   - Commands pedestrian actors to walk or wait
   - Uses gazebo-ros-actor-plugin velocity interface
   - Manages pedestrian group behavior

4. **Pedestrian_Detector (New)**
   - Subscribes to pedestrian sensors at crosswalks
   - Counts waiting pedestrians per crosswalk
   - Publishes pedestrian demand to Traffic_Manager

5. **Metrics_Collector (New)**
   - Subscribes to vehicle positions, light states, pedestrian crossings
   - Calculates wait times, throughput, queue lengths
   - Publishes metrics for logging and comparison

6. **Flow_Controller (Enhanced)**
   - Added yellow light response (slow down)
   - Integration with Intersection_Zone_Monitor
   - Improved collision avoidance at intersection

## Components and Interfaces

### Enhanced Traffic Light State Machine

```python
class TrafficState(Enum):
    NS_GREEN = "NS_GREEN"           # North-South vehicles green
    NS_YELLOW = "NS_YELLOW"         # North-South transitioning to red
    EW_GREEN = "EW_GREEN"           # East-West vehicles green
    EW_YELLOW = "EW_YELLOW"         # East-West transitioning to red
    NS_PED_CROSSING = "NS_PED"      # North-South crosswalks active
    EW_PED_CROSSING = "EW_PED"      # East-West crosswalks active
    ALL_RED = "ALL_RED"             # Clearance interval

# State transitions
VALID_TRANSITIONS = {
    TrafficState.NS_GREEN: [TrafficState.NS_YELLOW],
    TrafficState.NS_YELLOW: [TrafficState.ALL_RED],
    TrafficState.EW_GREEN: [TrafficState.EW_YELLOW],
    TrafficState.EW_YELLOW: [TrafficState.ALL_RED],
    TrafficState.NS_PED_CROSSING: [TrafficState.ALL_RED],
    TrafficState.EW_PED_CROSSING: [TrafficState.ALL_RED],
    TrafficState.ALL_RED: [
        TrafficState.NS_GREEN, 
        TrafficState.EW_GREEN,
        TrafficState.NS_PED_CROSSING,
        TrafficState.EW_PED_CROSSING
    ]
}
```

### Intersection Zone Monitor

```python
class IntersectionZoneMonitor(Node):
    """
    Monitors vehicles in the intersection zone to ensure safe phase changes.
    """
    
    ZONE_BOUNDS = {
        'x_min': -13.0, 'x_max': 13.0,
        'y_min': -13.0, 'y_max': 13.0
    }
    
    def __init__(self):
        super().__init__('intersection_zone_monitor')
        
        self.vehicles_in_zone = {}  # {vehicle_name: entry_time}
        self.STUCK_THRESHOLD = 10.0  # seconds
        
        # Subscribe to intersection zone sensor
        self.create_subscription(
            LogicalCameraImage,
            '/sensors/intersection_zone',
            self.zone_sensor_callback,
            10
        )
        
        # Publisher for zone status
        self.zone_status_pub = self.create_publisher(
            IntersectionZoneStatus,  # Custom message
            '/intersection/zone_status',
            10
        )
    
    def zone_sensor_callback(self, msg):
        """Process intersection zone sensor data"""
        current_vehicles = set()
        now = self.get_clock().now()
        
        for model in msg.models:
            if model.name.startswith('prius_'):
                current_vehicles.add(model.name)
                
                # Track entry time for new vehicles
                if model.name not in self.vehicles_in_zone:
                    self.vehicles_in_zone[model.name] = now
                else:
                    # Check for stuck vehicles
                    entry_time = self.vehicles_in_zone[model.name]
                    if (now - entry_time).nanoseconds / 1e9 > self.STUCK_THRESHOLD:
                        self.get_logger().warning(
                            f"Vehicle {model.name} stuck in intersection for "
                            f"{self.STUCK_THRESHOLD}+ seconds"
                        )
        
        # Remove vehicles that have exited
        exited = set(self.vehicles_in_zone.keys()) - current_vehicles
        for vehicle in exited:
            del self.vehicles_in_zone[vehicle]
        
        # Publish zone status
        self.publish_zone_status()
    
    def is_zone_clear(self, conflicting_directions=None):
        """
        Check if intersection zone is clear for phase change.
        
        Args:
            conflicting_directions: List of directions that would conflict
                                   with the new phase (e.g., ['n', 's'])
        
        Returns:
            bool: True if zone is clear for the specified directions
        """
        if conflicting_directions is None:
            return len(self.vehicles_in_zone) == 0
        
        for vehicle in self.vehicles_in_zone:
            direction = vehicle.split('_')[1][0]  # e.g., 'n' from 'prius_n1'
            if direction in conflicting_directions:
                return False
        return True
```

### Pedestrian Controller

```python
class PedestrianController(Node):
    """
    Controls pedestrian actor movements based on crossing signals.
    Uses gazebo-ros-actor-plugin velocity interface.
    """
    
    WALK_SPEED_MIN = 1.0  # m/s
    WALK_SPEED_MAX = 1.5  # m/s
    
    def __init__(self):
        super().__init__('pedestrian_controller')
        
        # Pedestrian configurations
        self.pedestrians = {}  # {name: PedestrianState}
        self.crosswalk_assignments = {}  # {name: crosswalk_id}
        
        # Initialize pedestrians (4-8 per crosswalk)
        self._initialize_pedestrians()
        
        # Subscribe to pedestrian signal state
        self.create_subscription(
            PedestrianSignalState,
            '/pedestrian_signals/state',
            self.signal_callback,
            10
        )
        
        # Publishers for each pedestrian actor
        self.pedestrian_pubs = {}
        for name in self.pedestrians:
            topic = f'/model/{name}/cmd_vel'
            self.pedestrian_pubs[name] = self.create_publisher(Twist, topic, 10)
        
        # Control timer
        self.create_timer(0.1, self.control_loop)
    
    def _initialize_pedestrians(self):
        """Initialize pedestrian configurations with varied speeds"""
        crosswalks = ['north', 'south', 'east', 'west']
        
        for cw in crosswalks:
            for i in range(1, 7):  # 6 pedestrians per crosswalk
                name = f'pedestrian_{cw}_{i}'
                # Assign varied walking speed
                speed = random.uniform(self.WALK_SPEED_MIN, self.WALK_SPEED_MAX)
                self.pedestrians[name] = PedestrianState(
                    name=name,
                    crosswalk=cw,
                    assigned_speed=speed,
                    is_crossing=False,
                    has_crossed=False
                )
                self.crosswalk_assignments[name] = cw
    
    def signal_callback(self, msg):
        """Handle pedestrian signal state changes"""
        # msg contains which crosswalks have walk signal
        for crosswalk, can_walk in msg.crosswalk_states.items():
            for name, state in self.pedestrians.items():
                if state.crosswalk == crosswalk:
                    if can_walk and not state.has_crossed:
                        state.is_crossing = True
                    elif not can_walk:
                        state.is_crossing = False
    
    def control_loop(self):
        """Publish velocity commands to pedestrian actors"""
        for name, state in self.pedestrians.items():
            msg = Twist()
            
            if state.is_crossing:
                # Walk across crosswalk
                velocity = state.assigned_speed
                
                # Direction depends on crosswalk
                if state.crosswalk in ['north', 'south']:
                    msg.linear.x = velocity if state.crosswalk == 'north' else -velocity
                else:  # east, west
                    msg.linear.y = velocity if state.crosswalk == 'east' else -velocity
            else:
                # Stay stationary
                msg.linear.x = 0.0
                msg.linear.y = 0.0
            
            self.pedestrian_pubs[name].publish(msg)
```

### Pedestrian Detector

```python
class PedestrianDetector(Node):
    """
    Detects pedestrians waiting at crosswalks using logical camera sensors.
    """
    
    def __init__(self):
        super().__init__('pedestrian_detector')
        
        self.waiting_counts = {
            'north': 0, 'south': 0, 'east': 0, 'west': 0
        }
        
        # Subscribe to pedestrian sensors at each crosswalk
        for direction in ['north', 'south', 'east', 'west']:
            self.create_subscription(
                LogicalCameraImage,
                f'/sensors/pedestrian_{direction}',
                lambda msg, d=direction: self.sensor_callback(msg, d),
                10
            )
        
        # Publisher for pedestrian demand
        self.demand_pub = self.create_publisher(
            PedestrianDemand,
            '/pedestrian/demand',
            10
        )
    
    def sensor_callback(self, msg, direction):
        """Process pedestrian sensor data"""
        count = 0
        for model in msg.models:
            if model.name.startswith('pedestrian_'):
                count += 1
        
        self.waiting_counts[direction] = count
        self.publish_demand()
    
    def publish_demand(self):
        """Publish current pedestrian demand"""
        msg = PedestrianDemand()
        msg.north_waiting = self.waiting_counts['north']
        msg.south_waiting = self.waiting_counts['south']
        msg.east_waiting = self.waiting_counts['east']
        msg.west_waiting = self.waiting_counts['west']
        msg.total_waiting = sum(self.waiting_counts.values())
        self.demand_pub.publish(msg)
```

### Metrics Collector

```python
class MetricsCollector(Node):
    """
    Collects and publishes traffic flow metrics for performance comparison.
    """
    
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Vehicle metrics
        self.vehicle_wait_times = {'n': [], 's': [], 'e': [], 'w': []}
        self.vehicle_entry_times = {}  # {vehicle_name: entry_time}
        self.vehicles_passed = {'n': 0, 's': 0, 'e': 0, 'w': 0}
        self.queue_lengths = {'n': [], 's': [], 'e': [], 'w': []}
        
        # Pedestrian metrics
        self.pedestrian_wait_times = []
        self.pedestrian_crossings = 0
        
        # Timing
        self.start_time = self.get_clock().now()
        
        # Subscriptions
        self.create_subscription(String, '/traffic_lights/state', self.state_cb, 10)
        # ... additional subscriptions for vehicle positions, pedestrian crossings
        
        # Publisher
        self.metrics_pub = self.create_publisher(TrafficMetrics, '/metrics', 10)
        
        # Periodic metrics calculation
        self.create_timer(5.0, self.calculate_and_publish_metrics)
    
    def calculate_and_publish_metrics(self):
        """Calculate and publish current metrics"""
        elapsed_minutes = (self.get_clock().now() - self.start_time).nanoseconds / 1e9 / 60
        
        msg = TrafficMetrics()
        
        # Average wait times per direction
        for direction in ['n', 's', 'e', 'w']:
            if self.vehicle_wait_times[direction]:
                msg.avg_wait_time[direction] = sum(self.vehicle_wait_times[direction]) / len(self.vehicle_wait_times[direction])
            else:
                msg.avg_wait_time[direction] = 0.0
        
        # Throughput (vehicles per minute)
        total_passed = sum(self.vehicles_passed.values())
        msg.throughput = total_passed / elapsed_minutes if elapsed_minutes > 0 else 0.0
        
        # Max queue lengths
        for direction in ['n', 's', 'e', 'w']:
            msg.max_queue_length[direction] = max(self.queue_lengths[direction]) if self.queue_lengths[direction] else 0
        
        # Pedestrian metrics
        msg.avg_pedestrian_wait = sum(self.pedestrian_wait_times) / len(self.pedestrian_wait_times) if self.pedestrian_wait_times else 0.0
        msg.total_pedestrian_crossings = self.pedestrian_crossings
        
        self.metrics_pub.publish(msg)
```

### Enhanced Flow Controller

```python
class EnhancedFlowController(Node):
    """
    Enhanced vehicle flow controller with yellow light response
    and intersection zone awareness.
    """
    
    def __init__(self):
        super().__init__('enhanced_flow_controller')
        
        # ... existing initialization ...
        
        # Yellow light handling
        self.YELLOW_SLOWDOWN_FACTOR = 0.5  # Reduce speed by 50% on yellow
        
        # Subscribe to zone status
        self.create_subscription(
            IntersectionZoneStatus,
            '/intersection/zone_status',
            self.zone_status_cb,
            10
        )
        
        self.zone_clear = True
    
    def timer_callback(self):
        """Enhanced control with yellow light and zone awareness"""
        for name, pub in self.car_publishers.items():
            msg = Twist()
            car_road = name.split('_')[1][0]
            
            # Determine if vehicle should stop or slow
            velocity = self.speeds[name]
            
            # Check traffic light state
            if self._should_stop_for_light(car_road):
                velocity = 0.0
            elif self._should_slow_for_yellow(car_road):
                velocity *= self.YELLOW_SLOWDOWN_FACTOR
            
            # Check collision avoidance
            distance_ahead = self.position_tracker.get_distance_to_vehicle_ahead(name)
            velocity = self._apply_collision_avoidance(velocity, distance_ahead)
            
            # Check intersection zone safety
            if self._is_approaching_intersection(name) and not self.zone_clear:
                velocity = min(velocity, self.speeds[name] * 0.3)
            
            # Publish velocity
            msg.linear.y = -velocity
            pub.publish(msg)
    
    def _should_stop_for_light(self, car_road):
        """Check if vehicle should stop for red light"""
        if self.light_state == "ALL_RED":
            return True
        if self.light_state in ["NS_GREEN", "NS_YELLOW"] and car_road in ['e', 'w']:
            return True
        if self.light_state in ["EW_GREEN", "EW_YELLOW"] and car_road in ['n', 's']:
            return True
        if self.light_state in ["NS_PED", "EW_PED"]:
            return True  # All vehicles stop during pedestrian crossing
        return False
    
    def _should_slow_for_yellow(self, car_road):
        """Check if vehicle should slow for yellow light"""
        if self.light_state == "NS_YELLOW" and car_road in ['n', 's']:
            return True
        if self.light_state == "EW_YELLOW" and car_road in ['e', 'w']:
            return True
        return False
```

## Data Models

### Pedestrian State
```python
@dataclass
class PedestrianState:
    name: str
    crosswalk: str  # 'north', 'south', 'east', 'west'
    assigned_speed: float
    is_crossing: bool
    has_crossed: bool
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
```

### Traffic Metrics
```python
@dataclass
class TrafficMetrics:
    avg_wait_time: Dict[str, float]  # per direction
    throughput: float  # vehicles per minute
    max_queue_length: Dict[str, int]  # per direction
    avg_pedestrian_wait: float
    total_pedestrian_crossings: int
    mode: str  # 'adaptive' or 'fixed'
    timestamp: float
```

### Intersection Zone Status
```python
@dataclass
class IntersectionZoneStatus:
    is_clear: bool
    vehicles_in_zone: List[str]
    ns_clear: bool  # Clear for NS traffic
    ew_clear: bool  # Clear for EW traffic
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: Conflicting Traffic Exclusion

*For any* traffic light state, vehicles from conflicting directions SHALL NOT both have permission to enter the Intersection_Zone simultaneously. Specifically: when NS_GREEN or NS_YELLOW, EW vehicles must be stopped; when EW_GREEN or EW_YELLOW, NS vehicles must be stopped; when ALL_RED or any PED_CROSSING state, all vehicles must be stopped.

**Validates: Requirements 1.1, 1.2, 1.3**

### Property 2: Intersection Clearance Before Phase Change

*For any* state transition from ALL_RED to a green phase, the Intersection_Zone SHALL be clear of vehicles from the conflicting direction before the transition occurs.

**Validates: Requirements 1.4, 1.6, 10.3**

### Property 3: Minimum Clearance Time

*For any* transition between conflicting green phases (NS_GREEN to EW_GREEN or vice versa), the elapsed time through intermediate states (YELLOW + ALL_RED) SHALL be at least the configured minimum clearance time.

**Validates: Requirements 1.5**

### Property 4: Pedestrian Crossing Speed Bounds

*For any* pedestrian actor that is crossing, the commanded velocity magnitude SHALL be between WALK_SPEED_MIN (1.0 m/s) and WALK_SPEED_MAX (1.5 m/s).

**Validates: Requirements 2.5**

### Property 5: Pedestrian Speed Variation

*For any* set of pedestrian actors crossing simultaneously, there SHALL exist at least two pedestrians with different assigned speeds (not all identical).

**Validates: Requirements 2.6**

### Property 6: Pedestrian Count Accuracy

*For any* pedestrian sensor data message, the reported waiting count SHALL equal the number of pedestrian models detected in the sensor's field of view.

**Validates: Requirements 3.3, 3.4**

### Property 7: Pedestrian Crossing Initiation Time Bound

*For any* crosswalk with waiting pedestrians, a pedestrian crossing phase SHALL be initiated within 60 seconds of the first pedestrian being detected.

**Validates: Requirements 4.1**

### Property 8: Pedestrian Phase Traffic Stop

*For any* pedestrian crossing state (NS_PED_CROSSING or EW_PED_CROSSING), all vehicle traffic lights SHALL be red and all vehicles SHALL be commanded to stop.

**Validates: Requirements 4.2, 6.2, 6.3**

### Property 9: Clearance Interval Before Walk Signal

*For any* transition to a pedestrian crossing phase, the walk signal SHALL NOT be activated until after the clearance interval has elapsed following the vehicle red light.

**Validates: Requirements 4.3**

### Property 10: Walk Signal Triggers Pedestrian Movement

*For any* crosswalk showing a walk signal, all waiting pedestrians at that crosswalk SHALL be commanded with non-zero velocity toward the opposite side.

**Validates: Requirements 4.4, 5.1**

### Property 11: Crossing Phase Duration

*For any* pedestrian crossing phase, the duration SHALL be at least the configured Safe_Crossing_Time unless extended due to occupied crosswalk.

**Validates: Requirements 4.5**

### Property 12: Yellow-to-Green Transition After Crossing

*For any* transition from a pedestrian crossing phase to a vehicle green phase, the system SHALL pass through ALL_RED state and the appropriate YELLOW state.

**Validates: Requirements 4.6, 6.5**

### Property 13: Red Light Extension for Occupied Crosswalk

*For any* pedestrian crossing phase where pedestrians are still in the crosswalk when Safe_Crossing_Time expires, the vehicle red light SHALL remain active until the crosswalk is clear.

**Validates: Requirements 4.7**

### Property 14: Pedestrian Stop at Destination

*For any* pedestrian that has crossed to the opposite side of the crosswalk, the commanded velocity SHALL be zero.

**Validates: Requirements 5.3**

### Property 15: Don't Walk Keeps Pedestrians Stationary

*For any* crosswalk showing a don't-walk signal, all pedestrians in the waiting area of that crosswalk SHALL be commanded with zero velocity.

**Validates: Requirements 5.4**

### Property 16: Pedestrian Spacing

*For any* pair of pedestrians crossing simultaneously, the distance between them SHALL be at least 0.5 meters (no overlapping).

**Validates: Requirements 5.6**

### Property 17: No Conflicting Pedestrian-Vehicle Movements

*For any* traffic state, there SHALL NOT exist both a walk signal for a crosswalk AND a green light for vehicles that would cross that crosswalk.

**Validates: Requirements 6.6**

### Property 18: Yellow Light Duration

*For any* yellow light phase, the duration SHALL be within the configured range (default 3 seconds ± 0.5 seconds).

**Validates: Requirements 7.2, 7.5**

### Property 19: Vehicle Slowdown on Yellow

*For any* vehicle approaching a yellow light in its direction, the commanded velocity SHALL be reduced compared to normal speed (at most YELLOW_SLOWDOWN_FACTOR × assigned_speed).

**Validates: Requirements 7.3, 7.4**

### Property 20: Metrics Calculation Correctness

*For any* metrics report, the average wait time SHALL equal the sum of individual wait times divided by count, throughput SHALL equal vehicles passed divided by elapsed minutes, and queue length SHALL be non-negative.

**Validates: Requirements 8.1, 8.2, 8.3, 8.4, 8.5**

### Property 21: Fixed-Time Mode Constant Intervals

*For any* simulation running in fixed-time mode, the duration of each phase SHALL be constant (within 0.5 second tolerance) regardless of traffic conditions.

**Validates: Requirements 9.2, 9.3**

### Property 22: Intersection Zone Vehicle Tracking

*For any* vehicle that enters the Intersection_Zone, it SHALL remain tracked in the zone monitor until it exits the zone bounds.

**Validates: Requirements 10.2**

### Property 23: Pedestrian Signal State Consistency

*For any* pedestrian signal, the displayed state (walk/don't-walk) SHALL match the current Traffic_Manager state for that crosswalk.

**Validates: Requirements 11.2, 11.3, 11.4**

## Error Handling

### Intersection Zone Errors

1. **Stuck Vehicle Detection**: If a vehicle remains in the intersection zone for more than 10 seconds, log a warning and consider it for emergency clearance
2. **Sensor Failure**: If intersection zone sensor fails, default to time-based clearance (conservative approach)
3. **Multiple Vehicles Stuck**: If multiple vehicles are stuck, trigger emergency all-red and log critical error

### Pedestrian System Errors

1. **Pedestrian Sensor Failure**: If a pedestrian sensor fails, assume pedestrians are waiting (safe default)
2. **Actor Command Failure**: If pedestrian velocity command fails, retry on next control cycle
3. **Crossing Timeout**: If pedestrians don't clear crosswalk within 2× Safe_Crossing_Time, log warning and force phase change

### Metrics Errors

1. **Division by Zero**: Handle zero elapsed time or zero counts gracefully
2. **Missing Data**: Use last known values if current data is unavailable
3. **Overflow**: Cap metrics at reasonable maximum values

## Testing Strategy

### Dual Testing Approach

This system will be validated using both unit tests and property-based tests:

- **Unit tests**: Verify specific examples, edge cases, and error conditions
- **Property tests**: Verify universal properties across all inputs using Hypothesis

### Property-Based Testing Configuration

```python
from hypothesis import given, settings
import hypothesis.strategies as st

@settings(max_examples=100)
@given(
    traffic_state=st.sampled_from([
        'NS_GREEN', 'NS_YELLOW', 'EW_GREEN', 'EW_YELLOW',
        'NS_PED', 'EW_PED', 'ALL_RED'
    ]),
    vehicle_positions=st.lists(
        st.tuples(st.floats(-50, 50), st.floats(-50, 50)),
        min_size=0, max_size=16
    ),
    pedestrian_waiting=st.dictionaries(
        st.sampled_from(['north', 'south', 'east', 'west']),
        st.integers(0, 8)
    )
)
def test_property_X(traffic_state, vehicle_positions, pedestrian_waiting):
    """Feature: adaptive-traffic-system-v2, Property X: [description]"""
    pass
```

### Test Organization

```
tests/
├── test_intersection_zone_properties.py
├── test_pedestrian_control_properties.py
├── test_signal_timing_properties.py
├── test_metrics_properties.py
├── test_integration.py
└── test_fixed_vs_adaptive.py
```

### Testing Priorities

**Critical (Must pass)**:
- Property 1: Conflicting Traffic Exclusion
- Property 2: Intersection Clearance Before Phase Change
- Property 8: Pedestrian Phase Traffic Stop
- Property 17: No Conflicting Pedestrian-Vehicle Movements

**High Priority**:
- Property 3: Minimum Clearance Time
- Property 10: Walk Signal Triggers Pedestrian Movement
- Property 13: Red Light Extension for Occupied Crosswalk
- Property 19: Vehicle Slowdown on Yellow

**Medium Priority**:
- Property 4-6: Pedestrian behavior properties
- Property 18: Yellow Light Duration
- Property 20-21: Metrics and mode properties

**Lower Priority**:
- Property 22-23: Tracking and signal consistency
