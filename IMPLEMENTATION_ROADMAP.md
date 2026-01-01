# Implementation Roadmap - Smart Traffic Light System

## Based on traffic-flow-logic.txt Specifications

---

## ✅ COMPLETED

### Phase 0: Vehicle System
- ✅ 16 vehicles at intersection (4 per direction)
- ✅ Vehicle movement control working
- ✅ Traffic light models in place
- ✅ Vehicle sensors configured

### Phase 1: Pedestrian Models
- ✅ 8 pedestrian actors added (2 per crosswalk)
- ✅ Pedestrian models positioned at waiting areas
- ✅ Walking speed set to 1.2 m/s (standard)
- ✅ Path-following plugin configured

---

## 🔄 IN PROGRESS / NEXT STEPS

### Phase 2: Pedestrian Detection Sensors

**Goal:** Add sensors to detect waiting pedestrians

**Implementation:**
1. Add logical camera sensors at each crosswalk waiting area
2. Position sensors to cover curb area (NOT road)
3. Configure sensor topics:
   ```
   /traffic/sensor/pedestrian/north
   /traffic/sensor/pedestrian/south
   /traffic/sensor/pedestrian/east
   /traffic/sensor/pedestrian/west
   ```
4. Output: Number of pedestrians waiting (Int32)

**Files to modify:**
- `intersection.world` - Add pedestrian sensors

---

### Phase 3: Central Traffic Controller Node

**Goal:** Implement the main traffic control logic

**File:** `ros2_ws/src/smart_traffic_system/scripts/traffic_controller_node.py`

**Responsibilities:**
1. Subscribe to all sensor topics (vehicles + pedestrians)
2. Calculate demand per phase
3. Implement state machine
4. Compute adaptive green times
5. Publish traffic light commands

**State Machine:**
```
STATE_NS_GREEN
  ↓ (timer expires)
STATE_NS_YELLOW (3 seconds)
  ↓
STATE_ALL_RED_1 (1-2 seconds)
  ↓
STATE_EW_GREEN
  ↓ (timer expires)
STATE_EW_YELLOW (3 seconds)
  ↓
STATE_ALL_RED_2 (1-2 seconds)
  ↓
[back to STATE_NS_GREEN]
```

**Demand Calculation:**
```python
# Aggregate demand per phase
NS_vehicle_demand = north_vehicles + south_vehicles
EW_vehicle_demand = east_vehicles + west_vehicles

NS_pedestrian_demand = north_peds + south_peds
EW_pedestrian_demand = east_peds + west_peds

# Weighted total demand (α = 2-5 for pedestrian priority)
NS_total_demand = NS_vehicle_demand + α * NS_pedestrian_demand
EW_total_demand = EW_vehicle_demand + α * EW_pedestrian_demand
```

**Green Time Calculation:**
```python
BASE_GREEN = 10  # seconds
MAX_GREEN = 60   # seconds
k_vehicle = 1.0  # seconds per vehicle
k_ped = 2.0      # seconds per pedestrian

green_time = BASE_GREEN + k_vehicle * vehicle_demand + k_ped * pedestrian_demand
green_time = clamp(green_time, BASE_GREEN, MAX_GREEN)
```

**Subscribed Topics:**
```
/traffic/sensor/vehicle/north
/traffic/sensor/vehicle/south
/traffic/sensor/vehicle/east
/traffic/sensor/vehicle/west
/traffic/sensor/pedestrian/north
/traffic/sensor/pedestrian/south
/traffic/sensor/pedestrian/east
/traffic/sensor/pedestrian/west
```

**Published Topics:**
```
/traffic_lights/state (String: "NS_GREEN", "EW_GREEN", etc.)
/traffic/light/north (String: "RED", "YELLOW", "GREEN")
/traffic/light/south
/traffic/light/east
/traffic/light/west
/pedestrian/signal/north (String: "WALK", "DONT_WALK")
/pedestrian/signal/south
/pedestrian/signal/east
/pedestrian/signal/west
```

---

### Phase 4: Pedestrian Controller Node

**Goal:** Control pedestrian crossing behavior

**File:** `ros2_ws/src/smart_traffic_system/scripts/pedestrian_controller.py`

**Responsibilities:**
1. Subscribe to pedestrian signal states
2. Publish crossing paths when WALK signal is active
3. Calculate crossing time based on crosswalk length
4. Ensure pedestrians complete crossing safely

**Crossing Logic:**
```python
# When pedestrian signal is WALK
if signal == "WALK":
    # Publish crossing path
    path = generate_crossing_path(crosswalk)
    publish_path(pedestrian_id, path)
    
# When signal is DONT_WALK
else:
    # Keep pedestrian at waiting position
    publish_wait_position(pedestrian_id)
```

**Crossing Time Calculation:**
```python
crosswalk_length = 12  # meters (road width)
walking_speed = 1.2    # m/s (standard)
MIN_WALK_TIME = 7      # seconds (minimum)

crossing_time = crosswalk_length / walking_speed
ped_green_time = max(crossing_time, MIN_WALK_TIME)
```

**Crossing Paths:**
- North crosswalk: East-West crossing
- South crosswalk: East-West crossing
- East crosswalk: North-South crossing
- West crosswalk: North-South crossing

**Subscribed Topics:**
```
/pedestrian/signal/north
/pedestrian/signal/south
/pedestrian/signal/east
/pedestrian/signal/west
```

**Published Topics:**
```
/pedestrian_north_1/cmd_path
/pedestrian_north_2/cmd_path
/pedestrian_south_1/cmd_path
/pedestrian_south_2/cmd_path
/pedestrian_east_1/cmd_path
/pedestrian_east_2/cmd_path
/pedestrian_west_1/cmd_path
/pedestrian_west_2/cmd_path
```

---

### Phase 5: Vehicle-Pedestrian Interaction

**Goal:** Make vehicles stop for crossing pedestrians

**File:** `ros2_ws/src/smart_traffic_system/scripts/traffic_flow.py` (modify existing)

**Modifications:**
1. Subscribe to pedestrian positions
2. Detect pedestrians in crosswalk zones
3. Stop vehicles when pedestrians are crossing
4. Resume vehicle movement when crosswalk is clear

**Crosswalk Zones:**
- North crosswalk: Y = 10 to 13
- South crosswalk: Y = -13 to -10
- East crosswalk: X = 10 to 13
- West crosswalk: X = -13 to -10

**Logic:**
```python
# Check if pedestrian is in vehicle's crosswalk
if vehicle_approaching_crosswalk(vehicle):
    if pedestrian_in_crosswalk(crosswalk):
        # Stop vehicle
        velocity = 0.0
    else:
        # Normal traffic light logic
        velocity = calculate_velocity(vehicle)
```

---

### Phase 6: Integration and Testing

**Goal:** Verify complete system operation

**Tests:**
1. **Vehicle-only test:** Vehicles follow traffic lights
2. **Pedestrian-only test:** Pedestrians cross on WALK signal
3. **Mixed traffic test:** Vehicles and pedestrians coordinate
4. **High demand test:** System adapts to heavy traffic
5. **Safety test:** No conflicts between vehicles and pedestrians

**Validation Checklist:**
- [ ] Vehicles stop at red lights
- [ ] Vehicles proceed on green lights
- [ ] Pedestrians cross only on WALK signal
- [ ] Pedestrians never cross with conflicting vehicle green
- [ ] ALL_RED phase occurs between phase transitions
- [ ] Yellow light precedes red
- [ ] Green time adapts to demand
- [ ] No collisions between vehicles
- [ ] No collisions between vehicles and pedestrians

---

## Safety Constraints (MUST FOLLOW)

1. ❌ **No conflicting green signals**
   - NS and EW vehicles never both green
   
2. ❌ **No pedestrian WALK during conflicting vehicle GREEN**
   - NS pedestrians don't walk during NS_GREEN
   - EW pedestrians don't walk during EW_GREEN
   
3. ✅ **Always include ALL-RED between phases**
   - 1-2 seconds of all red between phase changes
   
4. ✅ **Yellow must precede red**
   - 3 seconds of yellow before red
   
5. ✅ **Intersection must be empty before next green**
   - Verify no vehicles/pedestrians in intersection

---

## Implementation Priority

### High Priority (Do First)
1. ✅ Add pedestrian models (DONE)
2. 🔄 Add pedestrian sensors
3. 🔄 Create traffic_controller_node.py
4. 🔄 Create pedestrian_controller.py

### Medium Priority (Do Second)
5. 🔄 Integrate vehicle-pedestrian interaction
6. 🔄 Test and validate system

### Low Priority (Optional Enhancements)
7. ⏳ Add more pedestrian models (variety)
8. ⏳ Add pedestrian signal visualizations
9. ⏳ Add RViz visualization
10. ⏳ Add data logging and analysis

---

## File Structure

```
ros2_ws/src/smart_traffic_system/
├── scripts/
│   ├── traffic_flow.py (existing - modify)
│   ├── smart_traffic_manager.py (existing - may replace)
│   ├── traffic_controller_node.py (NEW - main controller)
│   └── pedestrian_controller.py (NEW - pedestrian control)
├── worlds/
│   └── intersection.world (modified - pedestrians added ✅)
├── launch/
│   └── simulation.launch.py (modify - launch new nodes)
└── tests/
    └── test_traffic_controller.py (NEW - validation tests)
```

---

## Current Status

✅ **Phase 1 Complete:** Pedestrians added to simulation
🔄 **Phase 2 Next:** Add pedestrian sensors
🔄 **Phase 3 Next:** Implement traffic controller node

---

**Last Updated:** December 30, 2025
**Reference:** traffic-flow-logic.txt
