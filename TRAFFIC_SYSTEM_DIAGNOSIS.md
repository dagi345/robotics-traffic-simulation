# Traffic System Diagnosis Report
## Date: December 30, 2025

## Problem Statement
Cars from different directions are crashing at the intersection, indicating the traffic flow control system is not working correctly.

## Root Cause Analysis

### CRITICAL BUG #1: Incorrect Velocity Axis for ALL Vehicles ⚠️
**Location:** `traffic_flow.py` line 234
**Severity:** CRITICAL - This is the PRIMARY cause of crashes

```python
# CURRENT CODE (WRONG):
msg.linear.y = -velocity  # ALL vehicles use Y axis
```

**Problem:**
- ALL 16 vehicles are being controlled using `linear.y` axis
- This is INCORRECT for east/west vehicles
- East/west vehicles should use `linear.x` axis

**Vehicle Orientations from World File:**
- **Northbound** (car_n1-4): rotation=3.1415 (180°) → moves in +Y direction → needs `linear.y = +velocity`
- **Southbound** (car_s1-4): rotation=0 → moves in -Y direction → needs `linear.y = -velocity`
- **Eastbound** (car_e1-4): rotation=1.5707 (90°) → moves in +X direction → needs `linear.x = +velocity`
- **Westbound** (car_w1-4): rotation=-1.5707 (-90°) → moves in -X direction → needs `linear.x = -velocity`

**Impact:**
- East/west vehicles are NOT moving correctly
- They may be moving in wrong directions or not at all
- This causes intersection collisions because vehicles don't respond to traffic lights properly

---

### CRITICAL BUG #2: Sensor Direction Mapping Confusion
**Location:** `traffic_flow.py` lines 119-130
**Severity:** HIGH

```python
# CURRENT CODE (CONFUSING):
self.create_subscription(
    LogicalCameraImage, '/sensors/north',
    lambda msg: self.sensor_cb(msg, 's'), 10)  # North sensor → 's' direction?
```

**Problem:**
- Sensor at position Y=20 (north) is mapped to direction 's' (south)
- This is counter-intuitive and may cause logic errors
- The mapping appears to be "sensor detects traffic FROM that direction"

**Clarification Needed:**
- sensor_north_road (Y=20) detects southbound traffic (cars moving south)
- sensor_south_road (Y=-20) detects northbound traffic (cars moving north)
- sensor_east_road (X=20) detects westbound traffic (cars moving west)
- sensor_west_road (X=-20) detects eastbound traffic (cars moving east)

**Impact:**
- Vehicles may be assigned to wrong detection zones
- Red light logic may apply to wrong vehicles
- Causes vehicles to not stop when they should

---

### CRITICAL BUG #3: Vehicle Position Tracker Not Instantiated as ROS Node
**Location:** `traffic_flow.py` line 52
**Severity:** HIGH

```python
# CURRENT CODE:
self.position_tracker = VehiclePositionTracker()
```

**Problem:**
- `VehiclePositionTracker` is a ROS Node (inherits from `Node`)
- It's being instantiated as a regular object inside another node
- ROS nodes should NOT be instantiated this way
- The tracker's subscriptions won't work properly

**Correct Approach:**
- Either make VehiclePositionTracker a standalone node (launched separately)
- OR refactor it to be a helper class (not inheriting from Node)
- OR integrate its functionality directly into traffic_flow.py

**Impact:**
- Vehicle position tracking may not work
- Collision avoidance logic fails
- Distance calculations return infinity
- Vehicles don't maintain safe following distances

---

### BUG #4: Duplicate Vehicle Positions in World File
**Location:** `intersection.world` lines for car_w2 and car_w4
**Severity:** MEDIUM

```xml
<!-- car_w2 -->
<pose>25 7 0.01 0 0 -1.5707</pose>

<!-- car_w4 -->
<pose>25 7 0.01 0 0 -1.5707</pose>  <!-- SAME POSITION! -->
```

**Problem:**
- car_w2 and car_w4 start at EXACTLY the same position
- This causes immediate collision at startup
- Vehicles overlap and physics engine may behave unpredictably

**Impact:**
- Westbound lane starts with collision
- May cause cascading failures in vehicle control

---

### BUG #5: Inconsistent Vehicle Naming Convention
**Location:** World file and Python code
**Severity:** LOW-MEDIUM

**World File Uses:** `car_n1`, `car_s1`, `car_e1`, `car_w1`
**Python Code Expects:** Same naming

**Problem in vehicle_position_tracker.py:**
```python
# Code checks for 'car_' prefix
if model.name.startswith('car_'):
```

This is correct, but the lane mapping is hardcoded:
```python
for i in range(1, 5):
    self.vehicle_lanes[f'car_n{i}'] = 'north'
```

**Impact:**
- Works for current setup
- But fragile if vehicle names change
- Should be more robust

---

### BUG #6: Detection Zone Coverage May Be Insufficient
**Location:** World file sensor positions
**Severity:** MEDIUM

**Current Sensor Positions:**
- North sensor: Y=20, range=15m → covers Y=5 to Y=35
- South sensor: Y=-20, range=15m → covers Y=-35 to Y=-5
- East sensor: X=20, range=15m → covers X=5 to X=35
- West sensor: X=-20, range=15m → covers X=-35 to X=-5

**Stop Lines Are At:** ±13m

**Problem:**
- Sensors cover Y=5 to Y=35 (north)
- Stop line is at Y=13
- Gap between Y=5 and Y=13 where vehicles might not be detected
- Vehicles approaching from Y=0 to Y=5 are NOT in detection zone

**Impact:**
- Vehicles may enter intersection without being detected
- Red light logic doesn't apply to vehicles close to intersection
- Causes crashes when vehicles don't stop

---

### BUG #7: No Velocity Ramping/Smoothing
**Location:** `traffic_flow.py` timer_callback
**Severity:** LOW

**Current Behavior:**
- Velocity changes instantly from full speed to zero
- No gradual acceleration or deceleration
- Unrealistic and may cause physics issues

**Impact:**
- Vehicles stop abruptly
- May cause rear-end collisions
- Unrealistic simulation behavior

---

## System Architecture Issues

### Issue #1: VehiclePositionTracker Architecture
The current design has `VehiclePositionTracker` as a Node but instantiates it inside another Node. This violates ROS2 best practices.

**Options:**
1. Make it a standalone node (launched separately)
2. Convert it to a helper class (remove Node inheritance)
3. Merge functionality into traffic_flow.py

### Issue #2: Sensor Topic Naming Inconsistency
- Lane sensors: `/sensors/north`, `/sensors/south`, etc. (no topic specified in world)
- Intersection sensor: `/sensors/intersection_zone` (explicit topic)
- Pedestrian sensors: `/sensors/pedestrian_north`, etc. (explicit topic)

**Problem:** Default topic names may not match expectations

---

## Testing Gaps

### Missing Tests:
1. No test verifying vehicles use correct velocity axis
2. No test for sensor-to-vehicle direction mapping
3. No integration test for complete traffic cycle
4. No test for vehicle starting positions (duplicates)

---

## Priority Fix List

### IMMEDIATE (Must Fix to Stop Crashes):
1. **Fix velocity axis mapping** - Use linear.x for E/W, linear.y for N/S
2. **Fix VehiclePositionTracker instantiation** - Make it work properly
3. **Fix duplicate vehicle positions** - Separate car_w2 and car_w4
4. **Verify sensor detection zones** - Ensure vehicles are detected before stop line

### HIGH PRIORITY (Improve Safety):
5. **Add velocity ramping** - Smooth acceleration/deceleration
6. **Strengthen red light enforcement** - Ensure vehicles ALWAYS stop
7. **Add stop line position checking** - Verify vehicles stop before ±13m

### MEDIUM PRIORITY (Improve Realism):
8. **Better queue management** - Vehicles should form orderly queues
9. **Improve intersection clearance logic** - More robust zone monitoring
10. **Add visual debugging** - Show detection zones, vehicle states

---

## Recommended Testing Approach

### Phase 1: Unit Tests
- Test velocity axis selection logic
- Test sensor direction mapping
- Test vehicle lane assignment

### Phase 2: Integration Tests
- Test complete traffic light cycle
- Test vehicle stopping at red lights
- Test intersection clearance

### Phase 3: Simulation Tests
- Run full simulation
- Monitor for collisions
- Verify traffic flow

---

## Next Steps

1. Create a detailed fix plan with specific tasks
2. Implement fixes in priority order
3. Test each fix individually
4. Run full integration test
5. Validate in simulation

