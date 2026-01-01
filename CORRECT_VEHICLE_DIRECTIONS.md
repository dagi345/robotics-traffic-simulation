# ✅ CORRECT VEHICLE DIRECTIONS - REFERENCE DOCUMENT

## ⚠️ CRITICAL: DO NOT CHANGE THIS CONFIGURATION ⚠️

This document records the **WORKING** vehicle direction configuration.
All vehicles move correctly with this setup.

---

## Working Velocity Control Code

```python
# CORRECT CODE - DO NOT MODIFY
msg = Twist()
msg.linear.y = -velocity  # ALL vehicles use negative linear.y
pub.publish(msg)
```

### Key Points:
- ✅ ALL 16 vehicles use `msg.linear.y = -velocity`
- ✅ NO axis switching (no linear.x usage)
- ✅ NO sign changes based on direction
- ✅ Simple and consistent for all vehicles

---

## Vehicle Orientations in World File

### Southbound Vehicles (REFERENCE - WORKING)
```xml
<!-- car_s1, car_s2, car_s3, car_s4 -->
<pose>-2 30 0.01 0 0 0</pose>
<pose>-2 25 0.01 0 0 0</pose>
<pose>-5 35 0.01 0 0 0</pose>
<pose>-5 30 0.01 0 0 0</pose>
```
- **Rotation**: `0` (no rotation)
- **Starting Position**: Y=25 to Y=35 (north of intersection)
- **Movement**: Toward Y=0 (south, toward intersection)
- **Velocity**: `linear.y = -velocity`
- **Status**: ✅ WORKING CORRECTLY

### Northbound Vehicles
```xml
<!-- car_n1, car_n2, car_n3, car_n4 -->
<pose>2 -30 0.01 0 0 3.1415</pose>
<pose>2 -25 0.01 0 0 3.1415</pose>
<pose>5 -35 0.01 0 0 3.1415</pose>
<pose>5 -30 0.01 0 0 3.1415</pose>
```
- **Rotation**: `3.1415` (180 degrees)
- **Starting Position**: Y=-25 to Y=-35 (south of intersection)
- **Movement**: Toward Y=0 (north, toward intersection)
- **Velocity**: `linear.y = -velocity` (same as south)
- **Status**: ✅ WORKING CORRECTLY

### Westbound Vehicles
```xml
<!-- car_w1, car_w2, car_w3, car_w4 -->
<pose>30 2 0.01 0 0 -1.5707</pose>
<pose>25 2 0.01 0 0 -1.5707</pose>
<pose>35 5 0.01 0 0 -1.5707</pose>
<pose>30 5 0.01 0 0 -1.5707</pose>
```
- **Rotation**: `-1.5707` (-90 degrees)
- **Starting Position**: X=25 to X=35 (east of intersection)
- **Movement**: Toward X=0 (west, toward intersection)
- **Velocity**: `linear.y = -velocity` (same as south)
- **Status**: ✅ WORKING CORRECTLY

### Eastbound Vehicles
```xml
<!-- car_e1, car_e2, car_e3, car_e4 -->
<pose>-30 -2 0.01 0 0 1.5707</pose>
<pose>-25 -2 0.01 0 0 1.5707</pose>
<pose>-30 -5 0.01 0 0 1.5707</pose>
<pose>-25 -5 0.01 0 0 1.5707</pose>
```
- **Rotation**: `1.5707` (90 degrees)
- **Starting Position**: X=-25 to X=-35 (west of intersection)
- **Movement**: Toward X=0 (east, toward intersection)
- **Velocity**: `linear.y = -velocity` (same as south)
- **Status**: ✅ WORKING CORRECTLY

---

## How It Works

### The Magic of Rotation
The vehicle **rotation in the world file** determines which direction the vehicle faces.
When we apply `linear.y = -velocity`, the vehicle moves **forward** in its facing direction.

- **South (rotation=0)**: Faces south, moves south
- **North (rotation=180°)**: Faces north, moves north
- **West (rotation=-90°)**: Faces west, moves west
- **East (rotation=90°)**: Faces east, moves east

### Why This Is Simple
- ✅ One velocity command works for all vehicles
- ✅ No complex axis switching logic
- ✅ Vehicle orientation handles the direction
- ✅ Easy to understand and maintain

---

## Traffic Light Control

### NS_GREEN State
- **North vehicles**: Move (green light)
- **South vehicles**: Move (green light)
- **East vehicles**: Stop (red light)
- **West vehicles**: Stop (red light)

### EW_GREEN State
- **North vehicles**: Stop (red light)
- **South vehicles**: Stop (red light)
- **East vehicles**: Move (green light)
- **West vehicles**: Move (green light)

### ALL_RED State
- **All vehicles**: Stop (red light)

---

## Position Tracking (Integrated)

### Lane Mappings
```python
self.vehicle_lanes = {
    'car_n1': 'north', 'car_n2': 'north', 'car_n3': 'north', 'car_n4': 'north',
    'car_s1': 'south', 'car_s2': 'south', 'car_s3': 'south', 'car_s4': 'south',
    'car_e1': 'east', 'car_e2': 'east', 'car_e3': 'east', 'car_e4': 'east',
    'car_w1': 'west', 'car_w2': 'west', 'car_w3': 'west', 'car_w4': 'west',
}
```

### Distance Calculations
- **South lane**: ahead = lower Y coordinate
- **North lane**: ahead = higher Y coordinate
- **West lane**: ahead = lower X coordinate
- **East lane**: ahead = higher X coordinate

---

## Critical Fixes Applied

### 1. ✅ Position Tracking
- Integrated directly into `MultiTrafficFlowControl`
- No separate node instantiation
- Updates from sensor callbacks
- Collision avoidance working

### 2. ✅ Duplicate Positions Fixed
- car_w2: Changed from (25, 7) to (25, 2)
- car_w3: Changed from (30, 12) to (35, 5)
- car_w4: Changed from (25, 7) to (30, 5)
- All vehicles have unique positions

### 3. ✅ Velocity Control
- Kept original: `msg.linear.y = -velocity`
- Works for all vehicles
- Simple and consistent

---

## ⚠️ IMPORTANT RULES ⚠️

### DO NOT:
- ❌ Change the velocity control code
- ❌ Add axis switching logic (linear.x vs linear.y)
- ❌ Add sign changes based on direction
- ❌ Modify vehicle rotations in world file
- ❌ Change the position tracking lane mappings

### DO:
- ✅ Keep `msg.linear.y = -velocity` for all vehicles
- ✅ Use vehicle rotation in world file to control direction
- ✅ Maintain the integrated position tracking
- ✅ Keep unique vehicle starting positions

---

## Testing Checklist

After any changes, verify:
- [ ] South cars move toward intersection (Y decreasing)
- [ ] North cars move toward intersection (Y increasing)
- [ ] West cars move toward intersection (X decreasing)
- [ ] East cars move toward intersection (X increasing)
- [ ] Cars stop at red lights
- [ ] Cars proceed on green lights
- [ ] No collisions at intersection
- [ ] Traffic lights cycle properly

---

## Build and Test Commands

```bash
cd ros2_ws
colcon build --packages-select smart_traffic_system
source install/setup.bash
ros2 launch smart_traffic_system simulation.launch.py
```

---

## Summary

**The system works because:**
1. All vehicles use the same velocity command: `linear.y = -velocity`
2. Vehicle rotation in world file determines facing direction
3. Position tracking is integrated and functional
4. No duplicate vehicle positions
5. Traffic lights properly control movement

**This configuration is PROVEN to work. Do not modify it.**

---

*Document created: December 30, 2025*
*Status: ✅ WORKING CONFIGURATION*
*Last verified: After restoring original velocity control*
