# Traffic Light Control Diagnostic

**Date:** December 31, 2025  
**Purpose:** Diagnose why vehicles are not stopping at red lights

---

## Current Configuration

### Stop Line Coordinates (FIXED)
```python
self.STOP_LINE_NORTH = 13.0   # Y coordinate for southbound stop
self.STOP_LINE_SOUTH = -13.0  # Y coordinate for northbound stop
self.STOP_LINE_EAST = 13.0    # X coordinate for westbound stop
self.STOP_LINE_WEST = -13.0   # X coordinate for eastbound stop
```

### Stop Line Detection Logic
```python
# Southbound: Detect from Y=18 to Y=13
at_line = y <= 18.0 and y > self.STOP_LINE_NORTH  # y > 13

# Northbound: Detect from Y=-18 to Y=-13
at_line = y >= -18.0 and y < self.STOP_LINE_SOUTH  # y < -13

# Westbound: Detect from X=18 to X=13
at_line = x <= 18.0 and x > self.STOP_LINE_EAST  # x > 13

# Eastbound: Detect from X=-18 to X=-13
at_line = x >= -18.0 and x < self.STOP_LINE_WEST  # x < -13
```

### Traffic Light Stop Logic
```python
def _should_stop_for_light(self, vehicle_name):
    light_state = self._get_controlling_light(vehicle_name)
    at_stop_line = self._is_at_stop_line(vehicle_name)
    
    # RED: Always stop if at stop line
    if light_state == 'RED' and at_stop_line:
        return True
    
    # YELLOW: Stop if at stop line
    if light_state == 'YELLOW' and at_stop_line:
        return True
    
    # GREEN: Don't stop
    return False
```

---

## Potential Issues

### Issue #1: Light State Initialization
```python
self.light_states = {
    'north': 'RED',  # Controls southbound vehicles
    'south': 'RED',  # Controls northbound vehicles
    'east': 'RED',   # Controls westbound vehicles
    'west': 'RED',   # Controls eastbound vehicles
}
```

**Problem:** All lights start as RED. If traffic manager doesn't publish states immediately, vehicles might think all lights are RED forever.

**Test:**
```bash
ros2 topic echo /traffic/light/north
```

**Expected:** Should see GREEN/YELLOW/RED cycling

### Issue #2: Light Mapping Logic
```python
# Map vehicle direction to controlling light
light_mapping = {
    's': 'north',  # Southbound → look at NORTH light
    'n': 'south',  # Northbound → look at SOUTH light
    'w': 'east',   # Westbound → look at EAST light
    'e': 'west',   # Eastbound → look at WEST light
}
```

**Question:** Is this mapping correct? Do southbound vehicles really look at the NORTH light?

**In real traffic:**
- Southbound vehicles approach from north, look at light BEFORE intersection
- The light they see is on the SOUTH side of intersection
- So southbound vehicles should look at SOUTH light, not NORTH light

**Possible Fix:**
```python
light_mapping = {
    's': 'south',  # Southbound → look at SOUTH light (before intersection)
    'n': 'north',  # Northbound → look at NORTH light (before intersection)
    'w': 'west',   # Westbound → look at WEST light (before intersection)
    'e': 'east',   # Eastbound → look at EAST light (before intersection)
}
```

### Issue #3: Stop Line Detection Range
```python
# Southbound example:
at_line = y <= 18.0 and y > self.STOP_LINE_NORTH  # y > 13
```

**Problem:** This detects vehicles from Y=18 down to Y=13.01, but NOT at Y=13 exactly.

**When vehicle is at Y=13.0:**
- `y <= 18.0` → True
- `y > 13.0` → False (13.0 is NOT greater than 13.0)
- Result: `at_line = False`

**Fix:** Should be `y >= self.STOP_LINE_NORTH` instead of `y > self.STOP_LINE_NORTH`

```python
# Southbound: Detect from Y=18 to Y=13 (inclusive)
at_line = y <= 18.0 and y >= self.STOP_LINE_NORTH

# Northbound: Detect from Y=-18 to Y=-13 (inclusive)
at_line = y >= -18.0 and y <= self.STOP_LINE_SOUTH

# Westbound: Detect from X=18 to X=13 (inclusive)
at_line = x <= 18.0 and x >= self.STOP_LINE_EAST

# Eastbound: Detect from X=-18 to X=-13 (inclusive)
at_line = x >= -18.0 and x <= self.STOP_LINE_WEST
```

### Issue #4: Traffic Manager State Machine
Check if traffic manager is actually cycling through states:
```bash
ros2 topic echo /traffic_lights/state
```

**Expected:** Should see:
- NS_GREEN
- NS_YELLOW
- ALL_RED
- EW_GREEN
- EW_YELLOW
- ALL_RED
- (repeat)

---

## Diagnostic Steps

### Step 1: Check Traffic Light Topics
```bash
# Terminal 1: Launch simulation
ros2 launch smart_traffic_system simulation.launch.py

# Terminal 2: Monitor traffic lights
ros2 topic echo /traffic/light/north
ros2 topic echo /traffic/light/south
ros2 topic echo /traffic/light/east
ros2 topic echo /traffic/light/west
```

**What to look for:**
- Are topics publishing?
- Do states change from RED to GREEN to YELLOW?
- What is the cycle time?

### Step 2: Check Vehicle Detection at Stop Lines
```bash
# Watch logs for stop line detection messages
# Should see messages like:
# "Vehicle car_s1 (southbound): Y=14.50, at_line=True"
```

**What to look for:**
- Are vehicles being detected at stop lines?
- At what Y/X coordinates?
- Is detection happening too early or too late?

### Step 3: Check Stop Logic Triggering
```bash
# Watch logs for stop logic messages
# Should see messages like:
# "Vehicle car_s1: at_stop_line=True, light=RED"
```

**What to look for:**
- Is stop logic being triggered?
- What light state is reported?
- Is vehicle actually stopping (velocity=0)?

### Step 4: Monitor Vehicle Velocities
```bash
ros2 topic echo /model/car_s1/cmd_vel
```

**What to look for:**
- Does velocity go to 0 when at red light?
- Does velocity resume when light turns green?

---

## Recommended Fixes

### Fix #1: Correct Light Mapping
```python
# CURRENT (POSSIBLY WRONG):
light_mapping = {
    's': 'north',  # Southbound → NORTH light
    'n': 'south',  # Northbound → SOUTH light
    'w': 'east',   # Westbound → EAST light
    'e': 'west',   # Eastbound → WEST light
}

# PROPOSED (LOGICAL):
light_mapping = {
    's': 'south',  # Southbound → SOUTH light (before intersection)
    'n': 'north',  # Northbound → NORTH light (before intersection)
    'w': 'west',   # Westbound → WEST light (before intersection)
    'e': 'east',   # Eastbound → EAST light (before intersection)
}
```

### Fix #2: Inclusive Stop Line Detection
```python
# Southbound
at_line = y <= 18.0 and y >= self.STOP_LINE_NORTH  # Include Y=13

# Northbound
at_line = y >= -18.0 and y <= self.STOP_LINE_SOUTH  # Include Y=-13

# Westbound
at_line = x <= 18.0 and x >= self.STOP_LINE_EAST  # Include X=13

# Eastbound
at_line = x >= -18.0 and x <= self.STOP_LINE_WEST  # Include X=-13
```

### Fix #3: Add Debug Logging
Add more detailed logging to understand what's happening:
```python
def _should_stop_for_light(self, vehicle_name):
    light_state = self._get_controlling_light(vehicle_name)
    at_stop_line = self._is_at_stop_line(vehicle_name)
    pos = self._get_vehicle_position(vehicle_name)
    
    # Always log when near intersection
    if pos and abs(pos[0]) < 20 and abs(pos[1]) < 20:
        self.get_logger().info(
            f"{vehicle_name}: pos=({pos[0]:.1f}, {pos[1]:.1f}), "
            f"at_line={at_stop_line}, light={light_state}")
    
    # ... rest of logic
```

---

## Testing Plan

1. **Test Light Mapping:**
   - Launch simulation
   - Watch which vehicles move during NS_GREEN
   - Should be: Northbound and Southbound
   - If East/West move instead, mapping is wrong

2. **Test Stop Line Detection:**
   - Watch logs for "at_line=True" messages
   - Verify they appear at correct coordinates
   - Verify vehicles actually stop

3. **Test Traffic Light Compliance:**
   - Watch vehicles approach red lights
   - Verify they stop at stop lines
   - Verify they proceed on green

---

## Summary

**Most Likely Issues:**
1. ❓ Light mapping might be inverted
2. ❓ Stop line detection excludes exact stop line position
3. ❓ Traffic light states not being published/received

**Next Steps:**
1. Run diagnostic commands above
2. Apply recommended fixes
3. Test with simulation

