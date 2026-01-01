# 🔍 DEBUG INSTRUCTIONS - Traffic Light System

## Current Status

The traffic light STATE is being published (NS_GREEN), but individual light messages are NOT reaching the traffic_flow_controller. This means vehicles don't know when to stop.

## What I've Added

### 1. Debug Logging in smart_traffic_manager.py
- Logs when `_publish_individual_light_states()` is called
- Logs each individual light state being published
- This will help us see if the manager is actually sending messages

### 2. Debug Logging in traffic_flow.py
- Logs when individual light messages are RECEIVED
- Logs when vehicles are at stop lines and checking lights
- This will help us see if the controller is receiving messages

### 3. Debug Scripts
- `debug_traffic_system.sh` - Comprehensive system check
- `test_traffic_lights.sh` - Quick light state check

---

## Steps to Debug

### Step 1: Restart the Simulation

**IMPORTANT**: You MUST restart for the new debug logging to take effect!

```bash
# In the terminal running the simulation, press Ctrl+C to stop it
# Then run:
cd ~/robotics-project
./restart_with_fix.sh
```

### Step 2: Watch the Terminal Output

Look for these messages in the simulation terminal:

**From smart_traffic_manager:**
```
[smart_traffic_manager]: Publishing light states: {'north': 'GREEN', 'south': 'GREEN', 'east': 'RED', 'west': 'RED'}
[smart_traffic_manager]: Published NORTH: GREEN
[smart_traffic_manager]: Published SOUTH: GREEN
[smart_traffic_manager]: Published EAST: RED
[smart_traffic_manager]: Published WEST: RED
```

**From traffic_flow_controller:**
```
[traffic_flow_controller]: Received north light: GREEN
[traffic_flow_controller]: Received south light: GREEN
[traffic_flow_controller]: Received east light: RED
[traffic_flow_controller]: Received west light: RED
```

**When vehicles approach stop lines:**
```
[traffic_flow_controller]: car_s1: at_stop_line=True, light=RED
[traffic_flow_controller]: car_e2: at_stop_line=True, light=GREEN
```

### Step 3: Run Debug Script

In a SECOND terminal:
```bash
cd ~/robotics-project
./debug_traffic_system.sh
```

This will check:
- Are nodes running?
- Are topics connected?
- Are messages being published?
- Are vehicles being tracked?

---

## What to Look For

### Scenario A: Messages ARE Being Published

If you see the "Publishing light states" and "Published NORTH/SOUTH/EAST/WEST" messages:

✅ **Manager is working correctly**

Then check if controller is receiving:
- If you see "Received north/south/east/west light" messages → **System is working!**
- If you DON'T see "Received" messages → **Subscription problem**

### Scenario B: Messages are NOT Being Published

If you DON'T see "Publishing light states" messages:

❌ **update_lights() is not being called**

Possible causes:
1. `update_lights()` call in `__init__` didn't work
2. State machine is stuck
3. Timer is not running

### Scenario C: Vehicles Not Stopping Despite Receiving Messages

If you see both "Publishing" and "Received" messages, but vehicles still don't stop:

Check for "at_stop_line" messages:
- If you see them → **Stop line detection is working**
- If you don't → **Position tracking problem**

---

## Common Issues and Fixes

### Issue 1: No "Publishing light states" messages

**Cause**: `update_lights()` not being called

**Fix**: Check if the `__init__` method has this line:
```python
self.update_lights()  # Initialize lights to current state
```

### Issue 2: "Publishing" but no "Received" messages

**Cause**: Topic connection problem

**Fix**: Check topic info:
```bash
ros2 topic info /traffic/light/north
# Should show: Publisher count: 1, Subscription count: 1
```

### Issue 3: Messages received but vehicles don't stop

**Cause**: Position tracking or stop line detection not working

**Fix**: Check if vehicles are being tracked:
```bash
ros2 topic echo /sensors/north --once
# Should show vehicle models with positions
```

### Issue 4: Stop line detection not working

**Cause**: Stop line positions might be wrong

**Current stop lines**:
- North: Y = 15.0 (for southbound cars)
- South: Y = -15.0 (for northbound cars)
- East: X = 15.0 (for westbound cars)
- West: X = -15.0 (for eastbound cars)

**Check**: Are vehicles actually reaching these positions?

---

## Next Steps Based on Debug Output

### If you see "Publishing" messages:
→ Good! Manager is working. Check if controller receives them.

### If you see "Received" messages:
→ Excellent! Communication is working. Check stop line detection.

### If you see "at_stop_line=True" messages:
→ Perfect! Detection is working. Check if light state is correct.

### If vehicles STILL don't stop:
→ There might be an issue with the velocity control logic.

---

## Emergency Fallback: Simpler Stop Line Detection

If nothing works, we can try a simpler approach - stop ALL vehicles when they get close to intersection center:

```python
def _is_near_intersection(self, vehicle_name):
    pos = self._get_vehicle_position(vehicle_name)
    if pos is None:
        return False
    
    x, y, z = pos
    distance_to_center = (x**2 + y**2)**0.5
    
    # Stop if within 15m of center
    return distance_to_center < 15.0
```

But let's try the debug logging first to see what's actually happening!

---

## What to Send Me

After running the debug steps, please send:

1. **Terminal output** from the simulation (showing the log messages)
2. **Output** from `./debug_traffic_system.sh`
3. **Description** of what you observe:
   - Are lights changing colors in Gazebo?
   - Are vehicles stopping at all?
   - Are vehicles queuing or just driving through?

This will help me identify exactly where the problem is!

---

*Document created: December 30, 2025*
*Purpose: Debug traffic light system communication*
