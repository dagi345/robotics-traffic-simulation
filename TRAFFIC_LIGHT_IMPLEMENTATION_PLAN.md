# Traffic Light System Implementation Plan

## Current Issues

1. ❌ **Vehicles don't obey traffic lights** - They enter intersection regardless of signal
2. ❌ **Stop line positioning unclear** - Need precise 2m before crosswalk
3. ❌ **Light-to-vehicle mapping incorrect** - Vehicles should look at light ACROSS intersection
4. ❌ **Collisions occurring** - Due to lack of signal enforcement

## Implementation Strategy

### Phase 1: Fix Stop Line Positions
- Define stop lines at ~2m before crosswalk (Y=±15m, X=±15m)
- Update vehicle detection zones to match stop lines
- Ensure vehicles stop BEFORE crosswalk

### Phase 2: Implement Correct Light-to-Vehicle Mapping
**Critical Rule:** Vehicles look at light ACROSS intersection

- Southbound vehicles (car_s*) → controlled by NORTH light
- Northbound vehicles (car_n*) → controlled by SOUTH light
- Westbound vehicles (car_w*) → controlled by EAST light
- Eastbound vehicles (car_e*) → controlled by WEST light

### Phase 3: Enforce Strict Vehicle Compliance
- Subscribe to individual light topics per vehicle
- Stop immediately on RED or YELLOW
- Only proceed on GREEN
- Never enter intersection on RED

### Phase 4: Update Traffic Manager
- Publish individual light states per direction
- Ensure proper phase synchronization
- Maintain state machine: GREEN → YELLOW → ALL_RED → GREEN

### Phase 5: Visual Light Behavior
- Bright active color
- Dim inactive colors
- Clear visual feedback

## Implementation Files

1. **traffic_flow.py** - Add strict signal compliance
2. **smart_traffic_manager.py** - Publish individual light states
3. **intersection.world** - Verify stop line positions
4. **New: traffic_light_controller.py** - Centralized controller

## Success Criteria

✅ Vehicles stop at stop lines (2m before crosswalk)
✅ Vehicles obey their controlling light
✅ No collisions in intersection
✅ Proper phase synchronization
✅ Clear visual light states
