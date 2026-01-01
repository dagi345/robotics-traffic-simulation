# Pedestrian Integration Plan

## Overview

This plan outlines how to add pedestrians to the Smart Traffic Light System simulation using the existing `gazebo-ros-actor-plugin` package.

## Current Status

✅ **Already Available:**
- `gazebo-ros-actor-plugin` package cloned and available in `ros2_ws/src/`
- DoctorFemaleWalk pedestrian model available
- Actor control plugin supports both velocity and path-following modes

## Integration Steps

### Phase 1: Add Pedestrian Models to Intersection World

**Goal:** Add pedestrians at crosswalks in the intersection simulation

**Tasks:**
1. Add pedestrian actors to `intersection.world` file
2. Position pedestrians at crosswalk locations (4 crosswalks: N, S, E, W)
3. Configure pedestrian walking animations
4. Set up pedestrian crossing paths

**Crosswalk Positions:**
- North crosswalk: X=0, Y=±13 (crossing east-west)
- South crosswalk: X=0, Y=±13 (crossing east-west)
- East crosswalk: X=±13, Y=0 (crossing north-south)
- West crosswalk: X=±13, Y=0 (crossing north-south)

### Phase 2: Download Additional Pedestrian Models

**Goal:** Add variety to pedestrian appearances

**Available Gazebo Models:**
- `actor://walk` (default walking person)
- `actor://stand` (standing person)
- `actor://run` (running person)
- Custom models from Gazebo Fuel or GitHub

**Recommended Models to Add:**
1. Male pedestrian models
2. Different clothing/appearance variations
3. Different walking speeds

### Phase 3: Create Pedestrian Control Node

**Goal:** Control pedestrian crossing behavior based on traffic signals

**Features:**
- Subscribe to traffic light state
- Control when pedestrians cross
- Implement "Walk/Don't Walk" signals
- Coordinate with vehicle traffic

**Logic:**
- When NS_GREEN: East/West pedestrians can cross
- When EW_GREEN: North/South pedestrians can cross
- When ALL_RED: No pedestrians cross (transition period)

### Phase 4: Add Pedestrian Signals

**Goal:** Visual pedestrian crossing signals at each crosswalk

**Implementation:**
- Add pedestrian signal models (red hand / green walking person)
- Sync with traffic light states
- Position at each crosswalk

### Phase 5: Implement Pedestrian-Vehicle Interaction

**Goal:** Vehicles must yield to pedestrians in crosswalks

**Features:**
- Detect pedestrians in crosswalk zones
- Stop vehicles when pedestrians are crossing
- Resume vehicle movement when crosswalk is clear

## File Structure

```
ros2_ws/src/smart_traffic_system/
├── scripts/
│   ├── traffic_flow.py (existing - will modify)
│   ├── smart_traffic_manager.py (existing - will modify)
│   └── pedestrian_controller.py (NEW)
├── worlds/
│   └── intersection.world (will modify - add actors)
└── launch/
    └── simulation.launch.py (will modify - launch pedestrian controller)
```

## Implementation Order

1. **First:** Add basic pedestrian actors to world file (static positions)
2. **Second:** Test that actors load and render correctly
3. **Third:** Add pedestrian crossing paths
4. **Fourth:** Create pedestrian controller node
5. **Fifth:** Integrate with traffic light system
6. **Sixth:** Add vehicle-pedestrian interaction

## Technical Details

### Actor Configuration Example

```xml
<include>
  <name>pedestrian_north_1</name>
  <pose>0 15 0 0 0 -1.57</pose>
  <uri>model://DoctorFemaleWalk</uri>
  <plugin filename="libgazebo_ros_actor_plugin.so" 
    name="gazebo_ros_actor_plugin::GazeboRosActorCommand">
    <follow_mode>path</follow_mode>
    <path_topic>/pedestrian_north_1/cmd_path</path_topic>
    <linear_velocity>1.0</linear_velocity>
    <animation_factor>4.0</animation_factor>
  </plugin>
</include>
```

### Pedestrian Controller Topics

**Subscribed:**
- `/traffic_lights/state` - Current traffic light state
- `/traffic_lights/pedestrian_signals` - Pedestrian signal states

**Published:**
- `/pedestrian_{id}/cmd_path` - Path commands for each pedestrian
- `/crosswalk/occupancy` - Which crosswalks have pedestrians

## Safety Considerations

1. **Collision Detection:** Vehicles must detect pedestrians in crosswalks
2. **Timing:** Pedestrian crossing time must be sufficient
3. **Clear Zones:** Ensure intersection is clear before allowing crossing
4. **Emergency Stop:** Vehicles must stop if pedestrian enters unexpectedly

## Testing Plan

1. **Visual Test:** Verify pedestrians appear and animate correctly
2. **Path Test:** Verify pedestrians follow crossing paths
3. **Signal Test:** Verify pedestrians cross only on correct signals
4. **Vehicle Test:** Verify vehicles stop for crossing pedestrians
5. **Integration Test:** Full simulation with vehicles and pedestrians

## Next Steps

**Immediate Actions:**
1. Review this plan with user
2. Start with Phase 1: Add pedestrian actors to world file
3. Test basic pedestrian rendering
4. Proceed to pedestrian control logic

---

**Status:** Ready to implement
**Created:** December 30, 2025
