# Pedestrian Setup Guide for Traffic Simulation

## Overview

This guide explains how to add pedestrians to your Smart Traffic Light System simulation. We'll use the existing `gazebo-ros-actor-plugin` package that's already in your workspace.

## What's Already Available

✅ **gazebo-ros-actor-plugin** - Already cloned in `ros2_ws/src/`
✅ **DoctorFemaleWalk model** - Pedestrian model with walking animation
✅ **Crosswalks** - Already marked in your intersection.world file
✅ **Generated pedestrian XML** - Ready to add to your world file

## Step-by-Step Implementation

### Step 1: Copy Pedestrian Model to Gazebo Models Directory

The DoctorFemaleWalk model needs to be accessible to Gazebo:

```bash
# Create Gazebo models directory if it doesn't exist
mkdir -p ~/.gz/models

# Copy the pedestrian model
cp -r ros2_ws/src/gazebo-ros-actor-plugin/config/skins/DoctorFemaleWalk ~/.gz/models/

# Verify it's there
ls ~/.gz/models/DoctorFemaleWalk
```

### Step 2: Add Pedestrians to World File

The pedestrian XML has been generated in `pedestrian_actors.xml`. You need to add it to your intersection world file:

**Location:** `ros2_ws/src/smart_traffic_system/worlds/intersection.world`

**Where to add:** Before the closing `</world>` tag at the end of the file

**What to add:** The content from `pedestrian_actors.xml`

This adds 8 pedestrians:
- 2 at North crosswalk (positions: X=-8,Y=14 and X=8,Y=14)
- 2 at South crosswalk (positions: X=-8,Y=-14 and X=8,Y=-14)
- 2 at East crosswalk (positions: X=14,Y=-8 and X=14,Y=8)
- 2 at West crosswalk (positions: X=-14,Y=-8 and X=-14,Y=8)

### Step 3: Build the gazebo-ros-actor-plugin

```bash
cd ros2_ws
colcon build --packages-select gazebo_ros_actor_plugin
source install/setup.bash
```

### Step 4: Test Pedestrians (Static)

Launch the simulation to verify pedestrians appear:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch smart_traffic_system simulation.launch.py
```

You should see 8 pedestrian models standing at the crosswalks.

### Step 5: Create Pedestrian Controller (Next Phase)

After verifying pedestrians appear correctly, we'll create a controller to make them cross:

**File:** `ros2_ws/src/smart_traffic_system/scripts/pedestrian_controller.py`

**Features:**
- Subscribe to traffic light state
- Publish crossing paths for pedestrians
- Coordinate with traffic signals
- Make pedestrians cross when safe

### Step 6: Integrate with Traffic System

**Modifications needed:**
1. Update `traffic_flow.py` to detect pedestrians in crosswalks
2. Make vehicles stop for crossing pedestrians
3. Add pedestrian signal logic to `smart_traffic_manager.py`

## Pedestrian Positions Explained

### North Crosswalk (Y=10 to Y=13)
- `pedestrian_north_1`: X=-8, Y=14 (west side, facing west)
- `pedestrian_north_2`: X=8, Y=14 (east side, facing east)
- **Crossing direction:** East-West across the north crosswalk

### South Crosswalk (Y=-13 to Y=-10)
- `pedestrian_south_1`: X=-8, Y=-14 (west side, facing west)
- `pedestrian_south_2`: X=8, Y=-14 (east side, facing east)
- **Crossing direction:** East-West across the south crosswalk

### East Crosswalk (X=10 to X=13)
- `pedestrian_east_1`: X=14, Y=-8 (south side, facing south)
- `pedestrian_east_2`: X=14, Y=8 (north side, facing north)
- **Crossing direction:** North-South across the east crosswalk

### West Crosswalk (X=-13 to X=-10)
- `pedestrian_west_1`: X=-14, Y=-8 (south side, facing south)
- `pedestrian_west_2`: X=-14, Y=8 (north side, facing north)
- **Crossing direction:** North-South across the west crosswalk

## Pedestrian Crossing Logic (To Implement)

### When NS_GREEN (North/South vehicles have green):
- **East/West pedestrians CAN cross** (perpendicular to vehicle flow)
- **North/South pedestrians WAIT** (parallel to vehicle flow)

### When EW_GREEN (East/West vehicles have green):
- **North/South pedestrians CAN cross** (perpendicular to vehicle flow)
- **East/West pedestrians WAIT** (parallel to vehicle flow)

### When ALL_RED:
- **All pedestrians WAIT** (transition period)

## Crossing Paths

Each pedestrian will follow a path across their crosswalk:

**North Crosswalk:**
- pedestrian_north_1: (-8, 14) → (8, 14) [west to east]
- pedestrian_north_2: (8, 14) → (-8, 14) [east to west]

**South Crosswalk:**
- pedestrian_south_1: (-8, -14) → (8, -14) [west to east]
- pedestrian_south_2: (8, -14) → (-8, -14) [east to west]

**East Crosswalk:**
- pedestrian_east_1: (14, -8) → (14, 8) [south to north]
- pedestrian_east_2: (14, 8) → (14, -8) [north to south]

**West Crosswalk:**
- pedestrian_west_1: (-14, -8) → (-14, 8) [south to north]
- pedestrian_west_2: (-14, 8) → (-14, -8) [north to south]

## Files Created

1. **PEDESTRIAN_INTEGRATION_PLAN.md** - Overall integration plan
2. **pedestrian_actors.xml** - Generated XML for 8 pedestrians
3. **add_pedestrians_to_world.py** - Script that generated the XML
4. **download_pedestrian_models.sh** - Script to download more models (optional)
5. **PEDESTRIAN_SETUP_GUIDE.md** - This file

## Next Steps

1. ✅ Copy DoctorFemaleWalk model to ~/.gz/models/
2. ✅ Add pedestrian XML to intersection.world
3. ✅ Build gazebo_ros_actor_plugin
4. ✅ Test that pedestrians appear in simulation
5. ⏳ Create pedestrian_controller.py (next phase)
6. ⏳ Integrate with traffic light system (next phase)
7. ⏳ Add vehicle-pedestrian interaction (next phase)

## Troubleshooting

### Pedestrians don't appear
- Check that DoctorFemaleWalk is in ~/.gz/models/
- Verify gazebo_ros_actor_plugin is built
- Check Gazebo console for error messages

### Pedestrians appear but don't move
- This is expected initially - they need a controller
- We'll add the pedestrian_controller.py in the next phase

### Plugin not found error
- Make sure you sourced the workspace: `source install/setup.bash`
- Rebuild: `colcon build --packages-select gazebo_ros_actor_plugin`

## Additional Models (Optional)

To add more variety, you can download additional pedestrian models:

```bash
./download_pedestrian_models.sh
```

This will download models like:
- person_walking
- person_standing
- MaleVisitorPhone
- etc.

---

**Status:** Ready to implement
**Created:** December 30, 2025
